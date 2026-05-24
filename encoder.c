#include "encoder.h"

#include <stdio.h>

#include <stdlib.h>

#include <libavutil/hwcontext_drm.h>

#include <drm/drm_fourcc.h>

#include <libavutil/imgutils.h>

#include <libavformat/avformat.h>

#include <libavutil/dict.h>

#include <sys/mman.h>

#include <time.h>

#include <RgaApi.h>

#include <im2d.h>

#include "osd_cache.h"



#define ALIGN_TO_2(x) ((x + 1) & ~1)



static int64_t get_mono_time_ms(void) {

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

}



static int64_t g_stream_start_ms = 0;



int streamer_init(FFmpegStreamer *s, const char *filename, int width, int height, int fps)

{

    const AVCodec *codec;

    int ret;



    s->width = width;

    s->height = height;

    s->frame_pts = 0;

    

    g_stream_start_ms = get_mono_time_ms();



    avformat_alloc_output_context2(&s->fmt_ctx, NULL, "flv", filename);

    if (!s->fmt_ctx) {

        fprintf(stderr, "Could not allocate output context\n");

        return -1;

    }



    // ==== 核心抗延迟配置 1：强行要求 FFmpeg 不做任何包积压 ====

    s->fmt_ctx->flags |= AVFMT_FLAG_FLUSH_PACKETS; 
    s->fmt_ctx->max_delay = 0; 



    codec = avcodec_find_encoder_by_name("h264_rkmpp");

    if (!codec) {

        fprintf(stderr, "Codec not found\n");

        return -1;

    }



    s->video_st = avformat_new_stream(s->fmt_ctx, codec);

    if (!s->video_st) {

        fprintf(stderr, "Could not create stream\n");

        return -1;

    }



    s->enc_ctx = avcodec_alloc_context3(codec);

    if (!s->enc_ctx) {

        fprintf(stderr, "Could not allocate codec context\n");

        return -1;

    }


    s->enc_ctx->width = width;
    s->enc_ctx->height = height;
    s->enc_ctx->time_base = (AVRational){1, fps};
    s->enc_ctx->framerate = (AVRational){fps, 1};
    
    // ========================================================
    // 【抗刷屏优化开始】
    // 1. 修改 GOP 大小为 60（约两秒一个I帧），减少频繁的大包刷新导致的卡顿
    s->enc_ctx->gop_size = fps; 
    
    s->enc_ctx->max_b_frames = 0;
    s->enc_ctx->pix_fmt = AV_PIX_FMT_NV12;
    
   // 1. 基础码率提升到 3Mbps，保证日常清晰度
    s->enc_ctx->bit_rate = 2000000; 
    
    // 2. 【核心】最大码率放宽到 6Mbps，允许在摄像头剧烈移动时“爆发”码流，吃透动态画面！
    s->enc_ctx->rc_max_rate = 2000000; 
    s->enc_ctx->rc_buffer_size = 300000;

    // 3. 【核心】放宽最高压缩比 (qmax)
    // qmin 保持 18（保证静止画面极其清晰）
    // qmax 必须拉高到 45（如果拉得太低比如32，剧烈运动时码率会超标，硬编引擎会直接报错花屏，拉高后最多是变柔和，绝不花屏撕裂）
    s->enc_ctx->qmin = 18;
    s->enc_ctx->qmax = 45;
    s->video_st->time_base = s->enc_ctx->time_base;
    // ========================================================

    if (s->fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
        s->enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }



    AVDictionary *codec_opts = NULL;
    av_dict_set(&codec_opts, "tune", "zerolatency", 0);
    av_dict_set(&codec_opts, "rc_mode", "CBR", 0);
    av_dict_set(&codec_opts, "profile", "baseline", 0);
    av_dict_set(&codec_opts, "bf", "0", 0);
    av_dict_set(&codec_opts, "delay", "0", 0);

    if (avcodec_open2(s->enc_ctx, codec, &codec_opts) < 0) {

        fprintf(stderr, "Could not open codec\n");
        av_dict_free(&codec_opts);

        return -1;

    }
    av_dict_free(&codec_opts);

    avcodec_parameters_from_context(s->video_st->codecpar, s->enc_ctx);



    s->sws_ctx = NULL;



    s->yuv_frame = av_frame_alloc();

    if (!s->yuv_frame) {

        fprintf(stderr, "Could not allocate frame\n");

        return -1;

    }



    s->yuv_frame->format = AV_PIX_FMT_NV12;

    s->yuv_frame->width = width;

    s->yuv_frame->height = 768;



    if (av_frame_get_buffer(s->yuv_frame, 64) < 0) {

        fprintf(stderr, "Could not allocate frame data.\n");

        return -1;

    }



    s->yuv_frame->height = height;



    if (!(s->fmt_ctx->oformat->flags & AVFMT_NOFILE)) {

        AVDictionary *opts = NULL;

        av_dict_set(&opts, "rw_timeout", "2000000", 0); 

        // ==== 核心抗延迟配置 2：禁用网络底层写缓冲 ====

        av_dict_set(&opts, "fflags", "nobuffer", 0); 
        av_dict_set(&opts, "flush_packets", "1", 0);
        av_dict_set(&opts, "tcp_nodelay", "1", 0); 

        

        if (avio_open2(&s->fmt_ctx->pb, filename, AVIO_FLAG_WRITE, NULL, &opts) < 0) {

            fprintf(stderr, "Could not open output file\n");

            av_dict_free(&opts);

            return -1;

        }

        av_dict_free(&opts);

    }



    ret = avformat_write_header(s->fmt_ctx, NULL);

    if (ret < 0) {

        fprintf(stderr, "Could not write stream header: %d\n", ret);

        return -1;

    }



    return 0;

}



int streamer_push(FFmpegStreamer *s, uint8_t *nv12_data)

{

    int ret;

    AVPacket *pkt = av_packet_alloc();

    if (!pkt) return -1;



    s->yuv_frame->data[0] = nv12_data;

    s->yuv_frame->linesize[0] = s->width;

    s->yuv_frame->data[1] = nv12_data + s->width * s->height;

    s->yuv_frame->linesize[1] = s->width;

    

    int fps = s->enc_ctx->framerate.num > 0 ? s->enc_ctx->framerate.num : 30;

    int64_t now_ms = get_mono_time_ms();

    int64_t target_pts = (now_ms - g_stream_start_ms) * fps / 1000;

    if (target_pts <= s->frame_pts) {

        target_pts = s->frame_pts + 1;

    }

    s->frame_pts = target_pts;

    s->yuv_frame->pts = s->frame_pts;



    ret = avcodec_send_frame(s->enc_ctx, s->yuv_frame);

    if (ret < 0) {

        av_packet_free(&pkt);

        return -1;

    }



    while (avcodec_receive_packet(s->enc_ctx, pkt) == 0) {

        av_packet_rescale_ts(pkt, s->enc_ctx->time_base, s->video_st->time_base);

        pkt->stream_index = s->video_st->index;

        

        // ==== 核心抗延迟配置 3：不再交织等待音频，直接暴力推向网络 ====

        av_write_frame(s->fmt_ctx, pkt);

        

        av_packet_unref(pkt);

    }



    av_packet_free(&pkt);

    return 0;

}



static int clamp_int(int value, int min_value, int max_value) {

    if (value < min_value) return min_value;

    if (value > max_value) return max_value;

    return value;

}



static int align_even_down(int value) { return value & ~1; }

static int align_even_up(int value) { return (value + 1) & ~1; }



static void draw_detect_boxes(FFmpegStreamer *s, const DetectSharedState *detect_state) {

    rga_buffer_t dst;

    int count;

    int i;



    if (s == NULL || detect_state == NULL || !detect_state->valid || detect_state->box_count <= 0) return;



    count = detect_state->box_count;

    if (count > DETECT_MAX_BOXES) count = DETECT_MAX_BOXES;



    dst = wrapbuffer_virtualaddr(s->yuv_frame->data[0], s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst.wstride = s->yuv_frame->linesize[0];

    dst.hstride = 768;



    for (i = 0; i < count; i++) {

        int x1 = clamp_int((int)(detect_state->boxes[i].x1 + 0.5f), 0, s->width - 2);

        int y1 = clamp_int((int)(detect_state->boxes[i].y1 + 0.5f), 0, s->height - 2);

        int x2 = clamp_int((int)(detect_state->boxes[i].x2 + 0.5f), 0, s->width);

        int y2 = clamp_int((int)(detect_state->boxes[i].y2 + 0.5f), 0, s->height);

        int width = x2 - x1;

        int height = y2 - y1;

        int border = 4;

        im_rect rects[4];

        IM_STATUS status;

        int edge;



        if (width < border || height < border) continue;



        x1 = align_even_down(x1);

        y1 = align_even_down(y1);

        x2 = align_even_up(x2);

        y2 = align_even_up(y2);

        x2 = clamp_int(x2, x1 + border, s->width);

        y2 = clamp_int(y2, y1 + border, s->height);

        width = x2 - x1;

        height = y2 - y1;

        if (width < border || height < border) continue;



        rects[0] = (im_rect){x1, y1, width, border};

        rects[1] = (im_rect){x1, y2 - border, width, border};

        rects[2] = (im_rect){x1, y1, border, height};

        rects[3] = (im_rect){x2 - border, y1, border, height};



        for (edge = 0; edge < 4; edge++) {

            status = imfill_t(dst, rects[edge], 0x00ff00, IM_SYNC);

            if (status != IM_STATUS_SUCCESS) return;

        }

    }

}







int streamer_push_zerocopy_overlay(FFmpegStreamer *s, int dma_fd, const DetectSharedState *detect_state) {

    int ret;

    AVPacket *pkt;

    rga_buffer_t src, dst;

    IM_STATUS status;
    int64_t push_start_ms;
    int64_t push_end_ms;
    static int64_t stat_start_ms = 0;
    static uint64_t stat_frames = 0;
    static int64_t stat_total_ms = 0;
    static int64_t stat_max_ms = 0;

    // 静态变量：用于平滑数值

    static float smoothed_scores[5] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f}; 



    if (dma_fd < 0 || !s->yuv_frame->data[0]) return -1;

    push_start_ms = get_mono_time_ms();
    if (stat_start_ms == 0) stat_start_ms = push_start_ms;

    // 1. 内存准备 (保持对齐)

    s->yuv_frame->height = 768;

    ret = av_frame_make_writable(s->yuv_frame);

    if (ret < 0) return -1;

    s->yuv_frame->height = 720;



    src = wrapbuffer_fd(dma_fd, s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst = wrapbuffer_virtualaddr(s->yuv_frame->data[0], s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst.wstride = s->yuv_frame->linesize[0];

    dst.hstride = 768;



    status = improcess(src, dst, (rga_buffer_t){0}, (im_rect){0, 0, s->width, s->height}, 

                       (im_rect){0, 0, s->width, s->height}, (im_rect){0}, IM_SYNC);

    

    draw_detect_boxes(s, detect_state);



    if (detect_state != NULL && detect_state->valid && detect_state->box_count > 0) {

        DetectSharedState local = *detect_state; 

        

        for (int i = 0; i < local.box_count; i++) {

            int id = local.boxes[i].class_id;

            float raw_score = local.boxes[i].score;

            if (id < 0 || id >= 5) continue;



            // --- 平滑逻辑修复：如果是第一帧，直接赋值 ---

            if (smoothed_scores[id] < 0.0f) smoothed_scores[id] = raw_score;

            else smoothed_scores[id] = (smoothed_scores[id] * 0.8f) + (raw_score * 0.2f);

            

            float display_score = smoothed_scores[id];



            int start_x = (int)local.boxes[i].x1 & ~1;

            int start_y = ((int)local.boxes[i].y1 - 40) & ~1;

            if (start_y < 20) start_y = ((int)local.boxes[i].y2 + 5) & ~1;

            int current_x = start_x;



            // A. 盖分类标签

            GlyphStamp *label = &g_stamp_labels[id];

            if (label->rgba_data) {

                int w = ALIGN_TO_2(label->width);

                int h = ALIGN_TO_2(label->height);

                improcess(label->rga_buf, dst, (rga_buffer_t){0}, (im_rect){0, 0, label->width, label->height}, 

                          (im_rect){current_x, start_y, w, h}, (im_rect){0}, IM_ALPHA_BLEND_SRC_OVER | IM_SYNC);

                current_x += (w + 4);

            }



            // B. 盖置信度 (使用平滑后的数值)

            int score_int = (int)(display_score * 100.0f);

            if (score_int > 99) score_int = 99;

            int tens = score_int / 10;

            int ones = score_int % 10;



            GlyphStamp *digits[3] = { &g_stamp_digits[tens], &g_stamp_digits[ones], &g_stamp_percent };

            

            for(int k = 0; k < 3; k++) {

                int w = ALIGN_TO_2(digits[k]->width);

                int h = ALIGN_TO_2(digits[k]->height);

                status = improcess(digits[k]->rga_buf, dst, (rga_buffer_t){0}, 

                          (im_rect){0, 0, digits[k]->width, digits[k]->height}, 

                          (im_rect){current_x, start_y, w, h}, 

                          (im_rect){0}, IM_ALPHA_BLEND_SRC_OVER | IM_SYNC);

                

                // 只有在这里加这个打印，如果没看到输出说明根本没进循环

                if (status != IM_STATUS_SUCCESS) {

                    printf("DEBUG: Failed to draw digit k=%d, w=%d, h=%d\n", k, w, h);

                }

                current_x += (w + 2);

            }

        }

    }


// 正确写法：使用真正的内存跨度 linesize[0] 去计算 UV 的起始地址
    s->yuv_frame->data[1] = s->yuv_frame->data[0] + (s->yuv_frame->linesize[0] * 768);
    s->yuv_frame->linesize[1] = s->width;

    int fps = s->enc_ctx->framerate.num > 0 ? s->enc_ctx->framerate.num : 30;
    int64_t now_ms = get_mono_time_ms();
    int64_t target_pts = (now_ms - g_stream_start_ms) * fps / 1000;
    if (target_pts <= s->frame_pts) {
        target_pts = s->frame_pts + 1;
    }
    s->frame_pts = target_pts;
    s->yuv_frame->pts = s->frame_pts;
    s->yuv_frame->pkt_duration = 1;

    ret = avcodec_send_frame(s->enc_ctx, s->yuv_frame);

    if (ret < 0) return ret;

    pkt = av_packet_alloc();

    if (!pkt) return -1;

    while (avcodec_receive_packet(s->enc_ctx, pkt) == 0) {

        av_packet_rescale_ts(pkt, s->enc_ctx->time_base, s->video_st->time_base);

        pkt->stream_index = s->video_st->index;

        ret = av_write_frame(s->fmt_ctx, pkt);
        if (ret < 0) {
            av_packet_unref(pkt);
            av_packet_free(&pkt);
            return ret;
        }

        av_packet_unref(pkt);

    }

    av_packet_free(&pkt);

    push_end_ms = get_mono_time_ms();
    stat_frames++;
    stat_total_ms += push_end_ms - push_start_ms;
    if (push_end_ms - push_start_ms > stat_max_ms) {
        stat_max_ms = push_end_ms - push_start_ms;
    }
    if (push_end_ms - stat_start_ms >= 5000) {
        double sec = (double)(push_end_ms - stat_start_ms) / 1000.0;
        double avg_ms = stat_frames > 0 ? (double)stat_total_ms / (double)stat_frames : 0.0;
        printf("[Child][Encode] fps=%.1f avg_ms=%.2f max_ms=%lld pts=%lld\n",
               stat_frames / sec, avg_ms, (long long)stat_max_ms, (long long)s->frame_pts);
        stat_start_ms = push_end_ms;
        stat_frames = 0;
        stat_total_ms = 0;
        stat_max_ms = 0;
    }

    return 0;

}

int streamer_push_zerocopy(FFmpegStreamer *s, int dma_fd) {

    return streamer_push_zerocopy_overlay(s, dma_fd, NULL);

}



int streamer_clean(FFmpegStreamer *s)

{

    if (!s) return -1;

    if (s->fmt_ctx) av_write_trailer(s->fmt_ctx);

    if (s->yuv_frame) av_frame_free(&s->yuv_frame);

    if (s->enc_ctx) avcodec_free_context(&s->enc_ctx);

    if (s->fmt_ctx && !(s->fmt_ctx->oformat->flags & AVFMT_NOFILE)) {

        avio_closep(&s->fmt_ctx->pb);

    }

    if (s->fmt_ctx) {

        avformat_free_context(s->fmt_ctx);

        s->fmt_ctx = NULL;

    }

    return 0;

}