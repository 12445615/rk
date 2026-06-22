#include "encoder.h"

#include <stdio.h>

#include <stdlib.h>

#include <string.h>

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
    s->side_fmt_ctx = NULL;
    s->side_video_st = NULL;

    

    g_stream_start_ms = get_mono_time_ms();



    avformat_alloc_output_context2(&s->fmt_ctx, NULL, "flv", filename);

    if (!s->fmt_ctx) {

        fprintf(stderr, "Could not allocate output context\n");

        return -1;

    }



    // ==== ���Ŀ��ӳ����� 1��ǿ��Ҫ�� FFmpeg �����κΰ���ѹ ====

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
    // ����ˢ���Ż���ʼ��
    // 1. �޸� GOP ��СΪ 60��Լ����һ��I֡��������Ƶ���Ĵ��ˢ�µ��µĿ���
    s->enc_ctx->gop_size = fps / 2;
    if (s->enc_ctx->gop_size < 1) {
        s->enc_ctx->gop_size = 1;
    }
    
    s->enc_ctx->max_b_frames = 0;
    s->enc_ctx->pix_fmt = AV_PIX_FMT_NV12;
    
   // 1. �������������� 3Mbps����֤�ճ�������
    s->enc_ctx->bit_rate = 3000000; 
    
    // 2. �����ġ�������ʷſ��� 6Mbps������������ͷ�����ƶ�ʱ����������������͸��̬���棡
    s->enc_ctx->rc_max_rate = 3000000; 
    s->enc_ctx->rc_buffer_size = 200000;

    // 3. �����ġ��ſ����ѹ���� (qmax)
    // qmin ���� 18����֤��ֹ���漫��������
    // qmax �������ߵ� 45���������̫�ͱ���32�������˶�ʱ���ʻᳬ�꣬Ӳ�������ֱ�ӱ������������ߺ�����Ǳ���ͣ���������˺�ѣ�
    s->enc_ctx->qmin = 18;
    s->enc_ctx->qmax = 38;
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

        // ==== ���Ŀ��ӳ����� 2����������ײ�д���� ====

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




static int streamer_write_side_packet(FFmpegStreamer *s, const AVPacket *pkt)
{
    AVPacket side_pkt;
    int ret;

    if (s == NULL || s->side_fmt_ctx == NULL || s->side_video_st == NULL || pkt == NULL) {
        return 0;
    }

    av_init_packet(&side_pkt);
    ret = av_packet_ref(&side_pkt, pkt);
    if (ret < 0) {
        return ret;
    }

    av_packet_rescale_ts(&side_pkt, s->video_st->time_base, s->side_video_st->time_base);
    side_pkt.stream_index = s->side_video_st->index;
    ret = av_write_frame(s->side_fmt_ctx, &side_pkt);
    av_packet_unref(&side_pkt);
    return ret;
}

int streamer_start_side_record(FFmpegStreamer *s, const char *filename)
{
    AVFormatContext *fmt = NULL;
    AVStream *st = NULL;
    int ret;

    if (s == NULL || filename == NULL || filename[0] == '\0') {
        return -1;
    }
    if (s->side_fmt_ctx != NULL) {
        return 0;
    }
    if (s->fmt_ctx == NULL || s->video_st == NULL || s->enc_ctx == NULL) {
        return -1;
    }

    ret = avformat_alloc_output_context2(&fmt, NULL, "mpegts", filename);
    if (ret < 0 || fmt == NULL) {
        fprintf(stderr, "[Encoder] side record alloc output failed: %s\n", filename);
        return -1;
    }

    fmt->flags |= AVFMT_FLAG_FLUSH_PACKETS;
    st = avformat_new_stream(fmt, NULL);
    if (st == NULL) {
        avformat_free_context(fmt);
        return -1;
    }
    ret = avcodec_parameters_copy(st->codecpar, s->video_st->codecpar);
    if (ret < 0) {
        avformat_free_context(fmt);
        return -1;
    }
    st->codecpar->codec_tag = 0;
    st->time_base = s->video_st->time_base;

    if (!(fmt->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open2(&fmt->pb, filename, AVIO_FLAG_WRITE, NULL, NULL);
        if (ret < 0) {
            fprintf(stderr, "[Encoder] side record open failed: %s ret=%d\n", filename, ret);
            avformat_free_context(fmt);
            return -1;
        }
    }

    ret = avformat_write_header(fmt, NULL);
    if (ret < 0) {
        fprintf(stderr, "[Encoder] side record header failed: %s ret=%d\n", filename, ret);
        if (!(fmt->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&fmt->pb);
        }
        avformat_free_context(fmt);
        return -1;
    }

    s->side_fmt_ctx = fmt;
    s->side_video_st = st;
    printf("[Encoder] side record started: %s\n", filename);
    return 0;
}

int streamer_stop_side_record(FFmpegStreamer *s)
{
    if (s == NULL || s->side_fmt_ctx == NULL) {
        return 0;
    }

    av_write_trailer(s->side_fmt_ctx);
    if (!(s->side_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&s->side_fmt_ctx->pb);
    }
    avformat_free_context(s->side_fmt_ctx);
    s->side_fmt_ctx = NULL;
    s->side_video_st = NULL;
    printf("[Encoder] side record stopped\n");
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

        

        // ==== ���Ŀ��ӳ����� 3�����ٽ�֯�ȴ���Ƶ��ֱ�ӱ����������� ====

        av_write_frame(s->fmt_ctx, pkt);
        streamer_write_side_packet(s, pkt);

        

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

static void nv12_fill_rect(FFmpegStreamer *s,
                           int x,
                           int y,
                           int width,
                           int height,
                           unsigned char y_value,
                           unsigned char uv_value) {
    int row;
    int uv_row;
    int uv_x;
    int uv_y;
    int uv_w;
    int uv_h;
    unsigned char *y_plane;
    unsigned char *uv_plane;

    if (s == NULL || s->yuv_frame == NULL || s->yuv_frame->data[0] == NULL ||
        width <= 0 || height <= 0) {
        return;
    }

    x = clamp_int(x, 0, s->width);
    y = clamp_int(y, 0, s->height);
    width = clamp_int(width, 0, s->width - x);
    height = clamp_int(height, 0, s->height - y);
    if (width <= 0 || height <= 0) return;

    y_plane = s->yuv_frame->data[0];
    for (row = 0; row < height; row++) {
        memset(y_plane + (y + row) * s->yuv_frame->linesize[0] + x, y_value, width);
    }

    uv_plane = s->yuv_frame->data[0] + s->yuv_frame->linesize[0] * 768;
    uv_x = x & ~1;
    uv_y = y & ~1;
    uv_w = align_even_up(width + (x - uv_x));
    uv_h = align_even_up(height + (y - uv_y)) / 2;
    uv_w = clamp_int(uv_w, 0, s->width - uv_x);
    if (uv_w <= 0 || uv_h <= 0) return;

    for (uv_row = 0; uv_row < uv_h; uv_row++) {
        memset(uv_plane + (uv_y / 2 + uv_row) * s->yuv_frame->linesize[1] + uv_x,
               uv_value,
               uv_w);
    }
}

static void nv12_fill_rect_yuv(FFmpegStreamer *s,
                               int x,
                               int y,
                               int width,
                               int height,
                               unsigned char y_value,
                               unsigned char u_value,
                               unsigned char v_value) {
    int row;
    int uv_row;
    int uv_col;
    int uv_x;
    int uv_y;
    int uv_w;
    int uv_h;
    unsigned char *y_plane;
    unsigned char *uv_plane;

    if (s == NULL || s->yuv_frame == NULL || s->yuv_frame->data[0] == NULL ||
        width <= 0 || height <= 0) {
        return;
    }

    x = clamp_int(x, 0, s->width);
    y = clamp_int(y, 0, s->height);
    width = clamp_int(width, 0, s->width - x);
    height = clamp_int(height, 0, s->height - y);
    if (width <= 0 || height <= 0) return;

    y_plane = s->yuv_frame->data[0];
    for (row = 0; row < height; row++) {
        memset(y_plane + (y + row) * s->yuv_frame->linesize[0] + x, y_value, width);
    }

    uv_plane = s->yuv_frame->data[0] + s->yuv_frame->linesize[0] * 768;
    uv_x = x & ~1;
    uv_y = y & ~1;
    uv_w = align_even_up(width + (x - uv_x));
    uv_h = align_even_up(height + (y - uv_y)) / 2;
    uv_w = clamp_int(uv_w, 0, s->width - uv_x);
    if (uv_w <= 0 || uv_h <= 0) return;

    for (uv_row = 0; uv_row < uv_h; uv_row++) {
        unsigned char *row_ptr = uv_plane + (uv_y / 2 + uv_row) * s->yuv_frame->linesize[1] + uv_x;
        for (uv_col = 0; uv_col < uv_w; uv_col += 2) {
            row_ptr[uv_col] = u_value;
            if (uv_col + 1 < uv_w) {
                row_ptr[uv_col + 1] = v_value;
            }
        }
    }
}

static void nv12_draw_stamp_luma(FFmpegStreamer *s,
                                 const GlyphStamp *stamp,
                                 int x,
                                 int y,
                                 unsigned char y_value) {
    int stride;
    int row;
    int col;
    unsigned char *y_plane;

    if (s == NULL || stamp == NULL || stamp->rgba_data == NULL ||
        s->yuv_frame == NULL || s->yuv_frame->data[0] == NULL) {
        return;
    }

    stride = (stamp->width + 3) & ~3;
    y_plane = s->yuv_frame->data[0];
    for (row = 0; row < stamp->height; row++) {
        int dst_y = y + row;
        if (dst_y < 0 || dst_y >= s->height) continue;
        for (col = 0; col < stamp->width; col++) {
            int dst_x = x + col;
            int src_index;
            if (dst_x < 0 || dst_x >= s->width) continue;
            src_index = (row * stride + col) * 4;
            if (stamp->rgba_data[src_index + 3] != 0) {
                y_plane[dst_y * s->yuv_frame->linesize[0] + dst_x] = y_value;
            }
        }
    }
}

static void draw_overlay_zone_rect(FFmpegStreamer *s,
                                   rga_buffer_t dst,
                                   const DetectZoneRect *zone,
                                   int border,
                                   unsigned int color) {
    int x1;
    int y1;
    int x2;
    int y2;
    int width;
    int height;
    im_rect rects[4];
    int edge;

    if (s == NULL || zone == NULL || !zone->valid || border <= 0) {
        return;
    }

    x1 = clamp_int((int)(zone->x1 + 0.5f), 0, s->width - 2);
    y1 = clamp_int((int)(zone->y1 + 0.5f), 0, s->height - 2);
    x2 = clamp_int((int)(zone->x2 + 0.5f), x1 + border, s->width);
    y2 = clamp_int((int)(zone->y2 + 0.5f), y1 + border, s->height);

    x1 = align_even_down(x1);
    y1 = align_even_down(y1);
    x2 = align_even_up(x2);
    y2 = align_even_up(y2);
    x2 = clamp_int(x2, x1 + border, s->width);
    y2 = clamp_int(y2, y1 + border, s->height);

    width = x2 - x1;
    height = y2 - y1;
    if (width < border || height < border) {
        return;
    }

    rects[0] = (im_rect){x1, y1, width, border};
    rects[1] = (im_rect){x1, y2 - border, width, border};
    rects[2] = (im_rect){x1, y1, border, height};
    rects[3] = (im_rect){x2 - border, y1, border, height};

    for (edge = 0; edge < 4; edge++) {
        if (imfill_t(dst, rects[edge], color, IM_SYNC) != IM_STATUS_SUCCESS) {
            return;
        }
    }
}

static void draw_nv12_line(FFmpegStreamer *s,
                           int x0,
                           int y0,
                           int x1,
                           int y1,
                           int thickness,
                           uint8_t y_value,
                           uint8_t u_value,
                           uint8_t v_value) {
    int dx;
    int dy;
    int sx;
    int sy;
    int err;

    if (s == NULL || thickness <= 0) {
        return;
    }

    x0 = clamp_int(x0, 0, s->width - 1);
    y0 = clamp_int(y0, 0, s->height - 1);
    x1 = clamp_int(x1, 0, s->width - 1);
    y1 = clamp_int(y1, 0, s->height - 1);

    dx = abs(x1 - x0);
    dy = -abs(y1 - y0);
    sx = x0 < x1 ? 1 : -1;
    sy = y0 < y1 ? 1 : -1;
    err = dx + dy;

    while (1) {
        nv12_fill_rect_yuv(s,
                           x0 - thickness / 2,
                           y0 - thickness / 2,
                           thickness,
                           thickness,
                           y_value,
                           u_value,
                           v_value);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        {
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

static void draw_overlay_zone_quad_red(FFmpegStreamer *s,
                                       const DetectZoneRect *zone,
                                       int thickness) {
    if (s == NULL || zone == NULL || !zone->valid) {
        return;
    }

    draw_nv12_line(s, (int)(zone->p0x + 0.5f), (int)(zone->p0y + 0.5f),
                      (int)(zone->p1x + 0.5f), (int)(zone->p1y + 0.5f),
                      thickness, 76, 84, 255);
    draw_nv12_line(s, (int)(zone->p1x + 0.5f), (int)(zone->p1y + 0.5f),
                      (int)(zone->p2x + 0.5f), (int)(zone->p2y + 0.5f),
                      thickness, 76, 84, 255);
    draw_nv12_line(s, (int)(zone->p2x + 0.5f), (int)(zone->p2y + 0.5f),
                      (int)(zone->p3x + 0.5f), (int)(zone->p3y + 0.5f),
                      thickness, 76, 84, 255);
    draw_nv12_line(s, (int)(zone->p3x + 0.5f), (int)(zone->p3y + 0.5f),
                      (int)(zone->p0x + 0.5f), (int)(zone->p0y + 0.5f),
                      thickness, 76, 84, 255);
}

static void draw_overlay_zone_quad_black(FFmpegStreamer *s,
                                         const DetectZoneRect *zone,
                                         int thickness) {
    if (s == NULL || zone == NULL || !zone->valid) {
        return;
    }

    draw_nv12_line(s, (int)(zone->p0x + 0.5f), (int)(zone->p0y + 0.5f),
                      (int)(zone->p1x + 0.5f), (int)(zone->p1y + 0.5f),
                      thickness, 16, 128, 128);
    draw_nv12_line(s, (int)(zone->p1x + 0.5f), (int)(zone->p1y + 0.5f),
                      (int)(zone->p2x + 0.5f), (int)(zone->p2y + 0.5f),
                      thickness, 16, 128, 128);
    draw_nv12_line(s, (int)(zone->p2x + 0.5f), (int)(zone->p2y + 0.5f),
                      (int)(zone->p3x + 0.5f), (int)(zone->p3y + 0.5f),
                      thickness, 16, 128, 128);
    draw_nv12_line(s, (int)(zone->p3x + 0.5f), (int)(zone->p3y + 0.5f),
                      (int)(zone->p0x + 0.5f), (int)(zone->p0y + 0.5f),
                      thickness, 16, 128, 128);
}

static void draw_detect_zones(FFmpegStreamer *s,
                              rga_buffer_t dst,
                              const ZoneOverlayState *zone_state) {
    if (zone_state == NULL || !zone_state->zone_valid) {
        return;
    }

    (void)dst;
    draw_overlay_zone_quad_red(s, &zone_state->work_zone, 4);
    draw_overlay_zone_quad_red(s, &zone_state->danger_zone, 4);
}



static void draw_detect_boxes(FFmpegStreamer *s, const DetectSharedState *detect_state, const ZoneOverlayState *zone_state) {

    rga_buffer_t dst;

    int count;

    int i;



    if (s == NULL) return;



    dst = wrapbuffer_virtualaddr(s->yuv_frame->data[0], s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst.wstride = s->yuv_frame->linesize[0];

    dst.hstride = 768;

    draw_detect_zones(s, dst, zone_state);

    if (detect_state == NULL || !detect_state->valid || detect_state->box_count <= 0) return;

    {
        static int64_t last_box_log_ms = 0;
        int64_t now_log_ms = get_mono_time_ms();
        if (now_log_ms - last_box_log_ms >= 1000) {
            int log_count = detect_state->box_count;
            if (log_count > 3) log_count = 3;
            printf("[Child][AIOverlay] boxes=%d frame=%lld ts=%lld\n",
                   detect_state->box_count,
                   (long long)detect_state->frame_seq,
                   (long long)detect_state->timestamp_ms);
            for (int bi = 0; bi < log_count; bi++) {
                printf("[Child][AIOverlay] #%d cls=%d score=%.3f xy=(%.1f,%.1f)-(%.1f,%.1f)\n",
                       bi,
                       detect_state->boxes[bi].class_id,
                       detect_state->boxes[bi].score,
                       detect_state->boxes[bi].x1,
                       detect_state->boxes[bi].y1,
                       detect_state->boxes[bi].x2,
                       detect_state->boxes[bi].y2);
            }
            last_box_log_ms = now_log_ms;
        }
    }

    count = detect_state->box_count;

    if (count > DETECT_MAX_BOXES) count = DETECT_MAX_BOXES;



    for (i = 0; i < count; i++) {

        int x1 = clamp_int((int)(detect_state->boxes[i].x1 + 0.5f), 0, s->width - 2);

        int y1 = clamp_int((int)(detect_state->boxes[i].y1 + 0.5f), 0, s->height - 2);

        int x2 = clamp_int((int)(detect_state->boxes[i].x2 + 0.5f), 0, s->width);

        int y2 = clamp_int((int)(detect_state->boxes[i].y2 + 0.5f), 0, s->height);

        int width = x2 - x1;

        int height = y2 - y1;

        int border = 4;
        int class_id = detect_state->boxes[i].class_id;
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



        (void)rects;
        (void)status;
        (void)edge;
        /* Draw AI boxes directly into NV12 so zone/RGA overlay cannot hide them. */
        draw_nv12_line(s, x1, y1, x2, y1, border, 144, 54, 34);
        draw_nv12_line(s, x2, y1, x2, y2, border, 144, 54, 34);
        draw_nv12_line(s, x2, y2, x1, y2, border, 144, 54, 34);
        draw_nv12_line(s, x1, y2, x1, y1, border, 144, 54, 34);

    }

}







int streamer_push_zerocopy_overlay(FFmpegStreamer *s, int dma_fd, const DetectSharedState *detect_state, const ZoneOverlayState *zone_state) {

    int ret;

    AVPacket *pkt;

    rga_buffer_t src, dst;

    int64_t push_start_ms;
    int64_t push_end_ms;
    static int64_t stat_start_ms = 0;
    static uint64_t stat_frames = 0;
    static int64_t stat_total_ms = 0;
    static int64_t stat_max_ms = 0;

    // ��̬����������ƽ����ֵ

    static float smoothed_scores[6] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};



    if (dma_fd < 0 || !s->yuv_frame->data[0]) return -1;

    push_start_ms = get_mono_time_ms();
    if (stat_start_ms == 0) stat_start_ms = push_start_ms;

    // 1. �ڴ�׼�� (���ֶ���)

    s->yuv_frame->height = 768;

    ret = av_frame_make_writable(s->yuv_frame);

    if (ret < 0) return -1;

    s->yuv_frame->height = 720;



    src = wrapbuffer_fd(dma_fd, s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst = wrapbuffer_virtualaddr(s->yuv_frame->data[0], s->width, s->height, RK_FORMAT_YCbCr_420_SP);

    dst.wstride = s->yuv_frame->linesize[0];

    dst.hstride = 768;



    if (improcess(src, dst, (rga_buffer_t){0}, (im_rect){0, 0, s->width, s->height},
                  (im_rect){0, 0, s->width, s->height}, (im_rect){0}, IM_SYNC) != IM_STATUS_SUCCESS) {
        return -1;
    }

    

    draw_detect_boxes(s, detect_state, zone_state);



    if (detect_state != NULL && detect_state->valid && detect_state->box_count > 0) {

        DetectSharedState local = *detect_state; 

        

        for (int i = 0; i < local.box_count; i++) {

            int id = local.boxes[i].class_id;

            float raw_score = local.boxes[i].score;

            if (id < 0 || id >= 6) continue;



            // --- ƽ���߼��޸�������ǵ�һ֡��ֱ�Ӹ�ֵ ---

            if (smoothed_scores[id] < 0.0f) smoothed_scores[id] = raw_score;

            else smoothed_scores[id] = (smoothed_scores[id] * 0.8f) + (raw_score * 0.2f);

            

            float display_score = smoothed_scores[id];



            int start_x = (int)local.boxes[i].x1 & ~1;

            int start_y = ((int)local.boxes[i].y1 - 40) & ~1;

            if (start_y < 20) start_y = ((int)local.boxes[i].y2 + 5) & ~1;

            int current_x = start_x + 8;
            int score_int = (int)(display_score * 100.0f);
            if (score_int > 99) score_int = 99;
            if (score_int < 0) score_int = 0;
            GlyphStamp *label = &g_stamp_labels[id];
            GlyphStamp *score = &g_stamp_score_text[id][score_int];
            int label_w = label->rgba_data ? ALIGN_TO_2(label->width) : 0;
            int label_h = label->rgba_data ? ALIGN_TO_2(label->height) : 0;
            int score_w = score->rgba_data ? ALIGN_TO_2(score->width) : 0;
            int score_h = score->rgba_data ? ALIGN_TO_2(score->height) : 0;
            int text_h = label_h;
            if (score_h > text_h) text_h = score_h;
            int text_w = label_w + (label_w > 0 && score_w > 0 ? 5 : 0) + score_w;
            int bg_x = clamp_int(start_x - 3, 0, s->width - 2);
            int bg_y = clamp_int(start_y - 3, 0, s->height - 2);
            int bg_w = clamp_int(text_w + 8, 2, s->width - bg_x);
            int bg_h = clamp_int(text_h + 6, 2, s->height - bg_y);

            bg_x = align_even_down(bg_x);
            bg_y = align_even_down(bg_y);
            bg_w = align_even_up(bg_w);
            bg_h = align_even_up(bg_h);
            if (bg_x + bg_w > s->width) bg_w = align_even_down(s->width - bg_x);
            if (bg_y + bg_h > s->height) bg_h = align_even_down(s->height - bg_y);
            if (bg_w > 0 && bg_h > 0) {
                nv12_fill_rect(s, bg_x, bg_y, bg_w, bg_h, 16, 128);
                if (bg_w >= 8) {
                    imfill_t(dst, (im_rect){bg_x, bg_y, 6, bg_h}, osd_class_color_rgb(id), IM_SYNC);
                }
            }



            // A. �Ƿ����ǩ

            if (label->rgba_data) {

                nv12_draw_stamp_luma(s, label, current_x, start_y, 235);

                current_x += (label_w + 4);

            }



            // B. �����Ŷ� (ʹ��ƽ�������ֵ)

            if (score->rgba_data) {

                nv12_draw_stamp_luma(s, score, current_x, start_y, 235);
            }

        }

    }


// ��ȷд����ʹ���������ڴ��� linesize[0] ȥ���� UV ����ʼ��ַ
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
        ret = streamer_write_side_packet(s, pkt);
        if (ret < 0) {
            fprintf(stderr, "[Encoder] side record packet write failed: %d\n", ret);
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

    return streamer_push_zerocopy_overlay(s, dma_fd, NULL, NULL);

}



int streamer_clean(FFmpegStreamer *s)

{

    if (!s) return -1;

    streamer_stop_side_record(s);

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
