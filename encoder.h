
#ifndef __ENCODER_H
#define __ENCODER_H

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include "detect_shared.h"

typedef struct {
    AVFormatContext *fmt_ctx;
    AVCodecContext  *enc_ctx;
    AVStream        *video_st;
    AVFormatContext *side_fmt_ctx;
    AVStream        *side_video_st;
    struct SwsContext *sws_ctx;
    AVFrame         *yuv_frame;
    int width;
    int height;
    int64_t frame_pts;
} FFmpegStreamer;

int streamer_init(FFmpegStreamer *s, const char *filename, int width, int height, int fps);
int streamer_push(FFmpegStreamer *s,uint8_t *nv12_data);
int streamer_push_zerocopy(FFmpegStreamer *s, int dma_fd);
int streamer_push_zerocopy_overlay(FFmpegStreamer *s, int dma_fd, const DetectSharedState *detect_state, const ZoneOverlayState *zone_state);
int streamer_start_side_record(FFmpegStreamer *s, const char *filename);
int streamer_stop_side_record(FFmpegStreamer *s);
int streamer_clean(FFmpegStreamer *s);


#endif
