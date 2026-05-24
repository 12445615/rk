#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>

// Initialize camera and mmap buffers
int camera_init(CameraCtx *ctx, const char *dev_name, int width, int height) {
    ctx->fd = open(dev_name, O_RDWR);
    if (ctx->fd < 0) {
        perror("camera open");
        return -1;
    }

    ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = ctx->buf_type;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.num_planes = 1;

    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("camera SetFormat");
        return -1;
    }

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = ctx->buf_type;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 30;
    if (ioctl(ctx->fd, VIDIOC_S_PARM, &parm) < 0) {
        perror("camera set 30fps warning");
    }
    memset(&parm, 0, sizeof(parm));
    parm.type = ctx->buf_type;
    if (ioctl(ctx->fd, VIDIOC_G_PARM, &parm) == 0 &&
        parm.parm.capture.timeperframe.numerator > 0 &&
        parm.parm.capture.timeperframe.denominator > 0) {
        double fps = (double)parm.parm.capture.timeperframe.denominator /
                     (double)parm.parm.capture.timeperframe.numerator;
        printf("[Camera] driver frame interval=%u/%u fps=%.2f\n",
               parm.parm.capture.timeperframe.numerator,
               parm.parm.capture.timeperframe.denominator,
               fps);
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUF_COUNT;
    req.type = ctx->buf_type;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("camera request buffer");
        return -1;
    }

    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];
    for (int i = 0; i < BUF_COUNT; ++i) {
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = ctx->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(ctx->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("camera query buffer");
            return -1;
        }

        ctx->buffers[i].length = buf.m.planes[0].length;
        ctx->buffers[i].start = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, 
                                     MAP_SHARED, ctx->fd, buf.m.planes[0].m.mem_offset);
        
        if (ctx->buffers[i].start == MAP_FAILED) {
            perror("camera mmap");
            return -1;
        }

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("camera queue buf");
            return -1;
        }
    }
    return 0;
}
// Start capture stream
int camera_start(CameraCtx *ctx) {
    return ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->buf_type);
}
// Stop capture stream
int camera_stop(CameraCtx *ctx) {
    return ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->buf_type);
}
// Release resources
void camera_deinit(CameraCtx *ctx) {
    for (int i = 0; i < BUF_COUNT; i++) {
        if (ctx->buffers[i].start && ctx->buffers[i].start != MAP_FAILED) {
            munmap(ctx->buffers[i].start, ctx->buffers[i].length);
        }
    }
    if (ctx->fd >= 0) close(ctx->fd);
}
