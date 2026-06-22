#include "camera.h"
#include "ipc.h"
#include "encoder.h"
#include "sensor_modbus.h"
#include "aliyun_mqtt.h"
#include "video_store.h"
#include "video_uploader.h"
#include "rknn_worker.h"
#include "safety_interlock_client.h"
#include "zone_detector.h"
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
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <signal.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <RgaApi.h>
#include <im2d.h>
#include "osd_cache.h"//加入�?
#include "audio_alert.h"//加入语音

#define VIDEO_DEV "/dev/video11"
#define RTMP_URL "rtmp://10.139.8.231:1935/fire_live/test"
#define WIDTH  1280
#define HEIGHT 720
#define BUF_COUNT 4
#define FRAME_SIZE (WIDTH * HEIGHT * 3 / 2)
#define SENSOR_DEVICE_ENV "SENSOR_MODBUS_DEV"
#define SENSOR_DEVICE_DISABLE_ENV "SENSOR_MODBUS_DISABLE"
#define SENSOR_DEVICE_DEFAULT "/dev/ttyUSB0"
#define CAMERA_SENSOR_SUBDEV_ENV "CAMERA_SENSOR_SUBDEV"
#define CAMERA_SENSOR_SUBDEV_DEFAULT "/dev/v4l-subdev2"
#define CAMERA_SENSOR_LOCK_ENV "CAMERA_SENSOR_LOCK_FPS"
#define CAMERA_SENSOR_VBLANK_ENV "CAMERA_SENSOR_VBLANK"
#define CAMERA_SENSOR_EXPOSURE_ENV "CAMERA_SENSOR_EXPOSURE"
#define CAMERA_SENSOR_GAIN_ENV "CAMERA_SENSOR_GAIN"
#define CAMERA_SENSOR_LOCK_INTERVAL_MS 5000
#define STREAM_RESTART_BASE_MS 1000
#define STREAM_RESTART_MAX_MS 30000
#define STREAM_FPS_ENV "CAMERA_FLOW_STREAM_FPS"
#define STREAM_FPS_DEFAULT 30
#define STREAM_DUP_FRAMES_ENV "CAMERA_FLOW_DUP_FRAMES"
#define STREAM_DUP_FRAMES_DEFAULT 1
#define CHILD_RTMP_RETRY_BASE_MS 1000
#define CHILD_RTMP_RETRY_MAX_MS 30000
#define CHILD_TARGET_RECORD_GRACE_MS 3000
#define CHILD_TARGET_RECORD_SEGMENT_MIN_MS 20000
#define CHILD_TARGET_RECORD_SEGMENT_MAX_MS 60000
#define AI_ENABLE_ENV "CAMERA_FLOW_AI_ENABLE"
#define AI_MODEL_PATH_ENV "CAMERA_FLOW_AI_MODEL_PATH"
#define AI_INTERVAL_ENV "CAMERA_FLOW_AI_INTERVAL"
#define AI_CONF_ENV "CAMERA_FLOW_AI_CONF"
#define AI_NMS_ENV "CAMERA_FLOW_AI_NMS"
#define AI_STATS_INTERVAL_MS 5000
#define VIDEO_STATS_INTERVAL_MS 5000
#define SAFETY_AI_STALE_MS 1000
#define SAFETY_AI_SEND_INTERVAL_MS 500
#define SAFETY_FUSION_SEND_INTERVAL_MS 500
#define SAFETY_CONFIRM_DELAY_MS 5000
#define SAFETY_SMOKE_HIGH_ENV "SAFETY_SMOKE_HIGH"
#define SAFETY_GAS_HIGH_ENV "SAFETY_GAS_HIGH"
#define SAFETY_TEMP_HIGH_X10_ENV "SAFETY_TEMP_HIGH_X10"
#define SAFETY_SMOKE_RISE_ENV "SAFETY_SMOKE_RISE_DELTA"
#define SAFETY_TEMP_RISE_X10_ENV "SAFETY_TEMP_RISE_X10_DELTA"
#define SAFETY_SMOKE_HIGH_DEFAULT 70
#define SAFETY_GAS_HIGH_DEFAULT 3000
#define SAFETY_TEMP_HIGH_X10_DEFAULT 600
#define SAFETY_SMOKE_RISE_DEFAULT 20
#define SAFETY_TEMP_RISE_X10_DEFAULT 30
#define SAFETY_AI_FLAG_VALID       (1u << 0)
#define SAFETY_AI_FLAG_PPE_OK      (1u << 1)
#define SAFETY_AI_FLAG_NO_HELMET   (1u << 2)
#define SAFETY_AI_FLAG_NO_VEST     (1u << 3)
#define SAFETY_AI_FLAG_FIRE        (1u << 4)
#define SAFETY_AI_FLAG_FIRE_OUT    (1u << 5)
#define SAFETY_RESET_FIRE_IGNORE_MS 5000
#define SAFETY_AI_FLAG_INTRUSION   (1u << 6)
#define SAFETY_AI_FLAG_ZONE1_BUSY  (1u << 7)
#define SAFETY_AI_FLAG_PERSON      (1u << 8)
#define SAFETY_AI_FLAG_ZONE2_BUSY  (1u << 9)
#define SAFETY_AI_FLAG_ZONE1_PPE_BAD (1u << 12)
#define SAFETY_AI_FLAG_ZONE2_PPE_BAD (1u << 13)
#define SAFETY_AI_FLAG_ZONE1_READY (1u << 10)
#define SAFETY_AI_FLAG_ZONE2_READY (1u << 11)

#define AI_DETECT_STATE_NONE 0
#define AI_DETECT_STATE_PPE_OK 1
#define AI_DETECT_STATE_NO_HELMET 2
#define AI_DETECT_STATE_NO_VEST 3
#define AI_DETECT_STATE_PPE_BOTH_BAD 4
#define AI_DETECT_STATE_FIRE_WORK_ZONE 5
#define AI_DETECT_STATE_FIRE_OUT_ZONE 6
#define AI_DETECT_STATE_INTRUSION 7

volatile sig_atomic_t is_running = 1;
static volatile sig_atomic_t g_video_uploader_signal_ready = 0;
static VideoUploader *g_video_uploader_for_signal = NULL;

typedef struct {
    int stream_online;
    pid_t child_pid;
    int parent_sock;
    DetectSharedState *detect_state;
    ZoneOverlayState *zone_state;
    int base_restart_ms;
    int max_restart_ms;
    int current_restart_ms;
    int restart_fail_count;
    int64_t last_restart_mono_ms;
} StreamState;

typedef enum {
    CHILD_OUTPUT_RTMP = 0,
    CHILD_OUTPUT_FILE = 1
} ChildOutputMode;

typedef struct {
    FFmpegStreamer streamer;
    int streamer_ready;
    ChildOutputMode mode;

    LocalStore store;
    int store_ready;
    VideoStore video_store;
    int video_store_ready;

    int64_t current_segment_id;
    int64_t current_segment_start_wall_ms;
    int64_t current_segment_start_mono_ms;
    char current_segment_path[PATH_MAX];

    FFmpegStreamer record_streamer;
    int record_streamer_ready;
    int64_t record_segment_id;
    int64_t record_segment_start_wall_ms;
    int64_t record_segment_start_mono_ms;
    int64_t record_target_last_seen_mono_ms;
    int record_segment_has_target;
    char record_segment_path[PATH_MAX];

    int rtmp_retry_backoff_ms;
    int rtmp_retry_max_ms;
    int64_t last_rtmp_retry_mono_ms;

    int debug_rtmp_fail_after_frames;
    int debug_rtmp_frame_count;
    int debug_rtmp_fail_triggered;

    DetectSharedState *detect_state;
    ZoneOverlayState *zone_state;
    DetectSharedState detect_snapshot;
    ZoneOverlayState zone_snapshot;
    DetectSharedState locked_zone_snapshot;
    DetectSharedState last_ai_snapshot;
} ChildOutputCtx;

static void overlay_prepare_static_zone_and_ai(ChildOutputCtx *ctx,
                                               DetectSharedState *overlay,
                                               int64_t frame_wall_ms) {
    if (ctx == NULL || overlay == NULL) {
        return;
    }

    if (overlay->zone_valid) {
        ctx->locked_zone_snapshot.zone_valid = overlay->zone_valid;
        ctx->locked_zone_snapshot.work_zone = overlay->work_zone;
        ctx->locked_zone_snapshot.danger_zone = overlay->danger_zone;
    } else if (ctx->locked_zone_snapshot.zone_valid) {
        overlay->zone_valid = ctx->locked_zone_snapshot.zone_valid;
        overlay->work_zone = ctx->locked_zone_snapshot.work_zone;
        overlay->danger_zone = ctx->locked_zone_snapshot.danger_zone;
    }

    if (overlay->valid && overlay->box_count > 0) {
        ctx->last_ai_snapshot.valid = overlay->valid;
        ctx->last_ai_snapshot.box_count = overlay->box_count;
        ctx->last_ai_snapshot.frame_seq = overlay->frame_seq;
        ctx->last_ai_snapshot.timestamp_ms = overlay->timestamp_ms;
        memcpy(ctx->last_ai_snapshot.boxes,
               overlay->boxes,
               sizeof(ctx->last_ai_snapshot.boxes));
        return;
    }

    if (ctx->last_ai_snapshot.valid &&
        ctx->last_ai_snapshot.box_count > 0 &&
        ctx->last_ai_snapshot.timestamp_ms > 0 &&
        frame_wall_ms - ctx->last_ai_snapshot.timestamp_ms <= SAFETY_AI_STALE_MS) {
        overlay->valid = ctx->last_ai_snapshot.valid;
        overlay->box_count = ctx->last_ai_snapshot.box_count;
        overlay->frame_seq = ctx->last_ai_snapshot.frame_seq;
        overlay->timestamp_ms = ctx->last_ai_snapshot.timestamp_ms;
        memcpy(overlay->boxes,
               ctx->last_ai_snapshot.boxes,
               sizeof(ctx->last_ai_snapshot.boxes));
    }
}
typedef struct {
    RknnWorker worker;
    int started;
    int enabled;
    int frame_interval;
    uint64_t frame_seq;
    size_t input_size;
    RknnWorkerInputInfo input_info;
    int64_t last_stats_log_ms;
    uint64_t preproc_fail_count;
    uint64_t skipped_busy_count;
    uint64_t skipped_interval_count;
    DetectSharedState *detect_state;
} AiPipeline;
static void sig_handler(int sig) {
    (void)sig;
    is_running = 0;
    if (g_video_uploader_signal_ready) {
        video_uploader_request_stop(g_video_uploader_for_signal);
    }
}

static void install_signal_handlers(void) {
    struct sigaction action;
    struct sigaction ignore_action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = sig_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    memset(&ignore_action, 0, sizeof(ignore_action));
    ignore_action.sa_handler = SIG_IGN;
    sigemptyset(&ignore_action.sa_mask);
    sigaction(SIGPIPE, &ignore_action, NULL);
}

static int64_t mono_now_ms(void) {
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
        return 0;
    }

    return (int64_t)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static int64_t wall_now_ms(void) {
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) < 0) {
        return 0;
    }

    return (int64_t)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static void reset_restart_backoff(StreamState *stream) {
    stream->restart_fail_count = 0;
    stream->current_restart_ms = stream->base_restart_ms;
}

static void increase_restart_backoff(StreamState *stream) {
    int next_delay = stream->current_restart_ms * 2;

    stream->restart_fail_count++;
    if (next_delay > stream->max_restart_ms) {
        next_delay = stream->max_restart_ms;
    }
    stream->current_restart_ms = next_delay;
}

static void child_reset_rtmp_backoff(ChildOutputCtx *ctx) {
    ctx->rtmp_retry_backoff_ms = CHILD_RTMP_RETRY_BASE_MS;
}

static void child_increase_rtmp_backoff(ChildOutputCtx *ctx) {
    int next_delay = ctx->rtmp_retry_backoff_ms * 2;

    if (next_delay > ctx->rtmp_retry_max_ms) {
        next_delay = ctx->rtmp_retry_max_ms;
    }
    ctx->rtmp_retry_backoff_ms = next_delay;
}

static int env_to_positive_int(const char *name, int fallback) {
    const char *value = getenv(name);
    char *endptr = NULL;
    long parsed;

    if (value == NULL || value[0] == '\0') {
        return fallback;
    }

    errno = 0;
    parsed = strtol(value, &endptr, 10);
    if (errno != 0 || endptr == value || *endptr != '\0' || parsed <= 0) {
        return fallback;
    }

    return (int)parsed;
}

static int env_to_bool_default(const char *name, int fallback) {
    const char *value = getenv(name);

    if (value == NULL || value[0] == '\0') {
        return fallback;
    }
    if (strcmp(value, "0") == 0 ||
        strcmp(value, "false") == 0 ||
        strcmp(value, "FALSE") == 0 ||
        strcmp(value, "off") == 0 ||
        strcmp(value, "OFF") == 0 ||
        strcmp(value, "no") == 0 ||
        strcmp(value, "NO") == 0) {
        return 0;
    }

    return 1;
}

static float env_to_float_range(const char *name, float fallback, float min_value, float max_value) {
    const char *value = getenv(name);
    char *endptr = NULL;
    float parsed;

    if (value == NULL || value[0] == '\0') {
        return fallback;
    }

    errno = 0;
    parsed = strtof(value, &endptr);
    if (errno != 0 || endptr == value || *endptr != '\0' ||
        parsed < min_value || parsed > max_value) {
        return fallback;
    }

    return parsed;
}


static int set_v4l2_ctrl_value(int fd, unsigned int id, int value, const char *name) {
    struct v4l2_control ctrl;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    ctrl.value = value;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        fprintf(stderr, "[Camera][Sensor] set %s=%d failed: %s\n", name, value, strerror(errno));
        return -1;
    }
    return 0;
}

static void camera_sensor_lock_fps_controls(void) {
    const char *subdev;
    int fd;
    int vblank;
    int exposure;
    int gain;

    if (!env_to_bool_default(CAMERA_SENSOR_LOCK_ENV, 0)) {
        return;
    }

    subdev = getenv(CAMERA_SENSOR_SUBDEV_ENV);
    if (subdev == NULL || subdev[0] == '\0') {
        subdev = CAMERA_SENSOR_SUBDEV_DEFAULT;
    }

    vblank = env_to_positive_int(CAMERA_SENSOR_VBLANK_ENV, 78);
    exposure = env_to_positive_int(CAMERA_SENSOR_EXPOSURE_ENV, 2048);
    gain = env_to_positive_int(CAMERA_SENSOR_GAIN_ENV, 256);

    fd = open(subdev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[Camera][Sensor] open %s failed: %s\n", subdev, strerror(errno));
        return;
    }

    set_v4l2_ctrl_value(fd, V4L2_CID_VBLANK, vblank, "vertical_blanking");
    set_v4l2_ctrl_value(fd, V4L2_CID_EXPOSURE, exposure, "exposure");
    set_v4l2_ctrl_value(fd, V4L2_CID_ANALOGUE_GAIN, gain, "analogue_gain");
    close(fd);

    printf("[Camera][Sensor] locked %s vblank=%d exposure=%d gain=%d\n",
           subdev, vblank, exposure, gain);
}

static int ai_preprocess_dma_to_rgb_fd(int dma_fd,
                                       const RknnWorkerInputBuffer *input_buffer) {
    rga_buffer_t src;
    rga_buffer_t dst;
    im_rect src_rect;
    im_rect dst_rect;
    IM_STATUS status;
    uint32_t dst_width;
    uint32_t dst_height;
    uint32_t resized_width;
    uint32_t resized_height;
    uint32_t left;
    uint32_t top;

    if (dma_fd < 0 || input_buffer == NULL || input_buffer->fd < 0 ||
        input_buffer->width == 0 || input_buffer->height == 0 ||
        input_buffer->channels != 3) {
        return -1;
    }

    dst_width = input_buffer->width;
    dst_height = input_buffer->height;
    if ((uint64_t)dst_width * HEIGHT <= (uint64_t)dst_height * WIDTH) {
        resized_width = dst_width;
        resized_height = (uint32_t)(((uint64_t)HEIGHT * dst_width) / WIDTH);
    } else {
        resized_height = dst_height;
        resized_width = (uint32_t)(((uint64_t)WIDTH * dst_height) / HEIGHT);
    }
    if (resized_width == 0 || resized_height == 0) {
        return -1;
    }

    src = wrapbuffer_fd(dma_fd, WIDTH, HEIGHT, RK_FORMAT_YCbCr_420_SP);
    src.wstride = WIDTH;
    src.hstride = HEIGHT;

    dst = wrapbuffer_fd(input_buffer->fd,
                        (int)dst_width,
                        (int)dst_height,
                        RK_FORMAT_RGB_888);
    dst.wstride = (int)dst_width;
    dst.hstride = (int)dst_height;

    if (input_buffer->virt_addr != NULL && input_buffer->size > 0) {
        memset(input_buffer->virt_addr, 0x72, input_buffer->size);
    }

    src_rect = (im_rect){0, 0, WIDTH, HEIGHT};
    left = (dst_width - resized_width) / 2;
    top = (dst_height - resized_height) / 2;
    dst_rect = (im_rect){(int)left, (int)top, (int)resized_width, (int)resized_height};

    status = improcess(src, dst, (rga_buffer_t){0},
                       src_rect, dst_rect, (im_rect){0}, IM_SYNC);
    if (status != IM_STATUS_SUCCESS) {
        static int rga_error_logs = 0;
        if (rga_error_logs < 3) {
            fprintf(stderr,
                    "[AI][RGA] preprocess failed: status=%d detail=%s src=%dx%d fmt=%d dst=%dx%d fmt=%d\n",
                    status,
                    imStrError((IM_STATUS)status),
                    src.width,
                    src.height,
                    src.format,
                    dst.width,
                    dst.height,
                    dst.format);
            rga_error_logs++;
        }
        return -1;
    }

    return 0;
}

static int ai_pipeline_start(AiPipeline *ai, DetectSharedState *detect_state) {
    RknnWorkerConfig config;
    const char *model_path;
    int rc;

    if (ai == NULL) {
        return EINVAL;
    }

    memset(ai, 0, sizeof(*ai));
    ai->detect_state = detect_state;
    ai->enabled = env_to_bool_default(AI_ENABLE_ENV, 1);
    ai->frame_interval = env_to_positive_int(AI_INTERVAL_ENV, 3);
    ai->last_stats_log_ms = mono_now_ms();

    if (!ai->enabled) {
        printf("[Parent][AI] RKNN worker disabled by %s\n", AI_ENABLE_ENV);
        return 0;
    }

    rknn_worker_config_defaults(&config);
    model_path = getenv(AI_MODEL_PATH_ENV);
    if (model_path != NULL && model_path[0] != '\0') {
        config.model_path = model_path;
    }
    config.camera_width = WIDTH;
    config.camera_height = HEIGHT;
    config.detect_state = detect_state;
    config.conf_threshold = env_to_float_range(AI_CONF_ENV, config.conf_threshold, 0.01f, 0.99f);
    config.nms_threshold = env_to_float_range(AI_NMS_ENV, config.nms_threshold, 0.01f, 0.99f);

    rc = rknn_worker_start(&ai->worker, &config);
    if (rc != 0) {
        fprintf(stderr, "[Parent][AI] RKNN worker start failed: %s\n", strerror(rc));
        ai->enabled = 0;
        return rc;
    }

    rc = rknn_worker_get_input_info(&ai->worker, &ai->input_info);
    if (rc != 0) {
        fprintf(stderr, "[Parent][AI] RKNN input info unavailable: %s\n", strerror(rc));
        rknn_worker_stop(&ai->worker);
        ai->enabled = 0;
        return rc;
    }

    if (ai->input_info.width == 0 ||
        ai->input_info.height == 0 ||
        ai->input_info.channels != 3 ||
        ai->input_info.size == 0) {
        fprintf(stderr,
                "[Parent][AI] Unsupported RKNN input shape for RGA preprocess: %ux%ux%u size=%u\n",
                ai->input_info.width,
                ai->input_info.height,
                ai->input_info.channels,
                ai->input_info.size);
        rknn_worker_stop(&ai->worker);
        ai->enabled = 0;
        return EINVAL;
    }

    ai->input_size = ai->input_info.size;

    ai->started = 1;
    printf("[Parent][AI] RKNN worker ready, dma-buf input=%ux%ux%u interval=%d model=%s\n",
           ai->input_info.width,
           ai->input_info.height,
           ai->input_info.channels,
           ai->frame_interval,
           config.model_path);
    return 0;
}

static void ai_pipeline_process_frame(AiPipeline *ai, int dma_fd, int64_t frame_wall_ms) {
    RknnWorkerInputBuffer input_buffer;
    int rc;

    if (ai == NULL || !ai->started || !rknn_worker_is_ready(&ai->worker)) {
        return;
    }

    ai->frame_seq++;
    if (ai->frame_interval > 1 &&
        (ai->frame_seq % (uint64_t)ai->frame_interval) != 0) {
        ai->skipped_interval_count++;
        return;
    }

    rc = rknn_worker_acquire_input_buffer(&ai->worker, &input_buffer);
    if (rc == EBUSY) {
        ai->skipped_busy_count++;
        return;
    }
    if (rc != 0) {
        if (rc != ECANCELED) {
            fprintf(stderr, "[Parent][AI] acquire input dma-buf failed: %s\n", strerror(rc));
        }
        return;
    }

    rc = ai_preprocess_dma_to_rgb_fd(dma_fd, &input_buffer);
    if (rc != 0) {
        rknn_worker_release_input_buffer(&ai->worker, input_buffer.slot);
        ai->preproc_fail_count++;
        if (ai->preproc_fail_count == 1 || (ai->preproc_fail_count % 100) == 0) {
            fprintf(stderr, "[Parent][AI] preprocess failed count=%llu\n",
                    (unsigned long long)ai->preproc_fail_count);
        }
        return;
    }

    rc = rknn_worker_submit_input_buffer(&ai->worker,
                                         input_buffer.slot,
                                         (int64_t)ai->frame_seq,
                                         frame_wall_ms);
    if (rc != 0) {
        rknn_worker_release_input_buffer(&ai->worker, input_buffer.slot);
        if (rc != ECANCELED) {
            fprintf(stderr, "[Parent][AI] submit failed: %s\n", strerror(rc));
        }
    }
}

static void ai_pipeline_log_stats(AiPipeline *ai, int64_t now_ms) {
    RknnWorkerStats stats;
    int box_count = 0;

    if (ai == NULL || !ai->started) {
        return;
    }
    if (now_ms - ai->last_stats_log_ms < AI_STATS_INTERVAL_MS) {
        return;
    }

    memset(&stats, 0, sizeof(stats));
    rknn_worker_get_stats(&ai->worker, &stats);
    if (ai->detect_state != NULL && ai->detect_state->valid) {
        box_count = ai->detect_state->box_count;
    }
    printf("[Parent][AI] stats submitted=%llu processed=%llu dropped=%llu failed=%llu candidates=%llu boxes=%d last_us=%lld\n",
           (unsigned long long)stats.submitted,
           (unsigned long long)stats.processed,
           (unsigned long long)stats.dropped,
           (unsigned long long)stats.failed,
           (unsigned long long)stats.decoded_candidates,
           box_count,
           (long long)stats.last_infer_us);
    ai->last_stats_log_ms = now_ms;
}

static void ai_pipeline_stop(AiPipeline *ai) {
    if (ai == NULL) {
        return;
    }

    if (ai->started) {
        rknn_worker_stop(&ai->worker);
        ai->started = 0;
    }
    ai->input_size = 0;
}

static int detect_snapshot_read(DetectSharedState *shared, DetectSharedState *snapshot) {
    uint32_t before;
    uint32_t after;
    int tries;

    if (shared == NULL || snapshot == NULL || !shared->valid) {
        return 0;
    }

    for (tries = 0; tries < 12; tries++) {
        before = shared->version;
        if ((before & 1U) != 0U) {
            continue;
        }
        __sync_synchronize();
        memcpy(snapshot, shared, sizeof(*snapshot));
        __sync_synchronize();
        after = shared->version;
        if (before == after && (after & 1U) == 0U && snapshot->valid) {
            return snapshot->box_count > 0;
        }
    }

    return 0;
}

static int detect_snapshot_read_overlay(DetectSharedState *shared, DetectSharedState *snapshot) {
    uint32_t before;
    uint32_t after;
    int tries;

    if (shared == NULL || snapshot == NULL || (!shared->valid && !shared->zone_valid)) {
        return 0;
    }

    for (tries = 0; tries < 12; tries++) {
        before = shared->version;
        if ((before & 1U) != 0U) {
            continue;
        }
        __sync_synchronize();
        memcpy(snapshot, shared, sizeof(*snapshot));
        __sync_synchronize();
        after = shared->version;
        if (before == after && (after & 1U) == 0U &&
            (snapshot->valid || snapshot->zone_valid)) {
            return snapshot->box_count > 0 || snapshot->zone_valid;
        }
    }

    return 0;
}

static int zone_snapshot_read(ZoneOverlayState *shared, ZoneOverlayState *snapshot) {
    uint32_t before;
    uint32_t after;
    int tries;

    if (shared == NULL || snapshot == NULL || !shared->zone_valid) {
        return 0;
    }

    for (tries = 0; tries < 12; tries++) {
        before = shared->version;
        if ((before & 1U) != 0U) {
            continue;
        }
        __sync_synchronize();
        memcpy(snapshot, shared, sizeof(*snapshot));
        __sync_synchronize();
        after = shared->version;
        if (before == after && (after & 1U) == 0U && snapshot->zone_valid) {
            return 1;
        }
    }

    return 0;
}

typedef struct {
    uint16_t ai_flags;
    uint8_t ai_confidence;
    uint8_t permit_decision;
    uint8_t risk_level;
    uint8_t risk_type;
    uint8_t voice_action;
    uint8_t stm32_action;
    uint8_t explain_code;
    int64_t last_ai_send_ms;
    int64_t last_fusion_send_ms;
    uint16_t last_ai_flags;
    uint8_t last_ai_confidence;
    uint8_t last_permit_decision;
    uint8_t last_risk_level;
    uint8_t last_risk_type;
    uint8_t last_voice_action;
    uint8_t last_stm32_action;
    uint8_t last_explain_code;
    int64_t ppe_first_seen_ms;
    int64_t intrusion_first_seen_ms;
    int64_t sensor_baseline_ms;
    int64_t fire_env_confirm_until_ms;
    uint16_t baseline_smoke;
    int16_t baseline_temperature_x10;
    int fire_locked;
    int fire_lock_sent;
    int emergency_locked;
    int fault_locked;
    int suppress_fusion_send;
    int manual_reset_override;
    int64_t fire_ignore_until_ms;
    uint16_t last_reset_ok_event_id;
    int zone1_power_enabled;
    int zone2_power_enabled;
    int initialized;
} SafetyRuntimeState;

static uint8_t safety_score_to_percent(float score) {
    if (score <= 0.0f) {
        return 0;
    }
    if (score >= 1.0f) {
        return 100;
    }
    return (uint8_t)(score * 100.0f + 0.5f);
}


static float safety_cross2(float ax, float ay, float bx, float by, float px, float py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

static int safety_point_in_zone(float x, float y, const ZoneRect *rect) {
    float c0;
    float c1;
    float c2;
    float c3;
    const float eps = 1.0f;

    if (rect == NULL || !rect->valid) {
        return 0;
    }

    if (x < rect->x1 - eps || x > rect->x2 + eps || y < rect->y1 - eps || y > rect->y2 + eps) {
        return 0;
    }

    if (rect->p0x == 0.0f && rect->p0y == 0.0f &&
        rect->p1x == 0.0f && rect->p1y == 0.0f &&
        rect->p2x == 0.0f && rect->p2y == 0.0f &&
        rect->p3x == 0.0f && rect->p3y == 0.0f) {
        return 1;
    }

    c0 = safety_cross2(rect->p0x, rect->p0y, rect->p1x, rect->p1y, x, y);
    c1 = safety_cross2(rect->p1x, rect->p1y, rect->p2x, rect->p2y, x, y);
    c2 = safety_cross2(rect->p2x, rect->p2y, rect->p3x, rect->p3y, x, y);
    c3 = safety_cross2(rect->p3x, rect->p3y, rect->p0x, rect->p0y, x, y);

    return (c0 >= -eps && c1 >= -eps && c2 >= -eps && c3 >= -eps) ||
           (c0 <= eps && c1 <= eps && c2 <= eps && c3 <= eps);
}

static int safety_box_center_in_zone(const DetectBox *box, const ZoneRect *rect) {
    if (box == NULL) {
        return 0;
    }
    return safety_point_in_zone((box->x1 + box->x2) * 0.5f,
                                (box->y1 + box->y2) * 0.5f,
                                rect);
}

static int safety_box_bottom_in_zone(const DetectBox *box, const ZoneRect *rect) {
    if (box == NULL) {
        return 0;
    }
    return safety_point_in_zone((box->x1 + box->x2) * 0.5f, box->y2, rect);
}

static void safety_build_ai_status(DetectSharedState *shared,
                                   int64_t now_ms,
                                   uint16_t *ai_flags_out,
                                   uint8_t *ai_confidence_out) {
    DetectSharedState snapshot;
    uint16_t flags = 0;
    uint8_t confidence = 0;
    uint8_t fire_confidence = 0;
    int has_helmet = 0;
    int has_vest = 0;
    int has_no_helmet = 0;
    int has_no_vest = 0;
    int has_person = 0;
    int has_fire = 0;
    int zone1_person_count = 0;
    int zone2_person_count = 0;
    int dynamic_zone_detected;
    ZoneRect work_zone1;
    ZoneRect work_zone2;

    dynamic_zone_detected = zone_runtime_get_detected(&work_zone1, &work_zone2);
    if (!dynamic_zone_detected) {
        memset(&work_zone1, 0, sizeof(work_zone1));
        memset(&work_zone2, 0, sizeof(work_zone2));
    }

    if (!detect_snapshot_read(shared, &snapshot) ||
        snapshot.timestamp_ms <= 0 ||
        now_ms - snapshot.timestamp_ms > SAFETY_AI_STALE_MS) {
        *ai_flags_out = 0;
        *ai_confidence_out = 0;
        return;
    }

    flags |= SAFETY_AI_FLAG_VALID;

    for (int i = 0; i < snapshot.box_count; i++) {
        char *name = coco_cls_to_name(snapshot.boxes[i].class_id);
        uint8_t score_percent = safety_score_to_percent(snapshot.boxes[i].score);
        if (score_percent < 65) {
            continue;
        }
        if (score_percent > confidence) {
            confidence = score_percent;
        }

        if (strcmp(name, "helmet") == 0) {
            has_helmet = 1;
        } else if (strcmp(name, "no-helmet") == 0) {
            has_no_helmet = 1;
        } else if (strcmp(name, "vest") == 0) {
            has_vest = 1;
        } else if (strcmp(name, "no-vest") == 0) {
            has_no_vest = 1;
        } else if (strcmp(name, "person") == 0) {
            has_person = 1;
            if (work_zone1.valid &&
                safety_box_bottom_in_zone(&snapshot.boxes[i], &work_zone1)) {
                zone1_person_count++;
            } else if (work_zone2.valid &&
                       safety_box_bottom_in_zone(&snapshot.boxes[i], &work_zone2)) {
                zone2_person_count++;
            }
        } else if (strcmp(name, "fire") == 0) {
            has_fire = 1;
            if (score_percent > fire_confidence) {
                fire_confidence = score_percent;
            }
        }
    }

    if (has_person) {
        flags |= SAFETY_AI_FLAG_PERSON;
    }
    if (has_no_helmet) {
        flags |= SAFETY_AI_FLAG_NO_HELMET;
    }
    if (has_no_vest) {
        flags |= SAFETY_AI_FLAG_NO_VEST;
    }
    if (has_fire) {
        flags |= SAFETY_AI_FLAG_FIRE_OUT;
    }
    if (zone1_person_count == 1) {
        flags |= SAFETY_AI_FLAG_ZONE1_READY;
    } else if (zone1_person_count >= 2) {
        flags |= SAFETY_AI_FLAG_INTRUSION | SAFETY_AI_FLAG_ZONE1_BUSY;
    }
    if (zone2_person_count == 1) {
        flags |= SAFETY_AI_FLAG_ZONE2_READY;
    } else if (zone2_person_count >= 2) {
        flags |= SAFETY_AI_FLAG_INTRUSION | SAFETY_AI_FLAG_ZONE2_BUSY;
    }
    if ((has_no_helmet || has_no_vest) && zone1_person_count > 0) {
        flags |= SAFETY_AI_FLAG_ZONE1_PPE_BAD;
    }
    if ((has_no_helmet || has_no_vest) && zone2_person_count > 0) {
        flags |= SAFETY_AI_FLAG_ZONE2_PPE_BAD;
    }
    if ((has_helmet || !has_person) && has_vest && !has_no_helmet && !has_no_vest) {
        flags |= SAFETY_AI_FLAG_PPE_OK;
    }

    *ai_flags_out = flags;
    *ai_confidence_out = has_fire ? fire_confidence : confidence;
}

static uint8_t safety_zone_power_action(const SafetyRuntimeState *state, int alarm) {
    int zone1_on = state != NULL && state->zone1_power_enabled;
    int zone2_on = state != NULL && state->zone2_power_enabled;

    if (alarm && zone1_on && zone2_on) {
        return SAFETY_STM32_ACTION_ALARM_KEEP_POWER;
    }
    if (zone1_on && zone2_on) {
        return SAFETY_STM32_ACTION_ENABLE_ZONE12;
    }
    if (zone1_on) {
        return alarm ? SAFETY_STM32_ACTION_CUT_ZONE2_ALARM : SAFETY_STM32_ACTION_ENABLE_ZONE1;
    }
    if (zone2_on) {
        return alarm ? SAFETY_STM32_ACTION_CUT_ZONE1_ALARM : SAFETY_STM32_ACTION_ENABLE_ZONE2;
    }
    return alarm ? SAFETY_STM32_ACTION_KEEP_POWER_OFF : SAFETY_STM32_ACTION_STANDBY_POWER_OFF;
}

static void safety_update_sensor_baseline(SafetyRuntimeState *state,
                                          const SafetyStm32Snapshot *stm32,
                                          int64_t frame_mono_ms) {
    if (state == NULL || stm32 == NULL || !stm32->online) {
        return;
    }

    if (state->sensor_baseline_ms == 0 ||
        frame_mono_ms - state->sensor_baseline_ms > SAFETY_CONFIRM_DELAY_MS) {
        state->sensor_baseline_ms = frame_mono_ms;
        state->baseline_smoke = stm32->smoke;
        state->baseline_temperature_x10 = stm32->temperature_x10;
    }
}

static int safety_sensor_rise_high(SafetyRuntimeState *state,
                                   const SafetyStm32Snapshot *stm32,
                                   int64_t frame_mono_ms) {
    int smoke_rise = env_to_positive_int(SAFETY_SMOKE_RISE_ENV, SAFETY_SMOKE_RISE_DEFAULT);
    int temp_rise_x10 = env_to_positive_int(SAFETY_TEMP_RISE_X10_ENV, SAFETY_TEMP_RISE_X10_DEFAULT);

    if (state == NULL || stm32 == NULL || !stm32->online) {
        return 0;
    }

    if (state->sensor_baseline_ms == 0) {
        safety_update_sensor_baseline(state, stm32, frame_mono_ms);
        return 0;
    }

    if (stm32->smoke >= state->baseline_smoke &&
        stm32->smoke - state->baseline_smoke >= (uint16_t)smoke_rise) {
        return 1;
    }
    if (stm32->temperature_x10 >= state->baseline_temperature_x10 &&
        stm32->temperature_x10 - state->baseline_temperature_x10 >= temp_rise_x10) {
        return 1;
    }
    return 0;
}

static void safety_evaluate_fusion(SafetyRuntimeState *state,
                                   uint16_t ai_flags,
                                   const SafetyStm32Snapshot *stm32,
                                   int64_t frame_mono_ms,
                                   uint8_t *permit_decision,
                                   uint8_t *risk_level,
                                   uint8_t *risk_type,
                                   uint8_t *voice_action,
                                   uint8_t *stm32_action,
                                   uint8_t *explain_code) {
    int smoke_high = env_to_positive_int(SAFETY_SMOKE_HIGH_ENV, SAFETY_SMOKE_HIGH_DEFAULT);
    int gas_high = env_to_positive_int(SAFETY_GAS_HIGH_ENV, SAFETY_GAS_HIGH_DEFAULT);
    int temp_high_x10 = env_to_positive_int(SAFETY_TEMP_HIGH_X10_ENV, SAFETY_TEMP_HIGH_X10_DEFAULT);
    int ppe_bad = (ai_flags & (SAFETY_AI_FLAG_NO_HELMET | SAFETY_AI_FLAG_NO_VEST)) != 0;
    int intrusion = (ai_flags & SAFETY_AI_FLAG_INTRUSION) != 0;
    int zone1_busy = (ai_flags & SAFETY_AI_FLAG_ZONE1_BUSY) != 0;
    int zone2_busy = (ai_flags & SAFETY_AI_FLAG_ZONE2_BUSY) != 0;
    int zone1_ready = (ai_flags & SAFETY_AI_FLAG_ZONE1_READY) != 0;
    int zone2_ready = (ai_flags & SAFETY_AI_FLAG_ZONE2_READY) != 0;
    int zone1_ppe_bad = (ai_flags & SAFETY_AI_FLAG_ZONE1_PPE_BAD) != 0;
    int zone2_ppe_bad = (ai_flags & SAFETY_AI_FLAG_ZONE2_PPE_BAD) != 0;
    int stm32_online = stm32 != NULL && stm32->online;
    int reset_ok = stm32 != NULL &&
                   stm32->last_event_type == SAFETY_STM32_CODE_RESET_OK &&
                   stm32->last_event_id != 0 &&
                   (state == NULL || stm32->last_event_id != state->last_reset_ok_event_id);
    int zone_detected = zone_runtime_get_detected(NULL, NULL);
    int zone_blocked = mqtt_get_zone_blocked();
    int zone_confirm_enabled = mqtt_get_zone_confirm_enabled();
    int env_abs_high = stm32_online &&
                       (stm32->smoke >= (uint16_t)smoke_high ||
                        stm32->gas >= (uint16_t)gas_high ||
                        stm32->temperature_x10 >= temp_high_x10);

    if (state != NULL && env_abs_high) {
        state->fire_env_confirm_until_ms = frame_mono_ms + 8000;
    }

    *permit_decision = SAFETY_PERMIT_DENY;
    *risk_level = SAFETY_RISK_WARNING;
    *risk_type = SAFETY_RISK_TYPE_NONE;
    *voice_action = SAFETY_VOICE_NONE;
    *stm32_action = SAFETY_STM32_ACTION_KEEP_POWER_OFF;
    *explain_code = 0;
    if (state != NULL) {
        state->suppress_fusion_send = 0;
    }

    if (state != NULL && reset_ok) {
        state->fire_locked = 0;
        state->fire_lock_sent = 0;
        state->emergency_locked = 0;
        state->fault_locked = 0;
        state->ppe_first_seen_ms = 0;
        state->intrusion_first_seen_ms = 0;
        state->last_fusion_send_ms = 0;
        state->last_permit_decision = 0xFF;
        state->last_risk_level = 0xFF;
        state->last_risk_type = 0xFF;
        state->last_voice_action = 0xFF;
        state->last_stm32_action = 0xFF;
        state->last_explain_code = 0xFF;
        state->manual_reset_override = 1;
        state->fire_ignore_until_ms = frame_mono_ms + SAFETY_RESET_FIRE_IGNORE_MS;
        state->fire_env_confirm_until_ms = 0;
        state->last_reset_ok_event_id = stm32->last_event_id;
        state->zone1_power_enabled = 1;
        state->zone2_power_enabled = 1;
        printf("[Safety] STM32 reset OK event=%u: ignore fire for %d ms\n",
               stm32->last_event_id, SAFETY_RESET_FIRE_IGNORE_MS);
        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_SAFE;
        *risk_type = SAFETY_RISK_TYPE_NONE;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = safety_zone_power_action(state, 0);
        *explain_code = 88;
        return;
    }

    if (state != NULL && state->fire_ignore_until_ms > 0) {
        if (frame_mono_ms < state->fire_ignore_until_ms) {
            if (ai_flags & (SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT)) {
                ai_flags &= (uint16_t)~(SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT);
                state->fire_locked = 0;
                state->fire_lock_sent = 0;
                printf("[Safety] reset fire ignore active: %lld ms left\n",
                       (long long)(state->fire_ignore_until_ms - frame_mono_ms));
            }
        } else {
            state->fire_ignore_until_ms = 0;
        }
    }

    if ((ai_flags & (SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT)) == 0) {
        safety_update_sensor_baseline(state, stm32, frame_mono_ms);
    }

    if (state != NULL && state->manual_reset_override) {
        if ((ai_flags & (SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT |
                         SAFETY_AI_FLAG_INTRUSION | SAFETY_AI_FLAG_NO_HELMET |
                         SAFETY_AI_FLAG_NO_VEST | SAFETY_AI_FLAG_ZONE1_BUSY |
                         SAFETY_AI_FLAG_ZONE2_BUSY)) == 0 &&
            !(stm32_online && stm32->last_event_type == SAFETY_STM32_CODE_EMERGENCY_STOP) &&
            !(stm32_online && stm32->last_event_type == SAFETY_STM32_CODE_FAULT) &&
            !(stm32_online && stm32->fault_code != 0)) {
            *permit_decision = SAFETY_PERMIT_ALLOW;
            *risk_level = SAFETY_RISK_SAFE;
            *risk_type = SAFETY_RISK_TYPE_NONE;
            *voice_action = SAFETY_VOICE_NONE;
            *stm32_action = safety_zone_power_action(state, 0);
            *explain_code = 88;
            return;
        }
        printf("[Safety] manual reset override exit: new event detected\n");
        state->manual_reset_override = 0;
    }

    if ((stm32_online && stm32->last_event_type == SAFETY_STM32_CODE_EMERGENCY_STOP) ||
        (state != NULL && state->emergency_locked)) {
        if (state != NULL) {
            state->emergency_locked = 1;
        }
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_EMERGENCY_STOP;
        *voice_action = SAFETY_VOICE_EMERGENCY_STOP;
        *stm32_action = SAFETY_STM32_ACTION_LOCKOUT_WAIT_RESET;
        *explain_code = 85;
        return;
    }

    if ((stm32_online && stm32->fault_code != 0) ||
        (stm32_online && stm32->last_event_type == SAFETY_STM32_CODE_FAULT) ||
        (state != NULL && state->fault_locked)) {
        if (state != NULL) {
            state->fault_locked = 1;
        }
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_STM32_FAULT;
        *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_ALARM;
        *explain_code = 82;
        return;
    }

    if ((stm32_online && stm32->last_event_type == SAFETY_STM32_CODE_RESET_WAIT) ||
        (stm32_online && (stm32->actuator_flags & SAFETY_ACT_RESET_WAIT))) {
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_STM32_FAULT;
        *voice_action = SAFETY_VOICE_WAIT_RESET;
        *stm32_action = SAFETY_STM32_ACTION_LOCKOUT_WAIT_RESET;
        *explain_code = 87;
        return;
    }

    if (state != NULL && state->fire_locked) {
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_FIRE;
        *voice_action = SAFETY_VOICE_FIRE_WARNING;
        *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
        *explain_code = 42;
        return;
    }

    if (!zone_detected) {
        if (zone_blocked || !zone_confirm_enabled) {
            *risk_level = SAFETY_RISK_DANGER;
            *risk_type = SAFETY_RISK_TYPE_NONE;
            *voice_action = SAFETY_VOICE_NONE;
            *stm32_action = SAFETY_STM32_ACTION_KEEP_POWER_OFF;
            *explain_code = 90;
            return;
        }
    }

    if ((ai_flags & SAFETY_AI_FLAG_VALID) == 0) {
        if (!stm32_online) {
            *permit_decision = SAFETY_PERMIT_ALLOW;
            *risk_level = SAFETY_RISK_WARNING;
            *risk_type = SAFETY_RISK_TYPE_STM32_LINK;
            *voice_action = SAFETY_VOICE_NONE;
            *stm32_action = SAFETY_STM32_ACTION_STANDBY_POWER_OFF;
            *explain_code = 81;
            return;
        }
        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_SAFE;
        *risk_type = SAFETY_RISK_TYPE_NONE;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = safety_zone_power_action(state, 0);
        *explain_code = 12;
        return;
    }

    if (ai_flags & (SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT)) {
        if (safety_sensor_rise_high(state, stm32, frame_mono_ms) ||
            env_abs_high ||
            (state != NULL && state->fire_env_confirm_until_ms > frame_mono_ms)) {
            if (state != NULL) {
                state->fire_locked = 1;
            }
            *risk_level = SAFETY_RISK_CRITICAL;
            *risk_type = SAFETY_RISK_TYPE_FIRE;
            *voice_action = SAFETY_VOICE_FIRE_WARNING;
            *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
            *explain_code = 42;
            return;
        }
    }

    if (0 && stm32_online && stm32->smoke >= (uint16_t)smoke_high) {
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_ENV;
        *voice_action = SAFETY_VOICE_ENV_WARNING;
        *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
        *explain_code = 61;
        return;
    }
    if (0 && stm32_online && stm32->gas >= (uint16_t)gas_high) {
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_ENV;
        *voice_action = SAFETY_VOICE_ENV_WARNING;
        *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
        *explain_code = 62;
        return;
    }
    if (0 && stm32_online && stm32->temperature_x10 >= temp_high_x10) {
        *risk_level = SAFETY_RISK_CRITICAL;
        *risk_type = SAFETY_RISK_TYPE_ENV;
        *voice_action = SAFETY_VOICE_ENV_WARNING;
        *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
        *explain_code = 63;
        return;
    }

    if (intrusion) {
        if (state != NULL && state->intrusion_first_seen_ms == 0) {
            state->intrusion_first_seen_ms = frame_mono_ms;
        }
    } else if (state != NULL) {
        state->intrusion_first_seen_ms = 0;
    }

    if (intrusion) {
        *risk_level = SAFETY_RISK_DANGER;
        *risk_type = SAFETY_RISK_TYPE_INTRUSION;
        *voice_action = SAFETY_VOICE_INTRUSION_WARNING;
        if (zone1_busy && zone2_busy) {
            *stm32_action = SAFETY_STM32_ACTION_CUT_ZONE12_ALARM;
            *explain_code = 53;
        } else if (zone2_busy) {
            *stm32_action = SAFETY_STM32_ACTION_CUT_ZONE2_ALARM;
            *explain_code = 52;
        } else {
            *stm32_action = SAFETY_STM32_ACTION_CUT_ZONE1_ALARM;
            *explain_code = 51;
        }
        return;
    }


    if (ai_flags & (SAFETY_AI_FLAG_FIRE | SAFETY_AI_FLAG_FIRE_OUT)) {
        if (safety_sensor_rise_high(state, stm32, frame_mono_ms) ||
            env_abs_high ||
            (state != NULL && state->fire_env_confirm_until_ms > frame_mono_ms)) {
            if (state != NULL) {
                state->fire_locked = 1;
            }
            *risk_level = SAFETY_RISK_CRITICAL;
            *risk_type = SAFETY_RISK_TYPE_FIRE;
            *voice_action = SAFETY_VOICE_FIRE_WARNING;
            *stm32_action = SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM;
            *explain_code = 42;
            return;
        }

        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_SAFE;
        *risk_type = SAFETY_RISK_TYPE_NONE;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = safety_zone_power_action(state, 0);
        *explain_code = 43;
        return;
    }

    if (ppe_bad) {
        if (state != NULL && state->ppe_first_seen_ms == 0) {
            state->ppe_first_seen_ms = frame_mono_ms;
        }
        *risk_type = SAFETY_RISK_TYPE_PPE;
        *voice_action = SAFETY_VOICE_PPE_WARNING;
        if (state != NULL) {
            if (zone1_ppe_bad) {
                state->zone1_power_enabled = 0;
            }
            if (zone2_ppe_bad) {
                state->zone2_power_enabled = 0;
            }
        }
        *stm32_action = safety_zone_power_action(state, 1);
        if ((ai_flags & SAFETY_AI_FLAG_NO_HELMET) && (ai_flags & SAFETY_AI_FLAG_NO_VEST)) {
            *explain_code = 24;
        } else if (ai_flags & SAFETY_AI_FLAG_NO_VEST) {
            *explain_code = 22;
        } else {
            *explain_code = 21;
        }
        if (state != NULL &&
            frame_mono_ms - state->ppe_first_seen_ms < SAFETY_CONFIRM_DELAY_MS) {
            *risk_type = SAFETY_RISK_TYPE_NONE;
            *stm32_action = safety_zone_power_action(state, 0);
            state->suppress_fusion_send = 1;
        }
        return;
    } else if (state != NULL) {
        state->ppe_first_seen_ms = 0;
    }

    if (!stm32_online) {
        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_WARNING;
        *risk_type = SAFETY_RISK_TYPE_STM32_LINK;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = SAFETY_STM32_ACTION_STANDBY_POWER_OFF;
        *explain_code = 81;
        return;
    }

    if (ai_flags & SAFETY_AI_FLAG_PPE_OK) {
        if (state != NULL) {
            if (zone1_ready) {
                state->zone1_power_enabled = 1;
            }
            if (zone2_ready) {
                state->zone2_power_enabled = 1;
            }
        }
        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_SAFE;
        *risk_type = SAFETY_RISK_TYPE_NONE;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = safety_zone_power_action(state, 0);
        *explain_code = 11;
        return;
    }

    if ((ai_flags & SAFETY_AI_FLAG_PERSON) == 0) {
        *permit_decision = SAFETY_PERMIT_ALLOW;
        *risk_level = SAFETY_RISK_SAFE;
        *risk_type = SAFETY_RISK_TYPE_NONE;
        *voice_action = SAFETY_VOICE_NONE;
        *stm32_action = safety_zone_power_action(state, 0);
        *explain_code = 12;
        return;
    }

    *permit_decision = SAFETY_PERMIT_ALLOW;
    *risk_level = SAFETY_RISK_SAFE;
    *risk_type = SAFETY_RISK_TYPE_NONE;
    *voice_action = SAFETY_VOICE_NONE;
    *stm32_action = safety_zone_power_action(state, 0);
    *explain_code = 13;
}

static uint8_t safety_ai_flags_to_detect_state(uint16_t ai_flags) {
    int no_helmet = (ai_flags & SAFETY_AI_FLAG_NO_HELMET) != 0;
    int no_vest = (ai_flags & SAFETY_AI_FLAG_NO_VEST) != 0;

    if ((ai_flags & SAFETY_AI_FLAG_VALID) == 0) {
        return AI_DETECT_STATE_NONE;
    }
    if (ai_flags & SAFETY_AI_FLAG_INTRUSION) {
        return AI_DETECT_STATE_INTRUSION;
    }
    if (ai_flags & SAFETY_AI_FLAG_FIRE_OUT) {
        return AI_DETECT_STATE_FIRE_OUT_ZONE;
    }
    if (ai_flags & SAFETY_AI_FLAG_FIRE) {
        return AI_DETECT_STATE_FIRE_WORK_ZONE;
    }
    if (no_helmet && no_vest) {
        return AI_DETECT_STATE_PPE_BOTH_BAD;
    }
    if (no_helmet) {
        return AI_DETECT_STATE_NO_HELMET;
    }
    if (no_vest) {
        return AI_DETECT_STATE_NO_VEST;
    }
    if (ai_flags & SAFETY_AI_FLAG_PPE_OK) {
        return AI_DETECT_STATE_PPE_OK;
    }
    return AI_DETECT_STATE_NONE;
}

static int safety_work_zone_configured(void) {
    return zone_runtime_get_detected(NULL, NULL);
}

static void safety_audio_update_from_fusion(const SafetyRuntimeState *state,
                                            int64_t frame_wall_ms) {
    int command = 0;

    if (state == NULL || state->voice_action == SAFETY_VOICE_NONE) {
        return;
    }

    if (state->voice_action == SAFETY_VOICE_FIRE_WARNING) {
        if (state->explain_code == 42 || !safety_work_zone_configured()) {
            command = 1;
        }
    } else if (state->voice_action == SAFETY_VOICE_INTRUSION_WARNING) {
        if (state->explain_code == 52) {
            command = 3;
        } else {
            command = 2;
        }
    } else if (state->voice_action == SAFETY_VOICE_PPE_WARNING) {
        command = 4;
    }

    audio_alert_send_command(command, frame_wall_ms);
}

static void safety_runtime_update(SafetyInterlockClient *client,
                                  SafetyRuntimeState *state,
                                  DetectSharedState *detect_state,
                                  int64_t frame_wall_ms,
                                  int64_t frame_mono_ms) {
    SafetyStm32Snapshot stm32_snapshot;
    int ai_changed;
    int fusion_changed;
    uint8_t ai_detect_state;
    uint8_t last_ai_detect_state;

    if (client == NULL || state == NULL) {
        return;
    }

    safety_build_ai_status(detect_state,
                           frame_wall_ms,
                           &state->ai_flags,
                           &state->ai_confidence);
    safety_client_get_snapshot(client, &stm32_snapshot);
    safety_evaluate_fusion(state,
                           state->ai_flags,
                           &stm32_snapshot,
                           frame_mono_ms,
                           &state->permit_decision,
                           &state->risk_level,
                           &state->risk_type,
                           &state->voice_action,
                           &state->stm32_action,
                           &state->explain_code);
    ai_detect_state = safety_ai_flags_to_detect_state(state->ai_flags);
    if (state->fire_locked && ai_detect_state == AI_DETECT_STATE_NONE) {
        ai_detect_state = AI_DETECT_STATE_FIRE_OUT_ZONE;
    }
    last_ai_detect_state = safety_ai_flags_to_detect_state(state->last_ai_flags);
    safety_client_update_ai_detect_state(client,
                                         ai_detect_state,
                                         state->ai_confidence,
                                         frame_wall_ms);
    if (ai_detect_state != AI_DETECT_STATE_NONE &&
        (!state->initialized || ai_detect_state != last_ai_detect_state)) {
        mqtt_request_immediate_ai_report(ai_detect_state);
    }
    safety_audio_update_from_fusion(state, frame_wall_ms);

    ai_changed = !state->initialized ||
                 state->ai_flags != state->last_ai_flags ||
                 state->ai_confidence != state->last_ai_confidence;
    fusion_changed = !state->initialized ||
                     state->permit_decision != state->last_permit_decision ||
                     state->risk_level != state->last_risk_level ||
                     state->risk_type != state->last_risk_type ||
                     state->voice_action != state->last_voice_action ||
                     state->stm32_action != state->last_stm32_action ||
                     state->explain_code != state->last_explain_code;

    if (ai_changed ||
        state->last_ai_send_ms == 0 ||
        frame_mono_ms - state->last_ai_send_ms >= SAFETY_AI_SEND_INTERVAL_MS) {
        if (safety_client_send_ai_status(client, state->ai_flags, state->ai_confidence) == 0) {
            state->last_ai_flags = state->ai_flags;
            state->last_ai_confidence = state->ai_confidence;
            state->last_ai_send_ms = frame_mono_ms;
        }
    }

    if (!state->suppress_fusion_send &&
        (fusion_changed ||
        state->last_fusion_send_ms == 0 ||
        frame_mono_ms - state->last_fusion_send_ms >= SAFETY_FUSION_SEND_INTERVAL_MS)) {
        if (safety_client_send_fusion_decision(client,
                                               state->permit_decision,
                                               state->risk_level,
                                               state->risk_type,
                                               state->voice_action,
                                               state->stm32_action,
                                               state->explain_code) == 0) {
            state->last_permit_decision = state->permit_decision;
            state->last_risk_level = state->risk_level;
            state->last_risk_type = state->risk_type;
            state->last_voice_action = state->voice_action;
            state->last_stm32_action = state->stm32_action;
            state->last_explain_code = state->explain_code;
            state->last_fusion_send_ms = frame_mono_ms;
            if (state->risk_type == SAFETY_RISK_TYPE_FIRE &&
                state->risk_level == SAFETY_RISK_CRITICAL) {
                state->fire_lock_sent = 1;
            }
        }
    }

    state->initialized = 1;
}

static int video_uploader_macro_ready(char *reason, size_t reason_size) {
    int has_url = VIDEO_UPLOADER_HTTP_UPLOAD_URL[0] != '\0';
    int has_token = VIDEO_UPLOADER_HTTP_AUTH_TOKEN[0] != '\0';
    int has_device_id = VIDEO_UPLOADER_HTTP_DEVICE_ID[0] != '\0';
    int offset = 0;
    int rc;

    if (reason != NULL && reason_size > 0) {
        reason[0] = '\0';
    }

    if (has_url && has_token && has_device_id) {
        return 1;
    }

    if (reason == NULL || reason_size == 0) {
        return 0;
    }

    rc = snprintf(reason + offset, reason_size - (size_t)offset,
                  "missing macro:");
    if (rc < 0 || (size_t)rc >= reason_size - (size_t)offset) {
        reason[reason_size - 1] = '\0';
        return 0;
    }
    offset += rc;

    if (!has_url && offset < (int)reason_size) {
        rc = snprintf(reason + offset, reason_size - (size_t)offset,
                      " VIDEO_UPLOADER_HTTP_UPLOAD_URL");
        if (rc < 0 || (size_t)rc >= reason_size - (size_t)offset) {
            reason[reason_size - 1] = '\0';
            return 0;
        }
        offset += rc;
    }

    if (!has_token && offset < (int)reason_size) {
        rc = snprintf(reason + offset, reason_size - (size_t)offset,
                      " VIDEO_UPLOADER_HTTP_AUTH_TOKEN");
        if (rc < 0 || (size_t)rc >= reason_size - (size_t)offset) {
            reason[reason_size - 1] = '\0';
            return 0;
        }
        offset += rc;
    }

    if (!has_device_id && offset < (int)reason_size) {
        rc = snprintf(reason + offset, reason_size - (size_t)offset,
                      " VIDEO_UPLOADER_HTTP_DEVICE_ID");
        if (rc < 0 || (size_t)rc >= reason_size - (size_t)offset) {
            reason[reason_size - 1] = '\0';
            return 0;
        }
    }

    return 0;
}

static void mark_stream_offline(StreamState *stream) {
    pid_t ret = 0;

    if (stream->parent_sock >= 0) {
        close(stream->parent_sock);
        stream->parent_sock = -1;
    }

    if (stream->child_pid > 0) {
        ret = waitpid(stream->child_pid, NULL, WNOHANG);
        if (ret == stream->child_pid) {
            stream->child_pid = -1;
        }
    }

    stream->stream_online = 0;
    stream->last_restart_mono_ms = mono_now_ms();
}

static void stop_stream_child(StreamState *stream) {
    if (stream->parent_sock >= 0) {
        close(stream->parent_sock);
        stream->parent_sock = -1;
    }

    if (stream->child_pid > 0) {
        int status;
        int waited_ms = 0;

        /*
         * Let the child close an active target-recording segment first. If it
         * does not exit quickly, fall back to SIGKILL to avoid hanging on MPP.
         */
        kill(stream->child_pid, SIGTERM);
        while (waited_ms < 1500) {
            pid_t ret = waitpid(stream->child_pid, &status, WNOHANG);
            if (ret == stream->child_pid) {
                stream->child_pid = -1;
                break;
            }
            usleep(100000);
            waited_ms += 100;
        }
        if (stream->child_pid > 0) {
            kill(stream->child_pid, SIGKILL);
            waitpid(stream->child_pid, NULL, 0);
            stream->child_pid = -1;
        }
    }

    stream->stream_online = 0;
}

static void child_reset_streamer(ChildOutputCtx *ctx) {
    memset(&ctx->streamer, 0, sizeof(ctx->streamer));
    ctx->streamer_ready = 0;
}

static int child_open_rtmp_output(ChildOutputCtx *ctx) {
    child_reset_streamer(ctx);

    if (streamer_init(&ctx->streamer, RTMP_URL, WIDTH, HEIGHT, env_to_positive_int(STREAM_FPS_ENV, STREAM_FPS_DEFAULT)) < 0) {
        fprintf(stderr, "[Child] Streamer init failed for RTMP\n");
        child_reset_streamer(ctx);
        return -1;
    }

    ctx->streamer_ready = 1;
    ctx->mode = CHILD_OUTPUT_RTMP;
    child_reset_rtmp_backoff(ctx);
    return 0;
}

static int child_close_file_segment(ChildOutputCtx *ctx, int broken) {
    int64_t end_ms;
    int64_t size_bytes = 0;
    int64_t bytes_after = 0;
    int deleted_count = 0;
    int rc = 0;

    if (ctx->mode != CHILD_OUTPUT_FILE || !ctx->streamer_ready) {
        return 0;
    }

    end_ms = wall_now_ms();
    streamer_clean(&ctx->streamer);
    child_reset_streamer(ctx);

    rc = video_store_get_file_size(ctx->current_segment_path, &size_bytes);
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to stat segment %s: %d\n",
                ctx->current_segment_path, rc);
        size_bytes = 0;
    }

    if (ctx->current_segment_id > 0) {
        if (broken) {
            rc = video_store_mark_segment_broken(&ctx->video_store,
                                                 ctx->current_segment_id,
                                                 end_ms,
                                                 size_bytes);
        } else {
            rc = video_store_finish_segment(&ctx->video_store,
                                            ctx->current_segment_id,
                                            end_ms,
                                            size_bytes);
        }
        if (rc != 0) {
            fprintf(stderr, "[Child] Failed to update segment metadata: %d\n", rc);
        }
    }

    rc = video_store_prune(&ctx->video_store, &bytes_after, &deleted_count);
    if (rc == 0 && deleted_count > 0) {
        printf("[Child] Video prune done, deleted=%d, bytes_after=%lld\n",
               deleted_count, (long long)bytes_after);
    }

    ctx->current_segment_id = 0;
    ctx->current_segment_start_wall_ms = 0;
    ctx->current_segment_start_mono_ms = 0;
    ctx->current_segment_path[0] = '\0';
    return 0;
}

static int child_open_file_segment(ChildOutputCtx *ctx,
                                   int64_t start_wall_ms,
                                   int64_t start_mono_ms) {
    int rc;
    int64_t segment_id = 0;
    char segment_path[PATH_MAX];

    if (!ctx->video_store_ready) {
        return -1;
    }

    rc = video_store_build_segment_path(&ctx->video_store,
                                        start_wall_ms,
                                        segment_path,
                                        sizeof(segment_path));
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to build segment path: %d\n", rc);
        return -1;
    }

    rc = video_store_begin_segment(&ctx->video_store,
                                   start_wall_ms,
                                   segment_path,
                                   &segment_id);
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to register segment: %d\n", rc);
        return -1;
    }

    child_reset_streamer(ctx);
    if (streamer_init(&ctx->streamer, segment_path, WIDTH, HEIGHT, env_to_positive_int(STREAM_FPS_ENV, STREAM_FPS_DEFAULT)) < 0) {
        fprintf(stderr, "[Child] Streamer init failed for local segment\n");
        local_store_delete_video_segment(&ctx->store, segment_id);
        child_reset_streamer(ctx);
        return -1;
    }

    ctx->streamer_ready = 1;
    ctx->mode = CHILD_OUTPUT_FILE;
    ctx->current_segment_id = segment_id;
    ctx->current_segment_start_wall_ms = start_wall_ms;
    ctx->current_segment_start_mono_ms = start_mono_ms;
    snprintf(ctx->current_segment_path, sizeof(ctx->current_segment_path), "%s", segment_path);

    printf("[Child] Local segment started: %s\n", ctx->current_segment_path);
    return 0;
}

static void child_reset_record_streamer(ChildOutputCtx *ctx) {
    ctx->record_streamer_ready = 0;
}

static int child_close_record_segment(ChildOutputCtx *ctx, int broken) {
    int64_t end_ms;
    int64_t size_bytes = 0;
    int rc = 0;

    if (!ctx->record_streamer_ready) {
        return 0;
    }

    end_ms = wall_now_ms();
    streamer_stop_side_record(&ctx->streamer);
    child_reset_record_streamer(ctx);

    rc = video_store_get_file_size(ctx->record_segment_path, &size_bytes);
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to stat target segment %s: %d\n",
                ctx->record_segment_path, rc);
        size_bytes = 0;
    }

    if (ctx->record_segment_id > 0) {
        if (!ctx->record_segment_has_target && !broken && ctx->mode != CHILD_OUTPUT_FILE) {
            if (ctx->record_segment_path[0] != '\0' && unlink(ctx->record_segment_path) != 0 && errno != ENOENT) {
                fprintf(stderr, "[Child] Failed to delete non-event segment %s: %s\n",
                        ctx->record_segment_path, strerror(errno));
            }
            rc = local_store_delete_video_segment(&ctx->store, ctx->record_segment_id);
            if (rc != 0) {
                fprintf(stderr, "[Child] Failed to delete non-event segment metadata: %d\n", rc);
            }
            printf("[Child] Non-event segment deleted: %s size=%lld\n",
                   ctx->record_segment_path, (long long)size_bytes);
        } else if (broken) {
            rc = video_store_mark_segment_broken(&ctx->video_store,
                                                 ctx->record_segment_id,
                                                 end_ms,
                                                 size_bytes);
            if (rc != 0) {
                fprintf(stderr, "[Child] Failed to mark target segment broken: %d\n", rc);
            }
        } else {
            rc = video_store_finish_segment(&ctx->video_store,
                                            ctx->record_segment_id,
                                            end_ms,
                                            size_bytes);
            if (rc != 0) {
                fprintf(stderr, "[Child] Failed to finish target segment metadata: %d\n", rc);
            }
            printf("[%s] segment kept: %s size=%lld\n",
                   ctx->record_segment_has_target ? "Child] Event" : "Child] Offline",
                   ctx->record_segment_path, (long long)size_bytes);
        }
    }
    ctx->record_segment_id = 0;
    ctx->record_segment_start_wall_ms = 0;
    ctx->record_segment_start_mono_ms = 0;
    ctx->record_segment_has_target = 0;
    ctx->record_segment_path[0] = '\0';
    return 0;
}

static int child_open_record_segment(ChildOutputCtx *ctx,
                                     int64_t start_wall_ms,
                                     int64_t start_mono_ms) {
    int rc;
    int64_t segment_id = 0;
    char segment_path[PATH_MAX];

    if (!ctx->video_store_ready || ctx->record_streamer_ready) {
        return ctx->record_streamer_ready ? 0 : -1;
    }

    rc = video_store_build_segment_path(&ctx->video_store,
                                        start_wall_ms,
                                        segment_path,
                                        sizeof(segment_path));
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to build target segment path: %d\n", rc);
        return -1;
    }

    rc = video_store_begin_segment(&ctx->video_store,
                                   start_wall_ms,
                                   segment_path,
                                   &segment_id);
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to register target segment: %d\n", rc);
        return -1;
    }

    child_reset_record_streamer(ctx);
    if (!ctx->streamer_ready || streamer_start_side_record(&ctx->streamer, segment_path) < 0) {
        fprintf(stderr, "[Child] Side record muxer init failed for target segment\n");
        local_store_delete_video_segment(&ctx->store, segment_id);
        child_reset_record_streamer(ctx);
        return -1;
    }

    ctx->record_streamer_ready = 1;
    ctx->record_segment_id = segment_id;
    ctx->record_segment_start_wall_ms = start_wall_ms;
    ctx->record_segment_start_mono_ms = start_mono_ms;
    snprintf(ctx->record_segment_path, sizeof(ctx->record_segment_path), "%s", segment_path);
    printf("[Child] Continuous local segment started: %s\n", ctx->record_segment_path);
    return 0;
}

static int child_overlay_has_target(const DetectSharedState *overlay,
                                    int64_t frame_wall_ms) {
    return overlay != NULL &&
           overlay->valid &&
           overlay->box_count > 0 &&
           overlay->timestamp_ms > 0 &&
           frame_wall_ms - overlay->timestamp_ms <= SAFETY_AI_STALE_MS;
}

static int child_update_target_recording(ChildOutputCtx *ctx,
                                         int dma_fd,
                                         const DetectSharedState *overlay,
                                         int64_t frame_mono_ms,
                                         int64_t frame_wall_ms) {
    int has_target = child_overlay_has_target(overlay, frame_wall_ms);
    int in_event_keep_window;
    int64_t segment_ms;

    (void)dma_fd;

    if (!ctx->video_store_ready || !ctx->streamer_ready) {
        return 0;
    }

    if (has_target) {
        ctx->record_target_last_seen_mono_ms = frame_mono_ms;
    }

    in_event_keep_window =
        ctx->record_target_last_seen_mono_ms > 0 &&
        frame_mono_ms - ctx->record_target_last_seen_mono_ms <= CHILD_TARGET_RECORD_SEGMENT_MIN_MS;

    if (!ctx->record_streamer_ready) {
        if (child_open_record_segment(ctx, frame_wall_ms, frame_mono_ms) != 0) {
            return 0;
        }
    }

    if (has_target || in_event_keep_window) {
        ctx->record_segment_has_target = 1;
    }

    segment_ms = ctx->video_store.segment_duration_ms > 0 ?
                 ctx->video_store.segment_duration_ms :
                 CHILD_TARGET_RECORD_SEGMENT_MIN_MS;

    if (ctx->record_streamer_ready &&
        frame_mono_ms - ctx->record_segment_start_mono_ms >= segment_ms) {
        child_close_record_segment(ctx, 0);
        child_open_record_segment(ctx, frame_wall_ms, frame_mono_ms);
        if (has_target || in_event_keep_window) {
            ctx->record_segment_has_target = 1;
        }
    }

    return 0;
}

static int child_switch_to_file_mode(ChildOutputCtx *ctx,
                                     int64_t frame_mono_ms,
                                     int64_t frame_wall_ms) {
    if (ctx->streamer_ready) {
        streamer_clean(&ctx->streamer);
        child_reset_streamer(ctx);
    }

    ctx->mode = CHILD_OUTPUT_FILE;
    ctx->last_rtmp_retry_mono_ms = frame_mono_ms;
    child_reset_rtmp_backoff(ctx);

    return child_open_file_segment(ctx, frame_wall_ms, frame_mono_ms);
}

static int child_rotate_or_restore_output(ChildOutputCtx *ctx,
                                          int64_t frame_mono_ms,
                                          int64_t frame_wall_ms) {
    int rc;

    if (ctx->mode != CHILD_OUTPUT_FILE || !ctx->streamer_ready) {
        return 0;
    }

    if (frame_mono_ms - ctx->current_segment_start_mono_ms < ctx->video_store.segment_duration_ms) {
        return 0;
    }

    child_close_file_segment(ctx, 0);

    if (frame_mono_ms - ctx->last_rtmp_retry_mono_ms >= ctx->rtmp_retry_backoff_ms) {
        ctx->last_rtmp_retry_mono_ms = frame_mono_ms;
        if (child_open_rtmp_output(ctx) == 0) {
            printf("[Child] RTMP output restored.\n");
            return 0;
        }

        child_increase_rtmp_backoff(ctx);
        printf("[Child] RTMP restore failed, next backoff=%d ms.\n",
               ctx->rtmp_retry_backoff_ms);
    }

    rc = child_open_file_segment(ctx, frame_wall_ms, frame_mono_ms);
    if (rc != 0) {
        fprintf(stderr, "[Child] Failed to open next local segment.\n");
        return -1;
    }

    return 0;
}


static int child_push_frame_repeated(ChildOutputCtx *ctx,
                                     int dma_fd,
                                     const DetectSharedState *overlay,
                                     const ZoneOverlayState *zone_overlay,
                                     int repeat_count) {
    int ret = 0;
    int i;

    if (repeat_count < 1) {
        repeat_count = 1;
    }

    for (i = 0; i < repeat_count; i++) {
        ret = streamer_push_zerocopy_overlay(&ctx->streamer, dma_fd, overlay, zone_overlay);
        if (ret < 0) {
            return ret;
        }
    }

    return ret;
}

static int child_stream_loop(int sock, DetectSharedState *detect_state, ZoneOverlayState *zone_state) {
    ChildOutputCtx ctx;
    int64_t frame_mono_ms;
    int64_t frame_wall_ms;
    int child_fd;
    int ret;
    int duplicate_frames;

    install_signal_handlers();

    memset(&ctx, 0, sizeof(ctx));
    ctx.detect_state = detect_state;
    ctx.zone_state = zone_state;
    ctx.rtmp_retry_backoff_ms = CHILD_RTMP_RETRY_BASE_MS;
    ctx.rtmp_retry_max_ms = CHILD_RTMP_RETRY_MAX_MS;
    duplicate_frames = env_to_positive_int(STREAM_DUP_FRAMES_ENV, STREAM_DUP_FRAMES_DEFAULT);
    if (duplicate_frames < 1) duplicate_frames = 1;
    printf("[Child] stream duplicate_frames=%d target_fps=%d\n",
           duplicate_frames,
           env_to_positive_int(STREAM_FPS_ENV, STREAM_FPS_DEFAULT));

    ctx.debug_rtmp_fail_after_frames = env_to_positive_int("CAMERA_FLOW_DEBUG_RTMP_FAIL_AFTER_FRAMES", 0);
    if (ctx.debug_rtmp_fail_after_frames > 0) {
        printf("[Child] Debug RTMP fail will trigger after %d frames.\n",
               ctx.debug_rtmp_fail_after_frames);
    }

    if (local_store_open(&ctx.store, NULL) == 0) {
        ctx.store_ready = 1;
        if (video_store_init(&ctx.video_store, &ctx.store, NULL) == 0) {
            ctx.video_store_ready = 1;
            local_store_recover_recording_video_segments(&ctx.store);
            printf("[Child] Video store ready, segment_ms=%lld, high_water=%lld, low_water=%lld\n",
                   (long long)ctx.video_store.segment_duration_ms,
                   (long long)ctx.video_store.high_water_bytes,
                   (long long)ctx.video_store.low_water_bytes);
        } else {
            fprintf(stderr, "[Child] video_store_init failed\n");
        }
    } else {
        fprintf(stderr, "[Child] local_store_open failed, file mode unavailable\n");
    }

    if (child_open_rtmp_output(&ctx) < 0) {
        fprintf(stderr, "[Child] RTMP init failed, switch to local file mode.\n");
        if (child_switch_to_file_mode(&ctx, mono_now_ms(), wall_now_ms()) < 0) {
            return -1;
        }
    }

    while (is_running && (child_fd = recv_fd(sock)) >= 0) {
        const DetectSharedState *overlay = NULL;
        const ZoneOverlayState *zone_overlay = NULL;

        frame_mono_ms = mono_now_ms();
        frame_wall_ms = wall_now_ms();
        if (detect_snapshot_read(ctx.detect_state, &ctx.detect_snapshot)) {
            ctx.last_ai_snapshot = ctx.detect_snapshot;
            overlay = &ctx.detect_snapshot;
        } else if (ctx.last_ai_snapshot.valid &&
                   ctx.last_ai_snapshot.box_count > 0 &&
                   ctx.last_ai_snapshot.timestamp_ms > 0 &&
                   frame_wall_ms - ctx.last_ai_snapshot.timestamp_ms <= SAFETY_AI_STALE_MS) {
            overlay = &ctx.last_ai_snapshot;
        }
        if (zone_snapshot_read(ctx.zone_state, &ctx.zone_snapshot)) {
            zone_overlay = &ctx.zone_snapshot;
        }

        if (ctx.mode == CHILD_OUTPUT_FILE) {
            ret = child_rotate_or_restore_output(&ctx, frame_mono_ms, frame_wall_ms);
            if (ret < 0) {
                close(child_fd);
                break;
            }
        }

        child_update_target_recording(&ctx,
                                      child_fd,
                                      overlay,
                                      frame_mono_ms,
                                      frame_wall_ms);

        if (ctx.mode == CHILD_OUTPUT_RTMP &&
            ctx.debug_rtmp_fail_after_frames > 0 &&
            !ctx.debug_rtmp_fail_triggered) {
            ctx.debug_rtmp_frame_count++;
            if (ctx.debug_rtmp_frame_count >= ctx.debug_rtmp_fail_after_frames) {
                ctx.debug_rtmp_fail_triggered = 1;
                ret = -1;
                fprintf(stderr, "[Child] Debug: simulate RTMP disconnect at frame %d.\n",
                        ctx.debug_rtmp_frame_count);
            } else {
                ret = child_push_frame_repeated(&ctx, child_fd, overlay, zone_overlay, duplicate_frames);
            }
        } else {
            ret = child_push_frame_repeated(&ctx, child_fd, overlay, zone_overlay, duplicate_frames);
        }

        if (ret < 0) {
            if (ctx.mode == CHILD_OUTPUT_RTMP && ctx.video_store_ready) {
                fprintf(stderr, "[Child] RTMP push failed, switch to local file mode.\n");
                if (child_switch_to_file_mode(&ctx, frame_mono_ms, frame_wall_ms) == 0) {
                    ret = child_push_frame_repeated(&ctx, child_fd, overlay, zone_overlay, duplicate_frames);
                    if (ret == 0) {
                        char ack = 'k';
                        if (write(sock, &ack, 1) <= 0) {
                            close(child_fd);
                            break;
                        }
                        close(child_fd);
                        continue;
                    }
                }
            }

            close(child_fd);
            fprintf(stderr, "[Child] Push failed, exit child loop.\n");
            break;
        }

        char ack = 'k';
        if (write(sock, &ack, 1) <= 0) {
            close(child_fd);
            break;
        }

        close(child_fd);
    }

    if (ctx.record_streamer_ready) {
        child_close_record_segment(&ctx, 0);
    }

    if (ctx.store_ready) {
        local_store_close(&ctx.store);
    }

    /*
     * Do not call streamer_clean() here. avcodec_free_context(h264_rkmpp)
     * has repeatedly triggered MPP ref-count errors followed by kernel Oops
     * on this board. The child process is short-lived, so let process exit
     * reclaim userspace resources and avoid the buggy MPP teardown path.
     */
    return 0;
}

static int spawn_stream_child(StreamState *stream) {
    int sv[2];
    pid_t pid;

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) {
        perror("socketpair");
        return -1;
    }

    pid = fork();
    if (pid < 0) {
        perror("fork");
        close(sv[0]);
        close(sv[1]);
        return -1;
    }

    if (pid == 0) {
        close(sv[0]);
        g_video_uploader_for_signal = NULL;
        g_video_uploader_signal_ready = 0;
        int ret = child_stream_loop(sv[1], stream->detect_state, stream->zone_state);
        close(sv[1]);
        _exit(ret == 0 ? 0 : 1);
    }

    close(sv[1]);
    stream->parent_sock = sv[0];
    stream->child_pid = pid;
    stream->stream_online = 1;
    reset_restart_backoff(stream);
    stream->last_restart_mono_ms = mono_now_ms();

    printf("[Parent] Stream child started: pid=%d\n", pid);
    return 0;
}

static void try_restart_stream_child(StreamState *stream) {
    int status;
    int64_t now;
    pid_t ret;

    if (stream->stream_online) {
        return;
    }

    if (stream->child_pid > 0) {
        ret = waitpid(stream->child_pid, &status, WNOHANG);
        if (ret == 0) {
            return;
        }
        if (ret == stream->child_pid) {
            stream->child_pid = -1;
        }
    }

    now = mono_now_ms();
    if (now - stream->last_restart_mono_ms < stream->current_restart_ms) {
        return;
    }

    stream->last_restart_mono_ms = now;
    printf("[Parent] Try restart stream child after %d ms backoff...\n",
           stream->current_restart_ms);

    if (spawn_stream_child(stream) < 0) {
        increase_restart_backoff(stream);
        printf("[Parent] Restart stream child failed, fail_count=%d, next backoff=%d ms.\n",
               stream->restart_fail_count,
               stream->current_restart_ms);
    }
}

int main(void) {
    CameraCtx cam;
    StreamState stream = {
        .stream_online = 0,
        .child_pid = -1,
        .parent_sock = -1,
        .detect_state = NULL,
        .zone_state = NULL,
        .base_restart_ms = STREAM_RESTART_BASE_MS,
        .max_restart_ms = STREAM_RESTART_MAX_MS,
        .current_restart_ms = STREAM_RESTART_BASE_MS,
        .restart_fail_count = 0,
        .last_restart_mono_ms = 0,
    };
    int shmid = -1;
    void *shmaddr = (void *)-1;
    DetectSharedState *detect_state = NULL;
    ZoneOverlayState *zone_state = NULL;
    int dma_fds[BUF_COUNT] = { -1, -1, -1, -1 };
    VideoUploader video_uploader;
    AiPipeline ai_pipeline;
    SafetyInterlockClient safety_client;
    SafetyRuntimeState safety_runtime;
    int safety_client_started = 0;
    int video_uploader_started = 0;
    char video_uploader_reason[256];
    uint64_t capture_frames = 0;
    uint64_t stream_frames = 0;
    uint64_t stream_failures = 0;
    int64_t last_video_stats_ms = 0;
    int64_t last_sensor_lock_ms = 0;
    int last_actuator_report_valid = 0;
    uint16_t last_actuator_report_flags = 0;

    memset(&video_uploader, 0, sizeof(video_uploader));
    memset(&ai_pipeline, 0, sizeof(ai_pipeline));
    memset(&safety_client, 0, sizeof(safety_client));
    memset(&safety_runtime, 0, sizeof(safety_runtime));

    install_signal_handlers();

    {
        int rc = safety_client_init(&safety_client, NULL, 0);
        if (rc != 0) {
            fprintf(stderr, "[Parent] Failed to start STM32 safety client: %s\n",
                    strerror(rc));
        } else {
            safety_client_started = 1;
            printf("[Parent] STM32 safety client started\n");
        }
    }

    detect_state = mmap(NULL,
                        sizeof(*detect_state),
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS,
                        -1,
                        0);
    if (detect_state == MAP_FAILED) {
        perror("detect mmap");
        detect_state = NULL;
    } else {
        memset(detect_state, 0, sizeof(*detect_state));
        stream.detect_state = detect_state;
    }

    zone_state = mmap(NULL, sizeof(*zone_state), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (zone_state == MAP_FAILED) {
        perror("zone mmap");
        zone_state = NULL;
    } else {
        memset(zone_state, 0, sizeof(*zone_state));
        stream.zone_state = zone_state;
    }

    camera_sensor_lock_fps_controls();

    if (camera_init(&cam, VIDEO_DEV, WIDTH, HEIGHT) < 0) {
        fprintf(stderr, "Failed to init camera\n");
        exit(1);
    }

    /* 保留这段共享内存初始化，后面如果有别的进程要复用帧数据还能继续接�?*/
    shmid = shmget(IPC_PRIVATE, FRAME_SIZE, IPC_CREAT | 0666);
    if (shmid < 0) {
        perror("shmget");
        camera_deinit(&cam);
        exit(1);
    }

    shmaddr = shmat(shmid, NULL, 0);
    if (shmaddr == (void *)-1) {
        perror("shmat");
        camera_deinit(&cam);
        exit(1);
    }

    for (int i = 0; i < BUF_COUNT; i++) {
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = cam.buf_type;
        expbuf.index = i;

        if (ioctl(cam.fd, VIDIOC_EXPBUF, &expbuf) < 0) {
            perror("[Parent] VIDIOC_EXPBUF failed (Does your driver support DMA-BUF?)");
            goto cleanup;
        }
        dma_fds[i] = expbuf.fd;
    }

    if (camera_start(&cam) < 0) {
        perror("camera start failed");
        goto cleanup;
    }
    camera_sensor_lock_fps_controls();
    if (osd_cache_init("/root/MOD20.TTF", 32) != 0) {
        fprintf(stderr, "[Parent] OSD 字模初始化失败，视频将不显示文字！\n");
    }
    
    if (!env_to_bool_default(SENSOR_DEVICE_DISABLE_ENV, 1)) {
        const char *sensor_device = getenv(SENSOR_DEVICE_ENV);
        if (sensor_device == NULL || sensor_device[0] == '\0') {
            sensor_device = SENSOR_DEVICE_DEFAULT;
        }

        int rc = start_sensor_collector(sensor_device);
        if (rc != 0) {
            fprintf(stderr, "[Parent] Failed to start sensor collector on %s: %s\n",
                    sensor_device, strerror(rc));
        } else {
            printf("[Parent] Sensor collector started on %s\n", sensor_device);
        }
    } else {
        printf("[Parent] Sensor collector skipped: %s=1\n", SENSOR_DEVICE_DISABLE_ENV);
    }

    {
        int rc = spawn_stream_child(&stream);
        if (rc < 0) {
            printf("[Parent] Initial stream child start failed, continue in offline mode.\n");
        }
    }

    if (ai_pipeline_start(&ai_pipeline, detect_state) != 0) {
        printf("[Parent][AI] Continue without RKNN inference.\n");
    }
    audio_alert_init();//启动音频线程
    {
        if (safety_client_started) {
            set_mqtt_safety_client(&safety_client);
        }
        int rc = start_mqtt_reporter();
        if (rc != 0) {
            fprintf(stderr, "[Parent] Failed to start MQTT reporter: %s\n", strerror(rc));
        } else {
            printf("[Parent] MQTT reporter started\n");
        }
    }

    if (video_uploader_macro_ready(video_uploader_reason, sizeof(video_uploader_reason))) {
        int rc = video_uploader_start(&video_uploader,
                                      NULL,
                                      video_uploader_http_upload_callback,
                                      &video_uploader);
        if (rc != 0) {
            fprintf(stderr, "[Parent] Failed to start video uploader: %s\n", strerror(rc));
        } else {
            video_uploader_started = 1;
            g_video_uploader_for_signal = &video_uploader;
            g_video_uploader_signal_ready = 1;
            printf("[Parent] Video uploader started\n");
        }
    } else {
        printf("[Parent] Video uploader skipped: %s\n", video_uploader_reason);
    }

    while (is_running) {
        struct v4l2_buffer qbuf;
        struct v4l2_plane qplanes[1];
        int current_dma_fd;
        int64_t frame_wall_ms;
        int64_t frame_mono_ms;

        memset(&qbuf, 0, sizeof(qbuf));
        memset(qplanes, 0, sizeof(qplanes));
        qbuf.type = cam.buf_type;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.m.planes = qplanes;
        qbuf.length = 1;

        if (ioctl(cam.fd, VIDIOC_DQBUF, &qbuf) < 0) {
            if (errno == EINTR) {
                break;
            }
            perror("camera dequeue buf");
            break;
        }

        current_dma_fd = dma_fds[qbuf.index];
        frame_wall_ms = wall_now_ms();
        frame_mono_ms = mono_now_ms();
        capture_frames++;
        zone_runtime_scan_frame(&cam, qbuf.index, frame_mono_ms, zone_state);

        if (stream.stream_online) {
            if (send_fd(stream.parent_sock, current_dma_fd) < 0) {
                stream_failures++;
                fprintf(stderr, "[Parent] send_fd failed, mark stream offline.\n");
                mark_stream_offline(&stream);
            } else {
                char ack;
                if (read(stream.parent_sock, &ack, 1) <= 0) {
                    stream_failures++;
                    fprintf(stderr, "[Parent] ACK failed, mark stream offline.\n");
                    mark_stream_offline(&stream);
                } else {
                    stream_frames++;
                }
            }
        }

        if (last_sensor_lock_ms == 0 || frame_mono_ms - last_sensor_lock_ms >= CAMERA_SENSOR_LOCK_INTERVAL_MS) {
            camera_sensor_lock_fps_controls();
            last_sensor_lock_ms = frame_mono_ms;
        }

        ai_pipeline_process_frame(&ai_pipeline, current_dma_fd, frame_wall_ms);
        ai_pipeline_log_stats(&ai_pipeline, frame_mono_ms);
        if (safety_client_started) {
            safety_runtime_update(&safety_client,
                                  &safety_runtime,
                                  detect_state,
                                  frame_wall_ms,
                                  frame_mono_ms);
            SafetyStm32Snapshot stm32_snapshot;
            if (safety_client_get_snapshot(&safety_client, &stm32_snapshot) == 0 &&
                stm32_snapshot.actuator_feedback_valid) {
                uint16_t actuator_flags =
                    (uint16_t)(stm32_snapshot.actuator_flags &
                               (SAFETY_ACT_DEVICE_POWER_ON |
                                SAFETY_ACT_FAN_ON |
                                SAFETY_ACT_ALARM_ON));
                if (!last_actuator_report_valid ||
                    actuator_flags != last_actuator_report_flags) {
                    last_actuator_report_valid = 1;
                    last_actuator_report_flags = actuator_flags;
                    mqtt_request_immediate_report();
                }
            }
        }

        if (last_video_stats_ms == 0) {
            last_video_stats_ms = frame_mono_ms;
        } else if (frame_mono_ms - last_video_stats_ms >= VIDEO_STATS_INTERVAL_MS) {
            double sec = (double)(frame_mono_ms - last_video_stats_ms) / 1000.0;
            printf("[Parent][Video] capture_fps=%.1f stream_fps=%.1f stream_failures=%llu\n",
                   capture_frames / sec,
                   stream_frames / sec,
                   (unsigned long long)stream_failures);
            capture_frames = 0;
            stream_frames = 0;
            stream_failures = 0;
            last_video_stats_ms = frame_mono_ms;
        }

        /* 推流失败不应该拖死采集主循环，这一帧必须回给驱动�?*/
        if (ioctl(cam.fd, VIDIOC_QBUF, &qbuf) < 0) {
            perror("[Parent] camera queue buf");
            break;
        }

        if (!stream.stream_online) {
            try_restart_stream_child(&stream);
        }
    }

cleanup:
    is_running = 0;
    if (safety_client_started) {
        safety_client_stop(&safety_client);
    }
    //音频清理
    audio_alert_deinit();
    if (video_uploader_started) {
        video_uploader_request_stop(&video_uploader);
    }
    g_video_uploader_signal_ready = 0;
    ai_pipeline_stop(&ai_pipeline);
    stop_mqtt_reporter();
    stop_stream_child(&stream);
    
    //字清�?
    osd_cache_deinit();
    
    
    if (video_uploader_started) {
        video_uploader_stop(&video_uploader);
    }
    g_video_uploader_for_signal = NULL;

    camera_stop(&cam);
    camera_deinit(&cam);

    for (int i = 0; i < BUF_COUNT; i++) {
        if (dma_fds[i] >= 0) {
            close(dma_fds[i]);
        }
    }

    if (shmaddr && shmaddr != (void *)-1) {
        shmdt(shmaddr);
    }
    if (shmid >= 0) {
        if (shmctl(shmid, IPC_RMID, NULL) < 0) {
            perror("shmctl IPC_RMID failed");
        }
    }
    if (detect_state != NULL) {
        munmap(detect_state, sizeof(*detect_state));
    }
    if (zone_state != NULL) {
        munmap(zone_state, sizeof(*zone_state));
    }

    return 0;
}
