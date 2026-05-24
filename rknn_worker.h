#ifndef RKNN_WORKER_H
#define RKNN_WORKER_H

#include <stdint.h>
#include "detect_shared.h"

typedef struct {
    int fd;             // NPU 内存的 DMA FD
    void* virt_addr;    // NPU 内存的虚拟地址 (备用)
    int width;
    int height;
    int channels;
    int size;
    int slot; 
} RknnWorkerInputBuffer;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t size;
} RknnWorkerInputInfo;

typedef struct {
    const char *model_path;
    int camera_width;
    int camera_height;
    DetectSharedState *detect_state;
    float conf_threshold;
    float nms_threshold;
} RknnWorkerConfig;

typedef struct {
    uint64_t submitted;
    uint64_t processed;
    uint64_t dropped;
    uint64_t failed;
    uint64_t decoded_candidates;
    int64_t last_infer_us;
} RknnWorkerStats;

typedef struct {
    void* internal_context; 
} RknnWorker;

#ifdef __cplusplus
extern "C" {
#endif

void rknn_worker_config_defaults(RknnWorkerConfig *config);
int rknn_worker_start(RknnWorker *worker, RknnWorkerConfig *config);
void rknn_worker_stop(RknnWorker *worker);
int rknn_worker_is_ready(RknnWorker *worker);
int rknn_worker_get_input_info(RknnWorker *worker, RknnWorkerInputInfo *info);
int rknn_worker_acquire_input_buffer(RknnWorker *worker, RknnWorkerInputBuffer *buffer);
int rknn_worker_submit_input_buffer(RknnWorker *worker, int slot, int64_t frame_seq, int64_t frame_wall_ms);
void rknn_worker_get_stats(RknnWorker *worker, RknnWorkerStats *stats);
void rknn_worker_release_input_buffer(RknnWorker *worker, int slot);
char *coco_cls_to_name(int cls_id);

#ifdef __cplusplus
}
#endif

#endif