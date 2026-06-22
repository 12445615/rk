#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <errno.h>

#ifndef DMA_BUF_IOCTL_SYNC
struct dma_buf_sync {
    __u64 flags;
};
#define DMA_BUF_SYNC_READ      (1 << 0)
#define DMA_BUF_SYNC_WRITE     (2 << 0)
#define DMA_BUF_SYNC_RW        (DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE)
#define DMA_BUF_SYNC_START     (0 << 2)
#define DMA_BUF_SYNC_END       (1 << 2)
#define DMA_BUF_BASE           'b'
#define DMA_BUF_IOCTL_SYNC     _IOW(DMA_BUF_BASE, 0, struct dma_buf_sync)
#endif

#include "rknn_api.h"
#include "rknn_worker.h"
#include "postprocess.h"
#include "Float16.h"

#define MAX_SLOTS 4
#define MAX_PENDING_JOBS 2

typedef struct {
    int slot;
    int64_t frame_seq;
    int64_t frame_wall_ms;
} RknnJob;

static rknn_tensor_mem* slot_mems[MAX_SLOTS];
static int slot_in_use[MAX_SLOTS];
static rknn_context ctx;
static rknn_input_output_num io_num;
static rknn_tensor_attr* input_attrs = NULL;
static rknn_tensor_attr* output_attrs = NULL;
static RknnWorkerStats g_stats;
static DetectSharedState* g_shared_state = NULL;
static float g_conf_threshold = 0.5f;
static float g_nms_threshold = 0.45f;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cond = PTHREAD_COND_INITIALIZER;
static pthread_t g_thread;
static int g_thread_started = 0;
static int g_stop = 0;
static RknnJob g_jobs[MAX_PENDING_JOBS];
static int g_job_head = 0;
static int g_job_tail = 0;
static int g_job_count = 0;

static int64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static uint32_t input_rgb_u8_size(void) {
    if (input_attrs == NULL || input_attrs[0].n_dims < 4) {
        return 0;
    }
    return (uint32_t)(input_attrs[0].dims[1] *
                      input_attrs[0].dims[2] *
                      input_attrs[0].dims[3]);
}

static int input_is_fp16(void) {
    return input_attrs != NULL && input_attrs[0].type == RKNN_TENSOR_FLOAT16;
}

static void convert_rgb_u8_to_fp16_inplace(void *buffer, uint32_t rgb_size) {
    uint8_t *rgb = (uint8_t *)buffer;
    rknpu2::float16 *fp16 = (rknpu2::float16 *)buffer;

    for (int64_t i = (int64_t)rgb_size - 1; i >= 0; --i) {
        fp16[i] = (float)rgb[i] / 255.0f;
    }
}

static void stats_inc(uint64_t *field) {
    pthread_mutex_lock(&g_lock);
    (*field)++;
    pthread_mutex_unlock(&g_lock);
}

static void release_slot_locked(int slot) {
    if (slot >= 0 && slot < MAX_SLOTS) {
        slot_in_use[slot] = 0;
    }
}

static int pop_job(RknnJob *job) {
    pthread_mutex_lock(&g_lock);
    while (!g_stop && g_job_count == 0) {
        pthread_cond_wait(&g_cond, &g_lock);
    }
    if (g_stop && g_job_count == 0) {
        pthread_mutex_unlock(&g_lock);
        return ECANCELED;
    }
    *job = g_jobs[g_job_head];
    g_job_head = (g_job_head + 1) % MAX_PENDING_JOBS;
    g_job_count--;
    pthread_mutex_unlock(&g_lock);
    return 0;
}

static void write_detect_results(const object_detect_result_list *od_results,
                                 int64_t frame_seq,
                                 int64_t frame_wall_ms) {
    if (g_shared_state == NULL || od_results == NULL) {
        return;
    }

    __sync_fetch_and_add(&g_shared_state->version, 1);
    g_shared_state->frame_seq = frame_seq;
    g_shared_state->timestamp_ms = frame_wall_ms;
    g_shared_state->box_count = (od_results->count > DETECT_MAX_BOXES) ? DETECT_MAX_BOXES : od_results->count;

    for (int i = 0; i < g_shared_state->box_count; i++) {
        g_shared_state->boxes[i].x1 = (float)od_results->results[i].box.left;
        g_shared_state->boxes[i].y1 = (float)od_results->results[i].box.top;
        g_shared_state->boxes[i].x2 = (float)od_results->results[i].box.right;
        g_shared_state->boxes[i].y2 = (float)od_results->results[i].box.bottom;
        g_shared_state->boxes[i].score = od_results->results[i].prop;
        g_shared_state->boxes[i].class_id = od_results->results[i].cls_id;
    }
    g_shared_state->valid = 1;
    if (g_shared_state->box_count > 0) {
        static int64_t last_write_log_ms = 0;
        if (frame_wall_ms - last_write_log_ms >= 1000) {
            printf("[AI][SharedWrite] boxes=%d frame=%lld ts=%lld #0 cls=%d score=%.3f xy=(%.1f,%.1f)-(%.1f,%.1f)\n",
                   g_shared_state->box_count,
                   (long long)frame_seq,
                   (long long)frame_wall_ms,
                   g_shared_state->boxes[0].class_id,
                   g_shared_state->boxes[0].score,
                   g_shared_state->boxes[0].x1,
                   g_shared_state->boxes[0].y1,
                   g_shared_state->boxes[0].x2,
                   g_shared_state->boxes[0].y2);
            last_write_log_ms = frame_wall_ms;
        }
    }
    __sync_fetch_and_add(&g_shared_state->version, 1);
}

static void process_job(const RknnJob *job) {
    int64_t start_us = now_us();
    int ret;
    rknn_output *outputs = NULL;
    object_detect_result_list od_results;
    struct dma_buf_sync sync = {0};

    if (job->slot < 0 || job->slot >= MAX_SLOTS || slot_mems[job->slot] == NULL) {
        stats_inc(&g_stats.failed);
        return;
    }

    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    ioctl(slot_mems[job->slot]->fd, DMA_BUF_IOCTL_SYNC, &sync);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = slot_mems[job->slot]->virt_addr;
    if (input_rgb_u8_size() == 0) {
        stats_inc(&g_stats.failed);
        goto out;
    }
    if (input_is_fp16()) {
        convert_rgb_u8_to_fp16_inplace(slot_mems[job->slot]->virt_addr, input_rgb_u8_size());
        inputs[0].type = RKNN_TENSOR_FLOAT16;
        inputs[0].size = input_attrs[0].size;
        inputs[0].pass_through = 1;
    } else {
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = input_rgb_u8_size();
        inputs[0].pass_through = 0;
    }

    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    ioctl(slot_mems[job->slot]->fd, DMA_BUF_IOCTL_SYNC, &sync);

    ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
    if (ret < 0) {
        fprintf(stderr, "[AI] rknn_inputs_set failed ret=%d\n", ret);
        stats_inc(&g_stats.failed);
        goto out;
    }

    ret = rknn_run(ctx, NULL);
    if (ret < 0) {
        fprintf(stderr, "[AI] rknn_run failed ret=%d\n", ret);
        stats_inc(&g_stats.failed);
        goto out;
    }

    outputs = (rknn_output*)calloc(io_num.n_output, sizeof(rknn_output));
    if (outputs == NULL) {
        stats_inc(&g_stats.failed);
        goto out;
    }
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        outputs[i].want_float = 1;
    }

    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
    if (ret < 0) {
        fprintf(stderr, "[AI] rknn_outputs_get failed ret=%d\n", ret);
        stats_inc(&g_stats.failed);
        goto out;
    }

    post_process(outputs, &output_attrs[0], io_num.n_output,
                 g_conf_threshold, g_nms_threshold, &od_results);
    rknn_outputs_release(ctx, io_num.n_output, outputs);

    write_detect_results(&od_results, job->frame_seq, job->frame_wall_ms);

    pthread_mutex_lock(&g_lock);
    g_stats.processed++;
    g_stats.decoded_candidates = od_results.count;
    g_stats.last_infer_us = now_us() - start_us;
    pthread_mutex_unlock(&g_lock);

out:
    if (outputs != NULL) {
        free(outputs);
    }
    pthread_mutex_lock(&g_lock);
    release_slot_locked(job->slot);
    pthread_mutex_unlock(&g_lock);
}

static void *infer_thread_main(void *arg) {
    (void)arg;
    while (1) {
        RknnJob job;
        if (pop_job(&job) != 0) {
            break;
        }
        process_job(&job);
    }
    return NULL;
}

extern "C" void rknn_worker_config_defaults(RknnWorkerConfig *config) {
    memset(config, 0, sizeof(*config));
    config->model_path = "/root/model/best_fp16.rknn";
    config->conf_threshold = 0.50f;
    config->nms_threshold = 0.45f;
}

extern "C" int rknn_worker_start(RknnWorker *worker, RknnWorkerConfig *config) {
    int ret;
    FILE* fp;
    int size;
    unsigned char* data;

    (void)worker;
    memset(&g_stats, 0, sizeof(g_stats));
    memset(slot_in_use, 0, sizeof(slot_in_use));
    memset(slot_mems, 0, sizeof(slot_mems));
    g_job_head = g_job_tail = g_job_count = 0;
    g_stop = 0;
    g_shared_state = config->detect_state;
    g_conf_threshold = config->conf_threshold;
    g_nms_threshold = config->nms_threshold;

    fp = fopen(config->model_path, "rb");
    if (!fp) {
        fprintf(stderr, "[AI] model load failed: %s\n", config->model_path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    data = (unsigned char*)malloc(size);
    if (data == NULL) {
        fclose(fp);
        return -1;
    }
    fseek(fp, 0, SEEK_SET);
    fread(data, 1, size, fp);
    fclose(fp);

    ret = rknn_init(&ctx, data, size, 0, NULL);
    free(data);
    if (ret < 0) {
        fprintf(stderr, "[AI] rknn_init failed ret=%d\n", ret);
        return -1;
    }

    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    input_attrs = (rknn_tensor_attr*)calloc(io_num.n_input, sizeof(rknn_tensor_attr));
    output_attrs = (rknn_tensor_attr*)calloc(io_num.n_output, sizeof(rknn_tensor_attr));
    if (input_attrs == NULL || output_attrs == NULL) {
        return -1;
    }

    for (uint32_t i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    }
    for (int s = 0; s < MAX_SLOTS; s++) {
        slot_mems[s] = rknn_create_mem(ctx, input_attrs[0].size);
        if (slot_mems[s] == NULL) {
            return -1;
        }
    }

    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        printf("[AI] output[%u] n_dims=%u dims=[%d,%d,%d,%d] n_elems=%u size=%u fmt=%d type=%d qnt=%d\n",
               i,
               output_attrs[i].n_dims,
               output_attrs[i].dims[0],
               output_attrs[i].dims[1],
               output_attrs[i].dims[2],
               output_attrs[i].dims[3],
               output_attrs[i].n_elems,
               output_attrs[i].size,
               output_attrs[i].fmt,
               output_attrs[i].type,
               output_attrs[i].qnt_type);
    }

    ret = pthread_create(&g_thread, NULL, infer_thread_main, NULL);
    if (ret != 0) {
        fprintf(stderr, "[AI] pthread_create failed: %s\n", strerror(ret));
        return -1;
    }
    g_thread_started = 1;

    printf("[AI] RKNN async worker started. slots=%d pending=%d tensor_size=%u rgb_u8_size=%u conf=%.2f nms=%.2f\n",
           MAX_SLOTS, MAX_PENDING_JOBS, input_attrs[0].size, input_rgb_u8_size(), g_conf_threshold, g_nms_threshold);
    return 0;
}

extern "C" int rknn_worker_submit_input_buffer(RknnWorker *worker, int slot, int64_t frame_seq, int64_t frame_wall_ms) {
    (void)worker;
    if (slot < 0 || slot >= MAX_SLOTS) {
        return EINVAL;
    }

    pthread_mutex_lock(&g_lock);
    if (g_stop) {
        release_slot_locked(slot);
        pthread_mutex_unlock(&g_lock);
        return ECANCELED;
    }
    if (g_job_count >= MAX_PENDING_JOBS) {
        release_slot_locked(slot);
        g_stats.dropped++;
        pthread_mutex_unlock(&g_lock);
        return EBUSY;
    }

    g_jobs[g_job_tail].slot = slot;
    g_jobs[g_job_tail].frame_seq = frame_seq;
    g_jobs[g_job_tail].frame_wall_ms = frame_wall_ms;
    g_job_tail = (g_job_tail + 1) % MAX_PENDING_JOBS;
    g_job_count++;
    g_stats.submitted++;
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

extern "C" int rknn_worker_acquire_input_buffer(RknnWorker *worker, RknnWorkerInputBuffer *buffer) {
    (void)worker;
    if (buffer == NULL || input_attrs == NULL) {
        return EINVAL;
    }

    pthread_mutex_lock(&g_lock);
    if (g_stop || g_job_count >= MAX_PENDING_JOBS) {
        pthread_mutex_unlock(&g_lock);
        return EBUSY;
    }
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (!slot_in_use[i]) {
            slot_in_use[i] = 1;
            buffer->slot = i;
            buffer->fd = slot_mems[i]->fd;
            buffer->virt_addr = slot_mems[i]->virt_addr;
            buffer->width = input_attrs[0].dims[2];
            buffer->height = input_attrs[0].dims[1];
            buffer->channels = input_attrs[0].dims[3];
            buffer->size = input_rgb_u8_size();
            pthread_mutex_unlock(&g_lock);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_lock);
    return EBUSY;
}

extern "C" void rknn_worker_release_input_buffer(RknnWorker *worker, int slot) {
    (void)worker;
    pthread_mutex_lock(&g_lock);
    release_slot_locked(slot);
    pthread_mutex_unlock(&g_lock);
}

extern "C" void rknn_worker_stop(RknnWorker *worker) {
    (void)worker;
    pthread_mutex_lock(&g_lock);
    g_stop = 1;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (g_thread_started) {
        pthread_join(g_thread, NULL);
        g_thread_started = 0;
    }

    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slot_mems[i]) {
            rknn_destroy_mem(ctx, slot_mems[i]);
            slot_mems[i] = NULL;
        }
    }
    if (ctx) {
        rknn_destroy(ctx);
        ctx = 0;
    }
    free(input_attrs);
    free(output_attrs);
    input_attrs = NULL;
    output_attrs = NULL;
}

extern "C" void rknn_worker_get_stats(RknnWorker *worker, RknnWorkerStats *stats) {
    (void)worker;
    if (stats == NULL) {
        return;
    }
    pthread_mutex_lock(&g_lock);
    memcpy(stats, &g_stats, sizeof(g_stats));
    pthread_mutex_unlock(&g_lock);
}

extern "C" int rknn_worker_get_input_info(RknnWorker *worker, RknnWorkerInputInfo *info) {
    (void)worker;
    if (info == NULL || input_attrs == NULL) {
        return EINVAL;
    }
    info->width = input_attrs[0].dims[2];
    info->height = input_attrs[0].dims[1];
    info->channels = input_attrs[0].dims[3];
    info->size = input_rgb_u8_size();
    return 0;
}

extern "C" int rknn_worker_is_ready(RknnWorker *worker) {
    (void)worker;
    return input_attrs != NULL && !g_stop;
}
