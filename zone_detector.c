#include "zone_detector.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aliyun_mqtt.h"

#define ZONE_COLOR_SCAN_INTERVAL_MS 200
#define ZONE_COLOR_MIN_PIXELS 120
#define ZONE_COLOR_STEP 2
#define ZONE_EDGE_MIN_SPAN 100
#define ZONE_EDGE_BAND_PX 12
#define ZONE_EDGE_MIN_RATIO 0.30f
#define WIDTH 1280
#define HEIGHT 720

typedef struct {
    ZoneRect work_zone;
    ZoneRect danger_zone;
    int detected;
    int reset_pending;
    int64_t last_scan_ms;
} ColorZoneRuntime;

static ColorZoneRuntime g_color_zone_runtime;
static pthread_mutex_t g_color_zone_lock = PTHREAD_MUTEX_INITIALIZER;

static int zone_rect_valid_size(const ZoneRect *rect) {
    if (rect == NULL || !rect->valid) return 0;
    return (rect->x2 - rect->x1) >= ZONE_EDGE_MIN_SPAN &&
           (rect->y2 - rect->y1) >= ZONE_EDGE_MIN_SPAN;
}

int zone_runtime_get_detected(ZoneRect *work_zone,
                                     ZoneRect *danger_zone) {
    int detected;

    pthread_mutex_lock(&g_color_zone_lock);
    detected = g_color_zone_runtime.detected;
    if (detected) {
        if (work_zone != NULL) {
            *work_zone = g_color_zone_runtime.work_zone;
        }
        if (danger_zone != NULL) {
            *danger_zone = g_color_zone_runtime.danger_zone;
        }
    }
    pthread_mutex_unlock(&g_color_zone_lock);

    return detected;
}

void zone_runtime_reset(void) {
    pthread_mutex_lock(&g_color_zone_lock);
    memset(&g_color_zone_runtime.work_zone, 0, sizeof(g_color_zone_runtime.work_zone));
    memset(&g_color_zone_runtime.danger_zone, 0, sizeof(g_color_zone_runtime.danger_zone));
    g_color_zone_runtime.detected = 0;
    g_color_zone_runtime.reset_pending = 1;
    g_color_zone_runtime.last_scan_ms = 0;
    pthread_mutex_unlock(&g_color_zone_lock);
    mqtt_update_zone_detection_result(0);
    printf("[ZoneDetect] reset, waiting for fresh blue/red zone detection\n");
}

static void zone_runtime_sync_shared(DetectSharedState *shared,
                                     const ZoneRect *work_zone,
                                     const ZoneRect *danger_zone,
                                     int detected) {
    if (shared == NULL) {
        return;
    }

    __sync_fetch_and_add(&shared->version, 1);
    if (detected && work_zone != NULL && danger_zone != NULL) {
        shared->work_zone.valid = work_zone->valid;
        shared->work_zone.x1 = work_zone->x1;
        shared->work_zone.y1 = work_zone->y1;
        shared->work_zone.x2 = work_zone->x2;
        shared->work_zone.y2 = work_zone->y2;
        shared->danger_zone.valid = danger_zone->valid;
        shared->danger_zone.x1 = danger_zone->x1;
        shared->danger_zone.y1 = danger_zone->y1;
        shared->danger_zone.x2 = danger_zone->x2;
        shared->danger_zone.y2 = danger_zone->y2;
        __sync_synchronize();
        shared->zone_valid = 1;
    } else {
        shared->zone_valid = 0;
        __sync_synchronize();
        memset(&shared->work_zone, 0, sizeof(shared->work_zone));
        memset(&shared->danger_zone, 0, sizeof(shared->danger_zone));
    }
    __sync_synchronize();
    __sync_fetch_and_add(&shared->version, 1);
    __sync_synchronize();
}

static void zone_update_candidate(ZoneRect *rect, int *count, int x, int y) {
    if (*count == 0) {
        rect->x1 = rect->x2 = (float)x;
        rect->y1 = rect->y2 = (float)y;
    } else {
        if ((float)x < rect->x1) rect->x1 = (float)x;
        if ((float)x > rect->x2) rect->x2 = (float)x;
        if ((float)y < rect->y1) rect->y1 = (float)y;
        if ((float)y > rect->y2) rect->y2 = (float)y;
    }
    (*count)++;
}

static int zone_rect_contains_center(const ZoneRect *outer, const ZoneRect *inner) {
    float cx;
    float cy;

    if (outer == NULL || inner == NULL || !outer->valid || !inner->valid) {
        return 0;
    }

    cx = (inner->x1 + inner->x2) * 0.5f;
    cy = (inner->y1 + inner->y2) * 0.5f;
    return cx >= outer->x1 && cx <= outer->x2 &&
           cy >= outer->y1 && cy <= outer->y2;
}

static int zone_color_rect_has_frame_edges(const ZoneRect *rect,
                                           const uint8_t *mask,
                                           int stride) {
    int left_hits = 0;
    int right_hits = 0;
    int top_hits = 0;
    int bottom_hits = 0;
    int x1;
    int y1;
    int x2;
    int y2;
    int vertical_required;
    int horizontal_required;
    int passed_edges = 0;

    if (!zone_rect_valid_size(rect) || mask == NULL || stride <= 0) {
        return 0;
    }

    x1 = (int)rect->x1;
    y1 = (int)rect->y1;
    x2 = (int)rect->x2;
    y2 = (int)rect->y2;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= WIDTH) x2 = WIDTH - 1;
    if (y2 >= HEIGHT) y2 = HEIGHT - 1;

    for (int y = y1; y <= y2; y += ZONE_COLOR_STEP) {
        int left_found = 0;
        int right_found = 0;
        for (int dx = 0; dx <= ZONE_EDGE_BAND_PX; dx += ZONE_COLOR_STEP) {
            if (x1 + dx < WIDTH && mask[y * stride + x1 + dx]) {
                left_found = 1;
            }
            if (x2 - dx >= 0 && mask[y * stride + x2 - dx]) {
                right_found = 1;
            }
        }
        left_hits += left_found;
        right_hits += right_found;
    }

    for (int x = x1; x <= x2; x += ZONE_COLOR_STEP) {
        int top_found = 0;
        int bottom_found = 0;
        for (int dy = 0; dy <= ZONE_EDGE_BAND_PX; dy += ZONE_COLOR_STEP) {
            if (y1 + dy < HEIGHT && mask[(y1 + dy) * stride + x]) {
                top_found = 1;
            }
            if (y2 - dy >= 0 && mask[(y2 - dy) * stride + x]) {
                bottom_found = 1;
            }
        }
        top_hits += top_found;
        bottom_hits += bottom_found;
    }

    vertical_required = (int)(((y2 - y1) / ZONE_COLOR_STEP + 1) * ZONE_EDGE_MIN_RATIO);
    horizontal_required = (int)(((x2 - x1) / ZONE_COLOR_STEP + 1) * ZONE_EDGE_MIN_RATIO);
    if (vertical_required < 8) vertical_required = 8;
    if (horizontal_required < 8) horizontal_required = 8;

    if (left_hits >= vertical_required) passed_edges++;
    if (right_hits >= vertical_required) passed_edges++;
    if (top_hits >= horizontal_required) passed_edges++;
    if (bottom_hits >= horizontal_required) passed_edges++;

    return passed_edges >= 3 &&
           (left_hits >= vertical_required || right_hits >= vertical_required) &&
           (top_hits >= horizontal_required || bottom_hits >= horizontal_required);
}

static int zone_find_best_color_component(const uint8_t *mask,
                                          ZoneRect *best_rect) {
    uint8_t *visited = NULL;
    int *queue = NULL;
    int best_count = 0;

    if (mask == NULL || best_rect == NULL) {
        return 0;
    }

    memset(best_rect, 0, sizeof(*best_rect));
    visited = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    queue = (int *)malloc((size_t)WIDTH * HEIGHT * sizeof(int));
    if (visited == NULL || queue == NULL) {
        free(visited);
        free(queue);
        return 0;
    }

    for (int y = 0; y < HEIGHT; y += ZONE_COLOR_STEP) {
        for (int x = 0; x < WIDTH; x += ZONE_COLOR_STEP) {
            int start = y * WIDTH + x;
            int head = 0;
            int tail = 0;
            int count = 0;
            ZoneRect rect;

            if (!mask[start] || visited[start]) {
                continue;
            }

            memset(&rect, 0, sizeof(rect));
            visited[start] = 1;
            queue[tail++] = start;

            while (head < tail) {
                int idx = queue[head++];
                int cx = idx % WIDTH;
                int cy = idx / WIDTH;
                const int nx[4] = {
                    cx - ZONE_COLOR_STEP,
                    cx + ZONE_COLOR_STEP,
                    cx,
                    cx
                };
                const int ny[4] = {
                    cy,
                    cy,
                    cy - ZONE_COLOR_STEP,
                    cy + ZONE_COLOR_STEP
                };

                zone_update_candidate(&rect, &count, cx, cy);

                for (int i = 0; i < 4; i++) {
                    int px = nx[i];
                    int py = ny[i];
                    int next;
                    if (px < 0 || px >= WIDTH || py < 0 || py >= HEIGHT) {
                        continue;
                    }
                    next = py * WIDTH + px;
                    if (!mask[next] || visited[next]) {
                        continue;
                    }
                    visited[next] = 1;
                    queue[tail++] = next;
                }
            }

            rect.valid = count >= ZONE_COLOR_MIN_PIXELS;
            rect.x2 += ZONE_COLOR_STEP;
            rect.y2 += ZONE_COLOR_STEP;
            if (!zone_rect_valid_size(&rect)) {
                rect.valid = 0;
            }
            if (rect.valid && !zone_color_rect_has_frame_edges(&rect, mask, WIDTH)) {
                rect.valid = 0;
            }
            if (rect.valid && count > best_count) {
                best_count = count;
                *best_rect = rect;
            }
        }
    }

    free(visited);
    free(queue);
    return best_count >= ZONE_COLOR_MIN_PIXELS;
}

static int zone_detect_color_rects_nv12(const uint8_t *nv12,
                                        ZoneRect *blue_rect,
                                        ZoneRect *red_rect) {
    const uint8_t *y_plane;
    const uint8_t *uv_plane;
    uint8_t *blue_mask = NULL;
    uint8_t *red_mask = NULL;
    int ok = 0;

    if (nv12 == NULL || blue_rect == NULL || red_rect == NULL) {
        return 0;
    }

    memset(blue_rect, 0, sizeof(*blue_rect));
    memset(red_rect, 0, sizeof(*red_rect));
    blue_mask = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    red_mask = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    if (blue_mask == NULL || red_mask == NULL) {
        free(blue_mask);
        free(red_mask);
        return 0;
    }

    y_plane = nv12;
    uv_plane = nv12 + WIDTH * HEIGHT;

    for (int y = 0; y < HEIGHT; y += ZONE_COLOR_STEP) {
        const uint8_t *y_row = y_plane + y * WIDTH;
        const uint8_t *uv_row = uv_plane + (y / 2) * WIDTH;
        for (int x = 0; x < WIDTH; x += ZONE_COLOR_STEP) {
            int yy = y_row[x];
            int uv_index = (x & ~1);
            int u = uv_row[uv_index] - 128;
            int v = uv_row[uv_index + 1] - 128;
            int r = yy + ((359 * v) >> 8);
            int g = yy - ((88 * u + 183 * v) >> 8);
            int b = yy + ((454 * u) >> 8);

            if (r < 0) r = 0; else if (r > 255) r = 255;
            if (g < 0) g = 0; else if (g > 255) g = 255;
            if (b < 0) b = 0; else if (b > 255) b = 255;

            if (b > 105 && b > r + 35 && b > g + 20) {
                blue_mask[y * WIDTH + x] = 1;
            } else if (r > 115 && r > g + 35 && r > b + 25) {
                red_mask[y * WIDTH + x] = 1;
            }
        }
    }

    blue_rect->valid = zone_find_best_color_component(blue_mask, blue_rect);
    red_rect->valid = zone_find_best_color_component(red_mask, red_rect);
    ok = blue_rect->valid && red_rect->valid &&
         zone_rect_contains_center(blue_rect, red_rect);
    free(blue_mask);
    free(red_mask);

    return ok;
}

void zone_runtime_scan_frame(const CameraCtx *cam,
                                    int buffer_index,
                                    int64_t frame_mono_ms,
                                    DetectSharedState *shared) {
    ZoneRect blue_rect;
    ZoneRect red_rect;
    ZoneRect locked_work_zone;
    ZoneRect locked_danger_zone;
    int ok;
    int already_detected;
    int reset_pending;

    if (cam == NULL || buffer_index < 0 || buffer_index >= BUF_COUNT) {
        return;
    }

    pthread_mutex_lock(&g_color_zone_lock);
    already_detected = g_color_zone_runtime.detected;
    reset_pending = g_color_zone_runtime.reset_pending;
    g_color_zone_runtime.reset_pending = 0;
    locked_work_zone = g_color_zone_runtime.work_zone;
    locked_danger_zone = g_color_zone_runtime.danger_zone;
    pthread_mutex_unlock(&g_color_zone_lock);
    if (reset_pending) {
        zone_runtime_sync_shared(shared, NULL, NULL, 0);
    }
    if (already_detected) {
        zone_runtime_sync_shared(shared, &locked_work_zone, &locked_danger_zone, 1);
        mqtt_update_zone_detection_result(1);
        return;
    }

    if (g_color_zone_runtime.last_scan_ms != 0 &&
        frame_mono_ms - g_color_zone_runtime.last_scan_ms < ZONE_COLOR_SCAN_INTERVAL_MS) {
        return;
    }
    g_color_zone_runtime.last_scan_ms = frame_mono_ms;

    ok = zone_detect_color_rects_nv12((const uint8_t *)cam->buffers[buffer_index].start,
                                      &blue_rect,
                                      &red_rect);
    pthread_mutex_lock(&g_color_zone_lock);
    if (ok) {
        g_color_zone_runtime.detected = 1;
        g_color_zone_runtime.work_zone = blue_rect;
        g_color_zone_runtime.danger_zone = red_rect;
    }
    pthread_mutex_unlock(&g_color_zone_lock);
    zone_runtime_sync_shared(shared,
                             ok ? &blue_rect : NULL,
                             ok ? &red_rect : NULL,
                             ok);
    mqtt_update_zone_detection_result(ok);

    if (ok) {
        printf("[ZoneDetect] locked blue work=[%.0f,%.0f,%.0f,%.0f] red danger=[%.0f,%.0f,%.0f,%.0f]\n",
               blue_rect.x1, blue_rect.y1, blue_rect.x2, blue_rect.y2,
               red_rect.x1, red_rect.y1, red_rect.x2, red_rect.y2);
    } else {
        static int fail_log_count = 0;
        if ((fail_log_count++ % 10) == 0) {
            printf("[ZoneDetect] not detected: blue_valid=%d red_valid=%d\n",
                   blue_rect.valid,
                   red_rect.valid);
        }
    }
}

