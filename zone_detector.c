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
#define ZONE_BLACK_EDGE_MIN_RATIO 0.20f
#define ZONE_BLACK_MAX_Y 74
#define ZONE_BLACK_MAX_CHROMA_DELTA 20
#define ZONE_BLACK_MIN_AREA 12000.0f
#define ZONE_BLACK_MAX_AREA 900000.0f
#define ZONE_BLACK_BORDER_MARGIN 8
#define ZONE_BLACK_SEGMENT_MIN_PIXELS 34
#define ZONE_BLACK_MERGED_MIN_WIDTH 360.0f
#define ZONE_BLACK_MERGED_MIN_HEIGHT 120.0f
#define ZONE_BLACK_LINE_MIN_PIXELS 150
#define ZONE_BLACK_LINE_MIN_SPAN 420
#define ZONE_SEARCH_Y_MIN 150
#define ZONE_SEARCH_Y_MAX 690
#define ZONE_RED_SEARCH_X_MIN 690
#define ZONE_RED_SEARCH_X_MAX 1230
#define ZONE_RED_SEARCH_Y_MIN 330
#define ZONE_RED_SEARCH_Y_MAX 690
#define ZONE_RED_MIN_PIXELS 80
#define ZONE_RED_MIN_WIDTH 70.0f
#define ZONE_RED_MIN_HEIGHT 35.0f
#define ZONE_RED_ROW_MIN_PIXELS 4
#define ZONE_RED_ROW_BAND_MIN 3
#define ZONE_LOCK_STABLE_FRAMES 3
#define ZONE_LOCK_CENTER_TOL 90.0f
#define ZONE_LOCK_SIZE_TOL 180.0f
#define WIDTH 1280
#define HEIGHT 720

typedef struct {
    ZoneRect work_zone;
    ZoneRect danger_zone;
    ZoneRect pending_work_zone;
    ZoneRect pending_danger_zone;
    int pending_count;
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
    memset(&g_color_zone_runtime.pending_work_zone, 0, sizeof(g_color_zone_runtime.pending_work_zone));
    memset(&g_color_zone_runtime.pending_danger_zone, 0, sizeof(g_color_zone_runtime.pending_danger_zone));
    g_color_zone_runtime.pending_count = 0;
    g_color_zone_runtime.detected = 0;
    g_color_zone_runtime.reset_pending = 1;
    g_color_zone_runtime.last_scan_ms = 0;
    pthread_mutex_unlock(&g_color_zone_lock);
    mqtt_update_zone_detection_result(0);
    printf("[ZoneDetect] reset, waiting for fresh black/red zone detection\n");
}

static void zone_runtime_sync_shared(ZoneOverlayState *shared,
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
        shared->work_zone.p0x = work_zone->p0x;
        shared->work_zone.p0y = work_zone->p0y;
        shared->work_zone.p1x = work_zone->p1x;
        shared->work_zone.p1y = work_zone->p1y;
        shared->work_zone.p2x = work_zone->p2x;
        shared->work_zone.p2y = work_zone->p2y;
        shared->work_zone.p3x = work_zone->p3x;
        shared->work_zone.p3y = work_zone->p3y;
        shared->danger_zone.valid = danger_zone->valid;
        shared->danger_zone.x1 = danger_zone->x1;
        shared->danger_zone.y1 = danger_zone->y1;
        shared->danger_zone.x2 = danger_zone->x2;
        shared->danger_zone.y2 = danger_zone->y2;
        shared->danger_zone.p0x = danger_zone->p0x;
        shared->danger_zone.p0y = danger_zone->p0y;
        shared->danger_zone.p1x = danger_zone->p1x;
        shared->danger_zone.p1y = danger_zone->p1y;
        shared->danger_zone.p2x = danger_zone->p2x;
        shared->danger_zone.p2y = danger_zone->p2y;
        shared->danger_zone.p3x = danger_zone->p3x;
        shared->danger_zone.p3y = danger_zone->p3y;
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

static float zone_abs_float(float value) {
    return value < 0.0f ? -value : value;
}

static int zone_rect_similar(const ZoneRect *a, const ZoneRect *b) {
    float acx;
    float acy;
    float bcx;
    float bcy;
    float aw;
    float ah;
    float bw;
    float bh;

    if (a == NULL || b == NULL || !a->valid || !b->valid) {
        return 0;
    }

    acx = (a->x1 + a->x2) * 0.5f;
    acy = (a->y1 + a->y2) * 0.5f;
    bcx = (b->x1 + b->x2) * 0.5f;
    bcy = (b->y1 + b->y2) * 0.5f;
    aw = a->x2 - a->x1;
    ah = a->y2 - a->y1;
    bw = b->x2 - b->x1;
    bh = b->y2 - b->y1;

    return zone_abs_float(acx - bcx) <= ZONE_LOCK_CENTER_TOL &&
           zone_abs_float(acy - bcy) <= ZONE_LOCK_CENTER_TOL &&
           zone_abs_float(aw - bw) <= ZONE_LOCK_SIZE_TOL &&
           zone_abs_float(ah - bh) <= ZONE_LOCK_SIZE_TOL;
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

static int zone_black_work_rect_candidate(const ZoneRect *rect) {
    float w;
    float h;
    float area;

    if (!zone_rect_valid_size(rect)) {
        return 0;
    }

    w = rect->x2 - rect->x1;
    h = rect->y2 - rect->y1;
    area = w * h;

    if (area < ZONE_BLACK_MIN_AREA || area > ZONE_BLACK_MAX_AREA) {
        return 0;
    }
    if (rect->x1 <= ZONE_BLACK_BORDER_MARGIN ||
        rect->y1 <= ZONE_BLACK_BORDER_MARGIN ||
        rect->x2 >= WIDTH - ZONE_BLACK_BORDER_MARGIN ||
        rect->y2 >= HEIGHT - ZONE_BLACK_BORDER_MARGIN) {
        return 0;
    }

    return 1;
}

static int zone_rect_intersects(const ZoneRect *a, const ZoneRect *b) {
    if (a == NULL || b == NULL || !a->valid || !b->valid) {
        return 0;
    }
    return !(a->x2 < b->x1 || b->x2 < a->x1 ||
             a->y2 < b->y1 || b->y2 < a->y1);
}

static void zone_rect_expand(ZoneRect *dst, const ZoneRect *src) {
    if (dst == NULL || src == NULL || !src->valid) {
        return;
    }
    if (!dst->valid) {
        *dst = *src;
        return;
    }
    if (src->x1 < dst->x1) dst->x1 = src->x1;
    if (src->y1 < dst->y1) dst->y1 = src->y1;
    if (src->x2 > dst->x2) dst->x2 = src->x2;
    if (src->y2 > dst->y2) dst->y2 = src->y2;
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

static int zone_black_rect_has_frame_edges(const ZoneRect *rect,
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
            if (x1 + dx < WIDTH && mask[y * stride + x1 + dx]) left_found = 1;
            if (x2 - dx >= 0 && mask[y * stride + x2 - dx]) right_found = 1;
        }
        left_hits += left_found;
        right_hits += right_found;
    }

    for (int x = x1; x <= x2; x += ZONE_COLOR_STEP) {
        int top_found = 0;
        int bottom_found = 0;
        for (int dy = 0; dy <= ZONE_EDGE_BAND_PX; dy += ZONE_COLOR_STEP) {
            if (y1 + dy < HEIGHT && mask[(y1 + dy) * stride + x]) top_found = 1;
            if (y2 - dy >= 0 && mask[(y2 - dy) * stride + x]) bottom_found = 1;
        }
        top_hits += top_found;
        bottom_hits += bottom_found;
    }

    vertical_required = (int)(((y2 - y1) / ZONE_COLOR_STEP + 1) * ZONE_BLACK_EDGE_MIN_RATIO);
    horizontal_required = (int)(((x2 - x1) / ZONE_COLOR_STEP + 1) * ZONE_BLACK_EDGE_MIN_RATIO);
    if (vertical_required < 6) vertical_required = 6;
    if (horizontal_required < 6) horizontal_required = 6;

    if (left_hits >= vertical_required) passed_edges++;
    if (right_hits >= vertical_required) passed_edges++;
    if (top_hits >= horizontal_required) passed_edges++;
    if (bottom_hits >= horizontal_required) passed_edges++;

    return passed_edges >= 2 &&
           (left_hits >= vertical_required || right_hits >= vertical_required) &&
           (top_hits >= horizontal_required || bottom_hits >= horizontal_required);
}

static int zone_find_best_color_component(const uint8_t *mask,
                                          ZoneRect *best_rect,
                                          int is_work_zone) {
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
            if (rect.valid &&
                (is_work_zone
                     ? !zone_black_rect_has_frame_edges(&rect, mask, WIDTH)
                     : !zone_color_rect_has_frame_edges(&rect, mask, WIDTH))) {
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

static int zone_find_red_region_rect(const uint8_t *mask, ZoneRect *red_rect) {
    int count = 0;
    int left_by_y[HEIGHT];
    int right_by_y[HEIGHT];
    int count_by_y[HEIGHT];
    int valid_rows[HEIGHT];
    int valid_row_count = 0;
    int top_band_count = 0;
    int bottom_band_count = 0;
    int top_left_sum = 0;
    int top_right_sum = 0;
    int top_y_sum = 0;
    int bottom_left_sum = 0;
    int bottom_right_sum = 0;
    int bottom_y_sum = 0;
    int band_rows;

    if (mask == NULL || red_rect == NULL) {
        return 0;
    }

    memset(red_rect, 0, sizeof(*red_rect));
    for (int y = 0; y < HEIGHT; y++) {
        left_by_y[y] = WIDTH;
        right_by_y[y] = -1;
        count_by_y[y] = 0;
    }

    for (int y = ZONE_RED_SEARCH_Y_MIN; y <= ZONE_RED_SEARCH_Y_MAX; y += ZONE_COLOR_STEP) {
        for (int x = ZONE_RED_SEARCH_X_MIN; x <= ZONE_RED_SEARCH_X_MAX; x += ZONE_COLOR_STEP) {
            if (!mask[y * WIDTH + x]) {
                continue;
            }
            zone_update_candidate(red_rect, &count, x, y);
            if (x < left_by_y[y]) left_by_y[y] = x;
            if (x > right_by_y[y]) right_by_y[y] = x;
            count_by_y[y]++;
        }
    }

    red_rect->valid = count >= ZONE_RED_MIN_PIXELS;
    red_rect->x2 += ZONE_COLOR_STEP;
    red_rect->y2 += ZONE_COLOR_STEP;
    if (!red_rect->valid ||
        (red_rect->x2 - red_rect->x1) < ZONE_RED_MIN_WIDTH ||
        (red_rect->y2 - red_rect->y1) < ZONE_RED_MIN_HEIGHT) {
        memset(red_rect, 0, sizeof(*red_rect));
        return 0;
    }

    for (int y = (int)red_rect->y1; y <= (int)red_rect->y2 && y < HEIGHT; y += ZONE_COLOR_STEP) {
        if (count_by_y[y] >= ZONE_RED_ROW_MIN_PIXELS &&
            right_by_y[y] > left_by_y[y] &&
            (right_by_y[y] - left_by_y[y]) >= 18) {
            valid_rows[valid_row_count++] = y;
        }
    }

    band_rows = valid_row_count / 4;
    if (band_rows < ZONE_RED_ROW_BAND_MIN) band_rows = ZONE_RED_ROW_BAND_MIN;
    if (band_rows > valid_row_count / 2) band_rows = valid_row_count / 2;

    if (valid_row_count < ZONE_RED_ROW_BAND_MIN * 2 || band_rows <= 0) {
        red_rect->p0x = red_rect->x1;
        red_rect->p0y = red_rect->y1;
        red_rect->p1x = red_rect->x2;
        red_rect->p1y = red_rect->y1;
        red_rect->p2x = red_rect->x2;
        red_rect->p2y = red_rect->y2;
        red_rect->p3x = red_rect->x1;
        red_rect->p3y = red_rect->y2;
    } else {
        for (int i = 0; i < band_rows; i++) {
            int y = valid_rows[i];
            top_left_sum += left_by_y[y];
            top_right_sum += right_by_y[y];
            top_y_sum += y;
            top_band_count++;
        }
        for (int i = valid_row_count - band_rows; i < valid_row_count; i++) {
            int y = valid_rows[i];
            bottom_left_sum += left_by_y[y];
            bottom_right_sum += right_by_y[y];
            bottom_y_sum += y;
            bottom_band_count++;
        }

        if (top_band_count > 0 && bottom_band_count > 0) {
            float top_left = (float)top_left_sum / top_band_count;
            float top_right = (float)top_right_sum / top_band_count;
            float bottom_left = (float)bottom_left_sum / bottom_band_count;
            float bottom_right = (float)bottom_right_sum / bottom_band_count;
            float top_y_avg = (float)top_y_sum / top_band_count;
            float bottom_y_avg = (float)bottom_y_sum / bottom_band_count;
            float left_edge = red_rect->x1 + 8.0f;
            float right_edge = red_rect->x2 - 8.0f;
            float top_edge = red_rect->y1 + 8.0f;
            float bottom_edge = red_rect->y2 - 8.0f;

            if (top_left > left_edge + 60.0f) top_left = left_edge;
            if (bottom_left > left_edge + 55.0f) bottom_left = left_edge + 10.0f;
            if (top_right < right_edge - 80.0f) top_right = right_edge - 85.0f;
            if (bottom_right < right_edge - 45.0f) bottom_right = right_edge;
            if (bottom_left > top_left + 70.0f) bottom_left = top_left + 45.0f;
            if (bottom_right < top_right + 20.0f) bottom_right = top_right + 55.0f;
            if (top_y_avg > top_edge + 55.0f) top_y_avg = top_edge;
            if (bottom_y_avg < bottom_edge - 55.0f) bottom_y_avg = bottom_edge;

            red_rect->p0x = top_left;
            red_rect->p0y = top_y_avg;
            red_rect->p1x = top_right;
            red_rect->p1y = top_y_avg + 4.0f;
            red_rect->p2x = bottom_right;
            red_rect->p2y = bottom_y_avg;
            red_rect->p3x = bottom_left;
            red_rect->p3y = bottom_y_avg - 4.0f;
        }
    }

    if (red_rect->p0x < red_rect->x1) red_rect->p0x = red_rect->x1;
    if (red_rect->p3x < red_rect->x1) red_rect->p3x = red_rect->x1;
    if (red_rect->p1x > red_rect->x2) red_rect->p1x = red_rect->x2;
    if (red_rect->p2x > red_rect->x2) red_rect->p2x = red_rect->x2;

    return 1;
}

static int zone_measure_black_row_band(const uint8_t *mask,
                                       int center_y,
                                       int x_min,
                                       int x_max,
                                       int *out_x1,
                                       int *out_x2) {
    int best_count = 0;
    int best_x1 = WIDTH;
    int best_x2 = -1;

    if (mask == NULL) {
        return 0;
    }
    if (center_y < ZONE_SEARCH_Y_MIN) center_y = ZONE_SEARCH_Y_MIN;
    if (center_y > ZONE_SEARCH_Y_MAX) center_y = ZONE_SEARCH_Y_MAX;
    if (x_min < ZONE_BLACK_BORDER_MARGIN) x_min = ZONE_BLACK_BORDER_MARGIN;
    if (x_max > WIDTH - ZONE_BLACK_BORDER_MARGIN) x_max = WIDTH - ZONE_BLACK_BORDER_MARGIN;

    for (int y = center_y - 6; y <= center_y + 6; y += ZONE_COLOR_STEP) {
        int run_count = 0;
        int run_x1 = -1;
        int gap = 0;
        if (y < ZONE_SEARCH_Y_MIN || y > ZONE_SEARCH_Y_MAX) {
            continue;
        }
        for (int x = x_min; x <= x_max; x += ZONE_COLOR_STEP) {
            if (mask[y * WIDTH + x]) {
                if (run_count == 0) {
                    run_x1 = x;
                }
                run_count++;
                gap = 0;
            } else if (run_count > 0 && gap < 5) {
                gap++;
            } else {
                int run_x2 = x - (gap + 1) * ZONE_COLOR_STEP;
                if (run_count > best_count && run_x2 > run_x1) {
                    best_count = run_count;
                    best_x1 = run_x1;
                    best_x2 = run_x2;
                }
                run_count = 0;
                run_x1 = -1;
                gap = 0;
            }
        }
        if (run_count > 0) {
            int run_x2 = x_max - gap * ZONE_COLOR_STEP;
            if (run_count > best_count && run_x2 > run_x1) {
                best_count = run_count;
                best_x1 = run_x1;
                best_x2 = run_x2;
            }
        }
    }

    if (out_x1 != NULL) *out_x1 = best_x1;
    if (out_x2 != NULL) *out_x2 = best_x2;
    return best_count;
}

static int zone_find_best_black_horizontal_line(const uint8_t *mask,
                                                int y_min,
                                                int y_max,
                                                int x_min,
                                                int x_max,
                                                int *best_y,
                                                int *best_x1,
                                                int *best_x2) {
    int best_score = 0;
    int found = 0;

    if (mask == NULL || best_y == NULL || best_x1 == NULL || best_x2 == NULL) {
        return 0;
    }
    if (y_min < ZONE_SEARCH_Y_MIN) y_min = ZONE_SEARCH_Y_MIN;
    if (y_max > ZONE_SEARCH_Y_MAX) y_max = ZONE_SEARCH_Y_MAX;
    if (y_min > y_max) {
        return 0;
    }

    for (int y = y_min; y <= y_max; y += ZONE_COLOR_STEP) {
        int x1 = WIDTH;
        int x2 = -1;
        int count = zone_measure_black_row_band(mask, y, x_min, x_max, &x1, &x2);
        int span = x2 - x1;
        int score;

        if (count < ZONE_BLACK_LINE_MIN_PIXELS || span < ZONE_BLACK_LINE_MIN_SPAN) {
            continue;
        }
        score = count * 3 + span / 4;
        if (!found || score > best_score) {
            found = 1;
            best_score = score;
            *best_y = y;
            *best_x1 = x1;
            *best_x2 = x2;
        }
    }

    return found;
}

static int zone_find_lowest_black_horizontal_line(const uint8_t *mask,
                                               int y_min,
                                               int y_max,
                                               int x_min,
                                               int x_max,
                                               int *best_y,
                                               int *best_x1,
                                               int *best_x2) {
    int found = 0;

    if (mask == NULL || best_y == NULL || best_x1 == NULL || best_x2 == NULL) {
        return 0;
    }
    if (y_min < ZONE_SEARCH_Y_MIN) y_min = ZONE_SEARCH_Y_MIN;
    if (y_max > ZONE_SEARCH_Y_MAX) y_max = ZONE_SEARCH_Y_MAX;
    if (y_min > y_max) {
        return 0;
    }

    for (int y = y_min; y <= y_max; y += ZONE_COLOR_STEP) {
        int x1 = WIDTH;
        int x2 = -1;
        int count = zone_measure_black_row_band(mask, y, x_min, x_max, &x1, &x2);
        int span = x2 - x1;

        if (count < ZONE_BLACK_LINE_MIN_PIXELS || span < ZONE_BLACK_LINE_MIN_SPAN) {
            continue;
        }

        if (!found || y > *best_y) {
            found = 1;
            *best_y = y;
            *best_x1 = x1;
            *best_x2 = x2;
        }
    }

    return found;
}

static float zone_clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int zone_find_black_work_rect_by_lines(const uint8_t *mask,
                                              const ZoneRect *red_rect,
                                              ZoneRect *work_rect) {
    int bottom_y = 0;
    int bottom_x1 = 0;
    int bottom_x2 = 0;
    int bottom_found;
    int search_x1;
    int search_x2;
    float outer_left;
    float outer_right;
    float top_y;
    float bottom_left_y;
    float bottom_right_y;
    float top_slope = 0.0f;
    float bottom_slope = 0.0f;
    float red_top_dx;
    float red_bottom_dx;
    float top_x1;
    float top_x2;
    float bottom_left;
    float bottom_right;
    float red_right_dx;
    float red_right_dy;
    float right_dx_per_y = 0.0f;

    if (mask == NULL || red_rect == NULL || !red_rect->valid || work_rect == NULL) {
        return 0;
    }

    memset(work_rect, 0, sizeof(*work_rect));
    search_x1 = ZONE_BLACK_BORDER_MARGIN;
    search_x2 = WIDTH - ZONE_BLACK_BORDER_MARGIN;

    bottom_found = zone_find_lowest_black_horizontal_line(mask,
                                                          (int)(red_rect->y2 + 18.0f),
                                                          (int)(red_rect->y2 + 115.0f),
                                                          search_x1,
                                                          search_x2,
                                                          &bottom_y,
                                                          &bottom_x1,
                                                          &bottom_x2);
    if (!bottom_found) {
        return 0;
    }

    red_top_dx = red_rect->p1x - red_rect->p0x;
    red_bottom_dx = red_rect->p2x - red_rect->p3x;
    if (red_top_dx > 20.0f || red_top_dx < -20.0f) {
        top_slope = (red_rect->p1y - red_rect->p0y) / red_top_dx;
    }
    if (red_bottom_dx > 20.0f || red_bottom_dx < -20.0f) {
        bottom_slope = (red_rect->p2y - red_rect->p3y) / red_bottom_dx;
    } else {
        bottom_slope = top_slope;
    }
    red_right_dx = red_rect->p2x - red_rect->p1x;
    red_right_dy = red_rect->p2y - red_rect->p1y;
    if (red_right_dy > 10.0f || red_right_dy < -10.0f) {
        right_dx_per_y = red_right_dx / red_right_dy;
    }

    /*
     * The black work zone is the outer frame.  The front/back edges are
     * generated parallel to the red danger zone edges, while the bottom edge
     * is anchored by the lowest long black line in the lower ROI.
     */
    outer_left = red_rect->x1 - 830.0f;
    outer_right = red_rect->x2 + 105.0f;
    outer_left = zone_clamp_float(outer_left, (float)ZONE_BLACK_BORDER_MARGIN, (float)(WIDTH - 260));
    outer_right = zone_clamp_float(outer_right, red_rect->x2 + 35.0f, (float)(WIDTH - ZONE_BLACK_BORDER_MARGIN));

    bottom_left = (float)bottom_x1;
    bottom_right = (float)bottom_x2;
    if (bottom_left > outer_left + 60.0f) bottom_left = outer_left;
    if (bottom_right < outer_right) bottom_right = outer_right;
    if (bottom_right > WIDTH - ZONE_BLACK_BORDER_MARGIN) {
        bottom_right = WIDTH - ZONE_BLACK_BORDER_MARGIN;
    }

    bottom_left_y = (float)bottom_y - 7.0f;
    bottom_right_y = bottom_left_y + bottom_slope * (bottom_right - bottom_left);
    bottom_left_y = zone_clamp_float(bottom_left_y, (float)ZONE_SEARCH_Y_MIN, (float)ZONE_SEARCH_Y_MAX);
    bottom_right_y = zone_clamp_float(bottom_right_y, (float)ZONE_SEARCH_Y_MIN, (float)ZONE_SEARCH_Y_MAX);

    top_y = red_rect->y1 - 48.0f;
    if (top_y < ZONE_SEARCH_Y_MIN) top_y = (float)ZONE_SEARCH_Y_MIN;
    if (top_y > bottom_left_y - 110.0f) top_y = bottom_left_y - 135.0f;
    if (top_y < ZONE_SEARCH_Y_MIN) top_y = (float)ZONE_SEARCH_Y_MIN;

    top_x1 = outer_left + 245.0f;
    /*
     * Make the black right side parallel to the red danger-zone right side:
     * p1(top-right) is derived from p2(bottom-right) using the red side
     * dx/dy, instead of being independently clamped toward the wall corner.
     */
    top_x2 = bottom_right - right_dx_per_y * (bottom_right_y - top_y);
    if (top_x2 < red_rect->x2 - 95.0f) top_x2 = red_rect->x2 - 95.0f;
    if (top_x2 > red_rect->x2 + 18.0f) top_x2 = red_rect->x2 + 18.0f;
    if (top_x2 <= top_x1 + 300.0f) top_x1 = top_x2 - 330.0f;
    if (top_x1 < ZONE_BLACK_BORDER_MARGIN) top_x1 = ZONE_BLACK_BORDER_MARGIN;

    work_rect->valid = 1;
    work_rect->p0x = top_x1;
    work_rect->p0y = top_y;
    if (top_x2 > red_rect->x2 - 20.0f) top_x2 = red_rect->x2 - 20.0f;
    work_rect->p1x = top_x2;
    work_rect->p1y = top_y + top_slope * (top_x2 - top_x1);
    work_rect->p2x = bottom_right;
    work_rect->p2y = bottom_right_y;
    work_rect->p3x = bottom_left;
    work_rect->p3y = bottom_left_y;

    work_rect->x1 = work_rect->p3x < work_rect->p0x ? work_rect->p3x : work_rect->p0x;
    work_rect->x2 = work_rect->p2x > work_rect->p1x ? work_rect->p2x : work_rect->p1x;
    work_rect->y1 = work_rect->p0y < work_rect->p1y ? work_rect->p0y : work_rect->p1y;
    work_rect->y2 = work_rect->p3y > work_rect->p2y ? work_rect->p3y : work_rect->p2y;

    if ((work_rect->x2 - work_rect->x1) < ZONE_BLACK_MERGED_MIN_WIDTH ||
        (work_rect->y2 - work_rect->y1) < ZONE_BLACK_MERGED_MIN_HEIGHT ||
        !zone_rect_contains_center(work_rect, red_rect) ||
        work_rect->x2 < red_rect->x2 + 35.0f ||
        work_rect->x1 > red_rect->x1 - 80.0f) {
        memset(work_rect, 0, sizeof(*work_rect));
        return 0;
    }

    printf("[ZoneDetect] black parallel work top=[%.0f,%.0f %.0f,%.0f] bottom=[%.0f,%.0f %.0f,%.0f] red=[%.0f,%.0f,%.0f,%.0f]\n",
           work_rect->p0x, work_rect->p0y,
           work_rect->p1x, work_rect->p1y,
           work_rect->p3x, work_rect->p3y,
           work_rect->p2x, work_rect->p2y,
           red_rect->x1, red_rect->y1, red_rect->x2, red_rect->y2);
    return 1;
}

static int zone_find_merged_black_work_rect(const uint8_t *mask,
                                            const ZoneRect *red_rect,
                                            ZoneRect *work_rect) {
    uint8_t *visited = NULL;
    int *queue = NULL;
    ZoneRect search_roi;
    int merged_pixels = 0;

    if (mask == NULL || red_rect == NULL || !red_rect->valid || work_rect == NULL) {
        return 0;
    }

    memset(work_rect, 0, sizeof(*work_rect));
    memset(&search_roi, 0, sizeof(search_roi));
    search_roi.valid = 1;
    search_roi.x1 = red_rect->x1 - 520.0f;
    search_roi.y1 = red_rect->y1 - 170.0f;
    search_roi.x2 = red_rect->x2 + 120.0f;
    search_roi.y2 = red_rect->y2 + 120.0f;
    if (search_roi.x1 < ZONE_BLACK_BORDER_MARGIN) search_roi.x1 = ZONE_BLACK_BORDER_MARGIN;
    if (search_roi.y1 < ZONE_SEARCH_Y_MIN) search_roi.y1 = ZONE_SEARCH_Y_MIN;
    if (search_roi.x2 > WIDTH - ZONE_BLACK_BORDER_MARGIN) search_roi.x2 = WIDTH - ZONE_BLACK_BORDER_MARGIN;
    if (search_roi.y2 > ZONE_SEARCH_Y_MAX) search_roi.y2 = ZONE_SEARCH_Y_MAX;

    visited = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    queue = (int *)malloc((size_t)WIDTH * HEIGHT * sizeof(int));
    if (visited == NULL || queue == NULL) {
        free(visited);
        free(queue);
        return 0;
    }

    for (int y = (int)search_roi.y1; y <= (int)search_roi.y2; y += ZONE_COLOR_STEP) {
        for (int x = (int)search_roi.x1; x <= (int)search_roi.x2; x += ZONE_COLOR_STEP) {
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
                const int nx[4] = {cx - ZONE_COLOR_STEP, cx + ZONE_COLOR_STEP, cx, cx};
                const int ny[4] = {cy, cy, cy - ZONE_COLOR_STEP, cy + ZONE_COLOR_STEP};

                zone_update_candidate(&rect, &count, cx, cy);

                for (int i = 0; i < 4; i++) {
                    int px = nx[i];
                    int py = ny[i];
                    int next;
                    if (px < (int)search_roi.x1 || px > (int)search_roi.x2 ||
                        py < (int)search_roi.y1 || py > (int)search_roi.y2) {
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

            rect.valid = count >= ZONE_BLACK_SEGMENT_MIN_PIXELS;
            rect.x2 += ZONE_COLOR_STEP;
            rect.y2 += ZONE_COLOR_STEP;
            if (!rect.valid || !zone_rect_intersects(&rect, &search_roi)) {
                continue;
            }

            zone_rect_expand(work_rect, &rect);
            merged_pixels += count;
        }
    }

    free(visited);
    free(queue);

    if (!work_rect->valid || merged_pixels < ZONE_COLOR_MIN_PIXELS) {
        return 0;
    }
    if ((work_rect->x2 - work_rect->x1) < ZONE_BLACK_MERGED_MIN_WIDTH ||
        (work_rect->y2 - work_rect->y1) < ZONE_BLACK_MERGED_MIN_HEIGHT ||
        (work_rect->x2 - work_rect->x1) > 760.0f ||
        (work_rect->y2 - work_rect->y1) > 360.0f) {
        work_rect->valid = 0;
        return 0;
    }
    if (!zone_rect_contains_center(work_rect, red_rect)) {
        work_rect->valid = 0;
        return 0;
    }

    return 1;
}

static int zone_detect_color_rects_nv12(const uint8_t *nv12,
                                        ZoneRect *work_rect,
                                        ZoneRect *red_rect) {
    const uint8_t *y_plane;
    const uint8_t *uv_plane;
    uint8_t *work_mask = NULL;
    uint8_t *red_mask = NULL;
    int ok = 0;

    if (nv12 == NULL || work_rect == NULL || red_rect == NULL) {
        return 0;
    }

    memset(work_rect, 0, sizeof(*work_rect));
    memset(red_rect, 0, sizeof(*red_rect));
    work_mask = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    red_mask = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    if (work_mask == NULL || red_mask == NULL) {
        free(work_mask);
        free(red_mask);
        return 0;
    }

    y_plane = nv12;
    uv_plane = nv12 + WIDTH * HEIGHT;

    for (int y = ZONE_SEARCH_Y_MIN; y <= ZONE_SEARCH_Y_MAX; y += ZONE_COLOR_STEP) {
        const uint8_t *y_row = y_plane + y * WIDTH;
        const uint8_t *uv_row = uv_plane + (y / 2) * WIDTH;
        for (int x = ZONE_BLACK_BORDER_MARGIN; x < WIDTH - ZONE_BLACK_BORDER_MARGIN; x += ZONE_COLOR_STEP) {
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

            if (yy < ZONE_BLACK_MAX_Y &&
                abs(u) <= ZONE_BLACK_MAX_CHROMA_DELTA &&
                abs(v) <= ZONE_BLACK_MAX_CHROMA_DELTA) {
                work_mask[y * WIDTH + x] = 1;
            } else if (x >= ZONE_RED_SEARCH_X_MIN &&
                       x <= ZONE_RED_SEARCH_X_MAX &&
                       y >= ZONE_RED_SEARCH_Y_MIN &&
                       y <= ZONE_RED_SEARCH_Y_MAX &&
                       r > 85 && r > g + 18 && r > b + 12) {
                red_mask[y * WIDTH + x] = 1;
            }
        }
    }

    red_rect->valid = zone_find_red_region_rect(red_mask, red_rect);
    if (red_rect->valid) {
        work_rect->valid = zone_find_black_work_rect_by_lines(work_mask, red_rect, work_rect);
    } else {
        work_rect->valid = zone_find_best_color_component(work_mask, work_rect, 1);
        if (work_rect->valid && !zone_black_work_rect_candidate(work_rect)) {
            work_rect->valid = 0;
        }
    }
    ok = work_rect->valid && red_rect->valid &&
         zone_rect_contains_center(work_rect, red_rect);
    free(work_mask);
    free(red_mask);

    return ok;
}

void zone_runtime_scan_frame(const CameraCtx *cam,
                                    int buffer_index,
                                    int64_t frame_mono_ms,
                                    ZoneOverlayState *shared) {
    ZoneRect work_rect;
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
        mqtt_update_zone_detection_result(1);
        return;
    }

    if (g_color_zone_runtime.last_scan_ms != 0 &&
        frame_mono_ms - g_color_zone_runtime.last_scan_ms < ZONE_COLOR_SCAN_INTERVAL_MS) {
        return;
    }
    g_color_zone_runtime.last_scan_ms = frame_mono_ms;

    ok = zone_detect_color_rects_nv12((const uint8_t *)cam->buffers[buffer_index].start,
                                      &work_rect,
                                      &red_rect);
    pthread_mutex_lock(&g_color_zone_lock);
    if (ok) {
        if (zone_rect_similar(&work_rect, &g_color_zone_runtime.pending_work_zone) &&
            zone_rect_similar(&red_rect, &g_color_zone_runtime.pending_danger_zone)) {
            g_color_zone_runtime.pending_count++;
        } else {
            g_color_zone_runtime.pending_work_zone = work_rect;
            g_color_zone_runtime.pending_danger_zone = red_rect;
            g_color_zone_runtime.pending_count = 1;
        }

        if (g_color_zone_runtime.pending_count >= ZONE_LOCK_STABLE_FRAMES) {
            g_color_zone_runtime.detected = 1;
            g_color_zone_runtime.work_zone = g_color_zone_runtime.pending_work_zone;
            g_color_zone_runtime.danger_zone = g_color_zone_runtime.pending_danger_zone;
            locked_work_zone = g_color_zone_runtime.work_zone;
            locked_danger_zone = g_color_zone_runtime.danger_zone;
        } else {
            ok = 0;
        }
    } else {
        g_color_zone_runtime.pending_count = 0;
        memset(&g_color_zone_runtime.pending_work_zone, 0, sizeof(g_color_zone_runtime.pending_work_zone));
        memset(&g_color_zone_runtime.pending_danger_zone, 0, sizeof(g_color_zone_runtime.pending_danger_zone));
    }
    pthread_mutex_unlock(&g_color_zone_lock);
    zone_runtime_sync_shared(shared,
                             ok ? &locked_work_zone : NULL,
                             ok ? &locked_danger_zone : NULL,
                             ok);
    mqtt_update_zone_detection_result(ok);

    if (ok) {
        printf("[ZoneDetect] locked black work=[%.0f,%.0f,%.0f,%.0f] red danger=[%.0f,%.0f,%.0f,%.0f] quad=[%.0f,%.0f %.0f,%.0f %.0f,%.0f %.0f,%.0f]\n",
               locked_work_zone.x1, locked_work_zone.y1, locked_work_zone.x2, locked_work_zone.y2,
               locked_danger_zone.x1, locked_danger_zone.y1, locked_danger_zone.x2, locked_danger_zone.y2,
               locked_danger_zone.p0x, locked_danger_zone.p0y,
               locked_danger_zone.p1x, locked_danger_zone.p1y,
               locked_danger_zone.p2x, locked_danger_zone.p2y,
               locked_danger_zone.p3x, locked_danger_zone.p3y);
    } else {
        static int fail_log_count = 0;
        if ((fail_log_count++ % 10) == 0) {
            printf("[ZoneDetect] not detected: black_valid=%d red_valid=%d\n",
                   work_rect.valid,
                   red_rect.valid);
        }
    }
}

