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
#define ZONE_BLACK_MAX_Y 94
#define ZONE_BLACK_MAX_CHROMA_DELTA 32
#define ZONE_BLACK_MIN_AREA 12000.0f
#define ZONE_BLACK_MAX_AREA 900000.0f
#define ZONE_BLACK_BORDER_MARGIN 8
#define ZONE_BLACK_SEGMENT_MIN_PIXELS 34
#define ZONE_BLACK_MERGED_MIN_WIDTH 360.0f
#define ZONE_BLACK_MERGED_MIN_HEIGHT 120.0f
#define ZONE_BLACK_LINE_MIN_PIXELS 95
#define ZONE_BLACK_LINE_MIN_SPAN 330
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
#define ZONE_LOCK_STABLE_FRAMES 1
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

static void zone_apply_workzone_perspective(ZoneRect *rect, int is_left_zone) {
    float w;
    float h;
    float left_small;
    float right_small;
    float right_outer;
    float top_y;
    float bottom_y;

    if (rect == NULL || !rect->valid) {
        return;
    }

    w = rect->x2 - rect->x1;
    h = rect->y2 - rect->y1;
    if (w <= 0.0f || h <= 0.0f) {
        return;
    }

    left_small = w * 0.128f;
    right_small = w * 0.024f;
    right_outer = w * 0.140f;
    top_y = rect->y1 + h * 0.12f;
    bottom_y = rect->y2 - h * 0.04f;

    if (is_left_zone) {
        /* Left zone: bottom x is slightly smaller than top x on both side edges. */
        rect->p0x = rect->x1 + left_small + w * 0.020f;
        rect->p0y = top_y + h * 0.020f;
        rect->p1x = rect->x2 - w * 0.020f;
        rect->p1y = top_y + h * 0.040f;
        rect->p2x = rect->x2 - right_small - w * 0.020f;
        rect->p2y = bottom_y - h * 0.020f;
        rect->p3x = rect->x1 - left_small * 0.45f + w * 0.020f;
        rect->p3y = bottom_y - h * 0.020f;
    } else {
        /* Right zone: outer/right edge bottom protrudes right more than the top. */
        rect->p0x = rect->x1 - right_small * 0.25f + w * 0.020f;
        rect->p0y = top_y + h * 0.020f;
        rect->p1x = rect->x2 - right_outer - w * 0.020f;
        rect->p1y = top_y + h * 0.040f;
        rect->p2x = rect->x2 + right_outer * 0.42f - w * 0.020f;
        rect->p2y = bottom_y - h * 0.020f;
        rect->p3x = rect->x1 + right_small + w * 0.020f;
        rect->p3y = bottom_y - h * 0.020f;
    }

    if (rect->p0x < rect->x1 - 40.0f) rect->p0x = rect->x1 - 40.0f;
    if (rect->p3x < rect->x1 - 40.0f) rect->p3x = rect->x1 - 40.0f;
    if (rect->p1x > rect->x2 + 40.0f) rect->p1x = rect->x2 + 40.0f;
    if (rect->p2x > rect->x2 + 40.0f) rect->p2x = rect->x2 + 40.0f;
}

static int zone_find_loose_red_component_roi(const uint8_t *mask,
                                             int x_min,
                                             int x_max,
                                             int y_min,
                                             int y_max,
                                             ZoneRect *best_rect) {
    uint8_t *visited = NULL;
    int *queue = NULL;
    int best_count = 0;

    if (mask == NULL || best_rect == NULL) {
        return 0;
    }

    if (x_min < ZONE_BLACK_BORDER_MARGIN) x_min = ZONE_BLACK_BORDER_MARGIN;
    if (x_max > WIDTH - ZONE_BLACK_BORDER_MARGIN) x_max = WIDTH - ZONE_BLACK_BORDER_MARGIN;
    if (y_min < ZONE_SEARCH_Y_MIN) y_min = ZONE_SEARCH_Y_MIN;
    if (y_max > ZONE_SEARCH_Y_MAX) y_max = ZONE_SEARCH_Y_MAX;
    if (x_min >= x_max || y_min >= y_max) return 0;

    memset(best_rect, 0, sizeof(*best_rect));
    visited = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    queue = (int *)malloc((size_t)WIDTH * HEIGHT * sizeof(int));
    if (visited == NULL || queue == NULL) {
        free(visited);
        free(queue);
        return 0;
    }

    for (int y = y_min; y <= y_max; y += ZONE_COLOR_STEP) {
        for (int x = x_min; x <= x_max; x += ZONE_COLOR_STEP) {
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
                const int nx[4] = { cx - ZONE_COLOR_STEP, cx + ZONE_COLOR_STEP, cx, cx };
                const int ny[4] = { cy, cy, cy - ZONE_COLOR_STEP, cy + ZONE_COLOR_STEP };

                zone_update_candidate(&rect, &count, cx, cy);

                for (int i = 0; i < 4; i++) {
                    int px = nx[i];
                    int py = ny[i];
                    int next;
                    if (px < x_min || px > x_max || py < y_min || py > y_max) {
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

            rect.valid = count >= 14;
            rect.x2 += ZONE_COLOR_STEP;
            rect.y2 += ZONE_COLOR_STEP;
            if (rect.valid &&
                (rect.x2 - rect.x1) >= 70.0f &&
                (rect.y2 - rect.y1) >= 24.0f &&
                (rect.x2 - rect.x1) <= 470.0f &&
                (rect.y2 - rect.y1) <= 240.0f &&
                count > best_count) {
                best_count = count;
                *best_rect = rect;
            }
        }
    }

    free(visited);
    free(queue);

    if (best_count > 0) {
        best_rect->p0x = best_rect->x1;
        best_rect->p0y = best_rect->y1;
        best_rect->p1x = best_rect->x2;
        best_rect->p1y = best_rect->y1;
        best_rect->p2x = best_rect->x2;
        best_rect->p2y = best_rect->y2;
        best_rect->p3x = best_rect->x1;
        best_rect->p3y = best_rect->y2;
        best_rect->valid = 1;
        return 1;
    }
    return 0;
}

static int zone_find_two_red_work_regions(const uint8_t *mask,
                                          ZoneRect *zone1,
                                          ZoneRect *zone2) {
    int left_ok;
    int right_ok;

    if (mask == NULL || zone1 == NULL || zone2 == NULL) {
        return 0;
    }

    /*
     * Restore the old stable red-zone strategy: red mask -> connected component
     * -> three-edge frame validation.  Run it independently in the lower-left
     * and lower-right floor ROIs for the two work zones.
     */
    left_ok = zone_find_loose_red_component_roi(mask,
                                                100,
                                                570,
                                                300,
                                                560,
                                                zone1);
    right_ok = zone_find_loose_red_component_roi(mask,
                                                 500,
                                                 1260,
                                                 270,
                                                 660,
                                                 zone2);

    if (left_ok) {
        zone_apply_workzone_perspective(zone1, 1);
    }
    if (right_ok) {
        zone_apply_workzone_perspective(zone2, 0);
    }

    if (!left_ok || !right_ok) {
        printf("[ZoneDetect] red work zones not detected: left=%d right=%d\n",
               left_ok, right_ok);
        return 0;
    }

    printf("[ZoneDetect] red work zones old-component zone1=[%.0f,%.0f,%.0f,%.0f] zone2=[%.0f,%.0f,%.0f,%.0f]\n",
           zone1->x1, zone1->y1, zone1->x2, zone1->y2,
           zone2->x1, zone2->y1, zone2->x2, zone2->y2);
    return 1;
}

static int zone_detect_color_rects_nv12(const uint8_t *nv12,
                                        ZoneRect *work_rect,
                                        ZoneRect *red_rect) {
    const uint8_t *y_plane;
    const uint8_t *uv_plane;
    uint8_t *red_mask = NULL;
    int ok = 0;

    if (nv12 == NULL || work_rect == NULL || red_rect == NULL) {
        return 0;
    }

    memset(work_rect, 0, sizeof(*work_rect));
    memset(red_rect, 0, sizeof(*red_rect));
    red_mask = (uint8_t *)calloc((size_t)WIDTH * HEIGHT, 1);
    if (red_mask == NULL) {
        return 0;
    }

    y_plane = nv12;
    uv_plane = nv12 + WIDTH * HEIGHT;

    for (int y = ZONE_RED_SEARCH_Y_MIN; y <= ZONE_RED_SEARCH_Y_MAX; y += ZONE_COLOR_STEP) {
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

            if (r > 88 && r > g + 24 && r > b + 14 && v > 6) {
                red_mask[y * WIDTH + x] = 1;
            }
        }
    }

    ok = zone_find_two_red_work_regions(red_mask, work_rect, red_rect);
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
        printf("[ZoneDetect] locked work1=[%.0f,%.0f,%.0f,%.0f] work2=[%.0f,%.0f,%.0f,%.0f] quad2=[%.0f,%.0f %.0f,%.0f %.0f,%.0f %.0f,%.0f]\n",
               locked_work_zone.x1, locked_work_zone.y1, locked_work_zone.x2, locked_work_zone.y2,
               locked_danger_zone.x1, locked_danger_zone.y1, locked_danger_zone.x2, locked_danger_zone.y2,
               locked_danger_zone.p0x, locked_danger_zone.p0y,
               locked_danger_zone.p1x, locked_danger_zone.p1y,
               locked_danger_zone.p2x, locked_danger_zone.p2y,
               locked_danger_zone.p3x, locked_danger_zone.p3y);
    } else {
        static int fail_log_count = 0;
        if ((fail_log_count++ % 10) == 0) {
            printf("[ZoneDetect] not detected: work1_valid=%d work2_valid=%d\n",
                   work_rect.valid,
                   red_rect.valid);
        }
    }
}

