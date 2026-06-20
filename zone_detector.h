#ifndef ZONE_DETECTOR_H
#define ZONE_DETECTOR_H

#include <stdint.h>
#include "camera.h"
#include "detect_shared.h"

typedef struct {
    int valid;
    float x1;
    float y1;
    float x2;
    float y2;
    float p0x;
    float p0y;
    float p1x;
    float p1y;
    float p2x;
    float p2y;
    float p3x;
    float p3y;
} ZoneRect;

int zone_runtime_get_detected(ZoneRect *work_zone, ZoneRect *danger_zone);
void zone_runtime_reset(void);
void zone_runtime_scan_frame(const CameraCtx *cam,
                             int buffer_index,
                             int64_t frame_mono_ms,
                             ZoneOverlayState *zone_state);

#endif
