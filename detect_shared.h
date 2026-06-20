#ifndef DETECT_SHARED_H
#define DETECT_SHARED_H

#include <stdint.h>

#define DETECT_MAX_BOXES 64

typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int class_id;
} DetectBox;

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
} DetectZoneRect;

typedef struct {
    uint32_t version;
    int zone_valid;
    DetectZoneRect work_zone;
    DetectZoneRect danger_zone;
} ZoneOverlayState;

typedef struct {
    uint32_t version;
    int valid;
    int box_count;
    int64_t frame_seq;
    int64_t timestamp_ms;
    int zone_valid;
    DetectZoneRect work_zone;
    DetectZoneRect danger_zone;
    DetectBox boxes[DETECT_MAX_BOXES];
} DetectSharedState;

#endif
