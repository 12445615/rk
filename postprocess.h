#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 6 // 此处无论多少都不影响自适应
#define NMS_THRESH 0.45
#define BOX_THRESH 0.05

typedef struct {
    image_rect_t box;
    float prop;
    int cls_id;
} object_detect_result;

typedef struct {
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

#ifdef __cplusplus
extern "C" {
#endif

int init_post_process();
void deinit_post_process();
char *coco_cls_to_name(int cls_id);

int post_process(rknn_output *outputs, rknn_tensor_attr *out_attr, int num_outputs,
                 float conf_threshold, float nms_threshold,
                 object_detect_result_list *od_results);

#ifdef __cplusplus
}
#endif

#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_