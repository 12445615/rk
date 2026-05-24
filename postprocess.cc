#include "postprocess.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>

// 1. 注入你专属的 5 个标签
static const char *project_labels[5] = {
    "kask var",  // 0: 有安全帽
    "kask yok",  // 1: 无安全帽
    "yelek var", // 2: 有反光衣
    "yelek yok", // 3: 无反光衣
    "fire"       // 4: 火灾
};

inline static float clamp(float val, float min, float max) { 
    return val > min ? (val < max ? val : max) : min; 
}

// IOU 计算 (直接沿用你验证通过的代码)
static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, 
                              float xmin1, float ymin1, float xmax1, float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    return u <= 0.f ? 0.f : (i / u);
}

// 视频流专用后处理
int post_process(rknn_output *outputs, rknn_tensor_attr *out_attr, int num_outputs,
                 float conf_threshold, float nms_threshold,
                 object_detect_result_list *od_results)
{
    memset(od_results, 0, sizeof(object_detect_result_list));
    if (num_outputs > 1) return 0;

    float *data = (float *)outputs[0].buf;
    int num_anchors = 8400;
    int num_classes = 5;

    // 视频流原始分辨率与模型分辨率
    const int model_w = 640;
    const int model_h = 640;
    const int cam_w = 1280;
    const int cam_h = 720;

    // 计算 RGA 的缩放比例和黑边偏移量
    float scale = std::min((float)model_w / cam_w, (float)model_h / cam_h);
    float offset_x = (model_w - cam_w * scale) / 2.0f;
    float offset_y = (model_h - cam_h * scale) / 2.0f;

    std::vector<object_detect_result> temp_results;

    // 2. 遍历 8400 个锚点 (严格采用你昨日验证通过的 NCHW 提取法)
    for (int i = 0; i < num_anchors; ++i) {
        float max_class_prob = 0.0f;
        int max_class_id = -1;

        // 提取 5 个类别中的最高得分
        for (int c = 0; c < num_classes; ++c) {
            float prob = data[(4 + c) * num_anchors + i];
            if (prob > max_class_prob) {
                max_class_prob = prob;
                max_class_id = c;
            }
        }

        // 3. 超过阈值才解析坐标 (推荐阈值 0.5)
        if (max_class_prob > conf_threshold) {
            float cx = data[0 * num_anchors + i];
            float cy = data[1 * num_anchors + i];
            float w  = data[2 * num_anchors + i];
            float h  = data[3 * num_anchors + i];

            // 过滤极端的垃圾框
            if (w <= 0 || h <= 0 || w > 2000) continue;

            // 中心点转角点
            float x1 = cx - w / 2.0f;
            float y1 = cy - h / 2.0f;
            float x2 = cx + w / 2.0f;
            float y2 = cy + h / 2.0f;

            object_detect_result res;
            res.cls_id = max_class_id;
            res.prop = max_class_prob;

            // 逆向缩放，还原回 1280x720 的真实坐标 (带 clamp 防越界)
            res.box.left   = (int)(clamp((x1 - offset_x) / scale, 0, cam_w));
            res.box.top    = (int)(clamp((y1 - offset_y) / scale, 0, cam_h));
            res.box.right  = (int)(clamp((x2 - offset_x) / scale, 0, cam_w));
            res.box.bottom = (int)(clamp((y2 - offset_y) / scale, 0, cam_h));

            temp_results.push_back(res);
        }
    }

    // 4. 排序与 NMS
    std::sort(temp_results.begin(), temp_results.end(), [](const object_detect_result &a, const object_detect_result &b) {
        return a.prop > b.prop;
    });

    std::vector<bool> suppressed(temp_results.size(), false);
    od_results->count = 0;

    for (size_t i = 0; i < temp_results.size(); ++i) {
        if (suppressed[i]) continue;
        if (od_results->count >= OBJ_NUMB_MAX_SIZE) break;
        
        od_results->results[od_results->count++] = temp_results[i];

        for (size_t j = i + 1; j < temp_results.size(); ++j) {
            if (suppressed[j] || temp_results[i].cls_id != temp_results[j].cls_id) continue;

            float iou = CalculateOverlap(
                temp_results[i].box.left, temp_results[i].box.top, temp_results[i].box.right, temp_results[i].box.bottom,
                temp_results[j].box.left, temp_results[j].box.top, temp_results[j].box.right, temp_results[j].box.bottom
            );

            if (iou > nms_threshold) suppressed[j] = true;
        }
    }

    // 在终端打印检测到的目标
    if (od_results->count > 0) {
        static int log_limit = 0;
        if (log_limit++ % 30 == 0) {
            auto best = od_results->results[0];
            printf("\n[AI 视频检测] 目标: %s | 置信度: %.2f%% | 坐标: [%d, %d, %d, %d]\n",
                   coco_cls_to_name(best.cls_id), best.prop * 100.0f, 
                   best.box.left, best.box.top, best.box.right, best.box.bottom);
        }
    }

    return 0;
}

int init_post_process() { return 0; }
void deinit_post_process() {}

extern "C" char *coco_cls_to_name(int cls_id) {
    if (cls_id >= 0 && cls_id < 5) return (char*)project_labels[cls_id];
    return (char*)"未知物体";
}