#include "postprocess.h"
#include "Float16.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>

// 1. 标签顺序必须和训练 data.yaml 保持一致
static const char *project_labels[OBJ_CLASS_NUM] = {
    "helmet",    // 0: 有安全帽
    "no-helmet", // 1: 未戴安全帽
    "no-vest",   // 2: 未穿反光背心
    "person",    // 3: 人员
    "vest",      // 4: 穿反光背心
    "fire"       // 5: 火光/火警
};

inline static float clamp(float val, float min, float max) { 
    return val > min ? (val < max ? val : max) : min; 
}

inline static float sigmoid_if_needed(float val) {
    if (val >= 0.0f && val <= 1.0f) {
        return val;
    }
    if (val > 20.0f) return 1.0f;
    if (val < -20.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-val));
}

inline static float read_output_value(const rknn_output *output, int index, int elem_count) {
    if (output->size == (uint32_t)(elem_count * 2)) {
        const rknpu2::float16 *data = (const rknpu2::float16 *)output->buf;
        return (float)data[index];
    }
    const float *data = (const float *)output->buf;
    return data[index];
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

    if (outputs == NULL || outputs[0].buf == NULL || out_attr == NULL) {
        return 0;
    }

    int elem_count = out_attr[0].n_elems;
    if (elem_count <= 0 && outputs[0].size > 0) {
        elem_count = (int)(outputs[0].size / sizeof(float));
    }

    int num_anchors = 0;
    int num_classes = 0;
    int channel_count = 0;

    if (out_attr[0].n_dims >= 3 &&
        out_attr[0].dims[1] >= 5 &&
        out_attr[0].dims[1] <= 4 + OBJ_CLASS_NUM &&
        out_attr[0].dims[2] > 0) {
        channel_count = out_attr[0].dims[1];
        num_classes = channel_count - 4;
        num_anchors = out_attr[0].dims[2];
    } else {
        for (int ch = 4 + OBJ_CLASS_NUM; ch >= 5; --ch) {
            if (elem_count > 0 && elem_count % ch == 0) {
                channel_count = ch;
                num_classes = ch - 4;
                num_anchors = elem_count / ch;
                break;
            }
        }
    }

    if (num_anchors <= 0 || num_classes <= 0) {
        static int warned_bad_shape = 0;
        if (!warned_bad_shape) {
            fprintf(stderr, "[AI] unsupported output shape: elems=%d size=%u n_dims=%u\n",
                    elem_count, outputs[0].size, out_attr[0].n_dims);
            warned_bad_shape = 1;
        }
        return 0;
    }
    if (num_classes > OBJ_CLASS_NUM) {
        num_classes = OBJ_CLASS_NUM;
    }

    static int logged_shape = 0;
    if (!logged_shape) {
        fprintf(stderr, "[AI] postprocess output elems=%d channels=%d anchors=%d classes=%d\n",
                elem_count, channel_count, num_anchors, num_classes);
        logged_shape = 1;
    }

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
    float debug_best_prob = -1.0f;
    int debug_best_class = -1;
    int debug_best_anchor = -1;

    // 2. 遍历 8400 个锚点 (严格采用你昨日验证通过的 NCHW 提取法)
    for (int i = 0; i < num_anchors; ++i) {
        float max_class_prob = -1.0f;
        int max_class_id = -1;

        // 提取所有类别中的最高得分
        for (int c = 0; c < num_classes; ++c) {
            float prob = sigmoid_if_needed(read_output_value(&outputs[0],
                                                              (4 + c) * num_anchors + i,
                                                              elem_count));
            if (prob > max_class_prob) {
                max_class_prob = prob;
                max_class_id = c;
            }
        }
        if (max_class_prob > debug_best_prob) {
            debug_best_prob = max_class_prob;
            debug_best_class = max_class_id;
            debug_best_anchor = i;
        }

        // 3. 超过阈值才解析坐标 (推荐阈值 0.5)
        if (max_class_prob > conf_threshold) {
            float cx = read_output_value(&outputs[0], 0 * num_anchors + i, elem_count);
            float cy = read_output_value(&outputs[0], 1 * num_anchors + i, elem_count);
            float w  = read_output_value(&outputs[0], 2 * num_anchors + i, elem_count);
            float h  = read_output_value(&outputs[0], 3 * num_anchors + i, elem_count);

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

    (void)debug_best_prob;
    (void)debug_best_class;
    (void)debug_best_anchor;

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
    if (cls_id >= 0 && cls_id < OBJ_CLASS_NUM) return (char*)project_labels[cls_id];
    return (char*)"未知物体";
}
