#ifndef AUDIO_ALERT_H
#define AUDIO_ALERT_H

#include <stdint.h>
#include "rknn_worker.h" 

// 初始化音频专属线程
void audio_alert_init(void);

// 音频状态检测（在主循环中无阻塞调用）
void audio_alert_update(DetectSharedState *shared, int64_t now_ms);

// 退出程序时清理线程
void audio_alert_deinit(void);

#endif // AUDIO_ALERT_H