#ifndef AUDIO_ALERT_H
#define AUDIO_ALERT_H

#include <stdint.h>
#include "rknn_worker.h"

void audio_alert_init(void);
void audio_alert_update(DetectSharedState *shared, int64_t now_ms);

// Send one alert command to /tmp/audio_fifo.
// 1=fire, 2=work zone 1, 3=work zone 2, 4=PPE warning.
void audio_alert_send_command(int command, int64_t now_ms);

void audio_alert_deinit(void);

#endif // AUDIO_ALERT_H
