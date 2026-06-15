#ifndef ALIYUN_MQTT_H
#define ALIYUN_MQTT_H

#include "safety_interlock_client.h"

int start_mqtt_reporter(void);
void stop_mqtt_reporter(void);
void set_mqtt_safety_client(SafetyInterlockClient *client);
void mqtt_request_immediate_report(void);
void mqtt_request_immediate_ai_report(uint8_t ai_detect_state);
int mqtt_get_zone_confirm_enabled(void);
int mqtt_get_zone_blocked(void);
void mqtt_update_zone_detection_result(int ok);

int mqtt_debug_set_force_offline(int enabled);
int mqtt_debug_enqueue_fake_record(const char *root_dir, const char *payload);
int mqtt_debug_flush_offline_once(const char *root_dir);
int mqtt_debug_run_end_to_end_test(const char *root_dir);

#endif
