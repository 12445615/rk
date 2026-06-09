#ifndef SAFETY_INTERLOCK_CLIENT_H
#define SAFETY_INTERLOCK_CLIENT_H

#include <pthread.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>

#define SAFETY_FRAME_HEAD 0xA5
#define SAFETY_FRAME_TAIL 0x5A
#define SAFETY_SENSOR_FRAME_CODE 0xB0
#define SAFETY_ACTUATOR_FRAME_CODE 0xC0

#define SAFETY_STM32_DEV_ENV "SAFETY_STM32_DEV"
#define SAFETY_STM32_BAUD_ENV "SAFETY_STM32_BAUD"
#define SAFETY_STM32_DEV_DEFAULT "/dev/ttyS9"
#define SAFETY_STM32_BAUD_DEFAULT 115200

#define SAFETY_ACT_DEVICE_POWER_ON 0x0001
#define SAFETY_ACT_FAN_ON 0x0002
#define SAFETY_ACT_ALARM_ON 0x0004
#define SAFETY_ACT_INTERLOCK 0x0008
#define SAFETY_ACT_RESET_WAIT 0x0010

typedef enum {
    SAFETY_CODE_SAFE = 0x00,
    SAFETY_CODE_PPE_DENY = 0x01,
    SAFETY_CODE_FIRE_WORK_ZONE = 0x02,
    SAFETY_CODE_FIRE_OUT_ZONE = 0x03,
    SAFETY_CODE_INTRUSION = 0x04,
    SAFETY_CODE_ENV_DANGER = 0x05,
    SAFETY_CODE_FAULT = 0x06,
    SAFETY_CODE_RESERVED = 0x07
} SafetyCode;

typedef enum {
    SAFETY_STM32_CODE_SAFE = 0x80,
    SAFETY_STM32_CODE_WORKING = 0x81,
    SAFETY_STM32_CODE_POWER_OFF = 0x82,
    SAFETY_STM32_CODE_INTERLOCK = 0x83,
    SAFETY_STM32_CODE_ENV_DANGER = 0x84,
    SAFETY_STM32_CODE_EMERGENCY_STOP = 0x85,
    SAFETY_STM32_CODE_FAULT = 0x86,
    SAFETY_STM32_CODE_RESET_WAIT = 0x87,
    SAFETY_STM32_CODE_RESET_OK = 0x88,
    SAFETY_STM32_CODE_FAN_ON = 0x89,
    SAFETY_STM32_CODE_ALARM_ON = 0x8A,
    SAFETY_STM32_CODE_START_REQUEST = 0x8B,
    SAFETY_STM32_CODE_RESET_REQUEST = 0x8C
} SafetyStm32Code;

typedef enum {
    SAFETY_RK_STATE_INIT = 0,
    SAFETY_RK_STATE_RUNNING = 1,
    SAFETY_RK_STATE_DEGRADED = 2,
    SAFETY_RK_STATE_FAULT = 3
} SafetyRkState;

typedef enum {
    SAFETY_PERMIT_DENY = 0,
    SAFETY_PERMIT_ALLOW = 1
} SafetyPermitDecision;

typedef enum {
    SAFETY_RISK_SAFE = 0,
    SAFETY_RISK_WARNING = 1,
    SAFETY_RISK_DANGER = 2,
    SAFETY_RISK_CRITICAL = 3
} SafetyRiskLevel;

typedef enum {
    SAFETY_RISK_TYPE_NONE = 0,
    SAFETY_RISK_TYPE_PPE = 1,
    SAFETY_RISK_TYPE_INTRUSION = 2,
    SAFETY_RISK_TYPE_WELDING = 3,
    SAFETY_RISK_TYPE_FIRE = 4,
    SAFETY_RISK_TYPE_ENV = 5,
    SAFETY_RISK_TYPE_EMERGENCY_STOP = 6,
    SAFETY_RISK_TYPE_STM32_LINK = 7,
    SAFETY_RISK_TYPE_STM32_FAULT = 8,
    SAFETY_RISK_TYPE_MITIGATION = 9
} SafetyRiskType;

typedef enum {
    SAFETY_VOICE_NONE = 0,
    SAFETY_VOICE_PPE_WARNING = 1,
    SAFETY_VOICE_INTRUSION_WARNING = 2,
    SAFETY_VOICE_FIRE_WARNING = 3,
    SAFETY_VOICE_ENV_WARNING = 4,
    SAFETY_VOICE_EMERGENCY_STOP = 5,
    SAFETY_VOICE_LINK_WARNING = 6,
    SAFETY_VOICE_WAIT_RESET = 7
} SafetyVoiceAction;

typedef enum {
    SAFETY_STM32_ACTION_KEEP = 0,
    SAFETY_STM32_ACTION_KEEP_POWER_OFF = 1,
    SAFETY_STM32_ACTION_CUT_POWER = 2,
    SAFETY_STM32_ACTION_FAN_ON = 3,
    SAFETY_STM32_ACTION_ALARM_ON = 4,
    SAFETY_STM32_ACTION_CUT_POWER_ALARM = 5,
    SAFETY_STM32_ACTION_CUT_POWER_FAN_ALARM = 6,
    SAFETY_STM32_ACTION_LOCKOUT_WAIT_RESET = 7
} SafetyStm32Action;

typedef enum {
    SAFETY_ACK_FAILED = 0,
    SAFETY_ACK_OK = 1,
    SAFETY_ACK_CRC_ERROR = 2,
    SAFETY_ACK_UNSUPPORTED_EVENT = 3,
    SAFETY_ACK_STORE_FAILED = 4
} SafetyAckResult;

typedef struct {
    int online;
    int actuator_feedback_valid;
    int expected_actuator_valid;
    uint8_t work_state;
    uint16_t stm32_flags;
    uint8_t fault_code;

    uint16_t smoke;
    uint16_t gas;
    int16_t temperature_x10;
    uint16_t actuator_flags;
    uint16_t expected_actuator_flags;

    uint8_t ai_detect_state;
    uint8_t ai_confidence;
    int ai_detect_valid;
    int64_t ai_detect_update_ms;

    uint16_t last_event_id;
    uint8_t last_event_type;
    uint8_t last_event_reason;
    uint32_t last_event_time_s;

    uint8_t last_fault_code;
    uint8_t last_fault_level;
    uint8_t last_fault_detail;
} SafetyStm32Snapshot;

typedef struct {
    pthread_t thread;
    pthread_mutex_t lock;
    volatile sig_atomic_t running;
    int started;
    int fd;
    char device[128];
    int baud;
    uint8_t tx_seq;
    int64_t start_mono_ms;
    int64_t last_rx_mono_ms;
    int64_t last_connect_attempt_ms;
    int64_t last_heartbeat_ms;
    uint8_t rx_buf[512];
    size_t rx_len;
    SafetyStm32Snapshot snapshot;
} SafetyInterlockClient;

int safety_client_init(SafetyInterlockClient *client,
                       const char *device,
                       int baud);
void safety_client_stop(SafetyInterlockClient *client);
int safety_client_get_snapshot(SafetyInterlockClient *client,
                               SafetyStm32Snapshot *snapshot_out);
int safety_client_update_ai_detect_state(SafetyInterlockClient *client,
                                         uint8_t ai_detect_state,
                                         uint8_t ai_confidence,
                                         int64_t update_ms);
int safety_client_send_ai_status(SafetyInterlockClient *client,
                                 uint16_t ai_flags,
                                 uint8_t ai_confidence);
int safety_client_send_fusion_decision(SafetyInterlockClient *client,
                                       uint8_t permit_decision,
                                       uint8_t risk_level,
                                       uint8_t risk_type,
                                       uint8_t voice_action,
                                       uint8_t stm32_action,
                                       uint8_t explain_code);

#endif
