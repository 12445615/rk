#include <stdio.h>

#include <stdlib.h>

#include <string.h>

#include <unistd.h>

#include <pthread.h>

#include <signal.h>

#include <errno.h>

#include <stdint.h>

#include <poll.h>

#include <sys/eventfd.h>

#include <sys/timerfd.h>



#include "sensor_modbus.h"

#include "aliyun_mqtt.h"

void zone_runtime_reset(void);

#include "local_store.h"

#include "safety_interlock_client.h"



#if defined(__has_include)

#if __has_include("MQTTClient.h")

#define HAVE_PAHO_MQTT 1

#endif

#endif



#ifndef HAVE_PAHO_MQTT

#define HAVE_PAHO_MQTT 0

#endif



#if HAVE_PAHO_MQTT

#include "MQTTClient.h"



#define ADDRESS "tcp://iot-06z00be8pk7p1uz.mqtt.iothub.aliyuncs.com:1883"

#define CLIENTID "k29ovUMboAH.0122|securemode=2,signmethod=hmacsha256,timestamp=1781166423351|"

#define USERNAME "0122&k29ovUMboAH"

#define PASSWORD "db3c4187b154d863a52a7eafd0e44ad7e094fd9704d571dfdbfa10278ed07073"

#define TOPIC "/sys/k29ovUMboAH/0122/thing/event/property/post"

#define ZONE_CONTROL_TOPIC "/k29ovUMboAH/0122/user/get"

#define HEARTBEAT_TOPIC "/k29ovUMboAH/0122/user/update"

#define ZONE_RESULT_TOPIC "/k29ovUMboAH/0122/user/update"

#define MQTT_REPORT_INTERVAL_SEC 3

#define MQTT_OFFLINE_FLUSH_BATCH 10

#define MQTT_AI_NONE_REPORT_INTERVAL_MS 60000

#define MQTT_ZONE_STATUS_REPEAT_MS 10000

#define MQTT_AI_STATE_FIRE_WORK_ZONE 5
#define MQTT_AI_STATE_FIRE_OUT_ZONE 6
#define MQTT_AI_STATE_INTRUSION 7

static int mqtt_ai_state_should_alarm(int ai_detect_state) {
    return ai_detect_state == MQTT_AI_STATE_FIRE_WORK_ZONE ||
           ai_detect_state == MQTT_AI_STATE_FIRE_OUT_ZONE ||
           ai_detect_state == MQTT_AI_STATE_INTRUSION;
}




typedef struct {

    int ppm;

    float temp;

    int alarm_status;

    int power_switch;

    int power_switch1;

    int power_switch2;

    int fan_status;

    float combustible_gas;

    float smoke_concentration;

    int ai_detect_valid;

    int ai_detect_state;

    int ai_confidence;

    int fire_confidence;

} SensorSnapshot;



extern volatile sig_atomic_t is_running;



static pthread_t g_mqtt_tid;

static int g_mqtt_stop_fd = -1;
static int g_mqtt_wakeup_fd = -1;

static int g_mqtt_started = 0;

static int g_mqtt_force_offline = 0;

static SafetyInterlockClient *g_safety_client = NULL;

static int g_last_reported_ai_state = -1;
static int g_last_reported_fire_value = 0;

static int64_t g_last_ai_none_report_ms = 0;

static pthread_mutex_t g_immediate_ai_lock = PTHREAD_MUTEX_INITIALIZER;

static int g_immediate_ai_valid = 0;

static int g_immediate_ai_state = 0;

static pthread_mutex_t g_zone_lock = PTHREAD_MUTEX_INITIALIZER;

static int g_zone_confirm_enabled = 0;

static int g_zone_blocked = 0;

static int g_zone_detect_request_pending = 0;

static int g_zone_detect_ok = 0;

static int64_t g_zone_last_cmd_ms = 0;

static int64_t g_zone_last_publish_request_ms = 0;



static int64_t current_time_ms(void) {

    struct timespec ts;



    if (clock_gettime(CLOCK_REALTIME, &ts) < 0) {

        return 0;

    }



    return (int64_t)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;

}

static int payload_contains(const char *payload, const char *needle) {
    return payload != NULL && needle != NULL && strstr(payload, needle) != NULL;
}

static void mqtt_wakeup_reporter_internal(void) {
    uint64_t one = 1;

    if (!g_mqtt_started || g_mqtt_wakeup_fd < 0) {
        return;
    }

    if (write(g_mqtt_wakeup_fd, &one, sizeof(one)) < 0 &&
        errno != EAGAIN) {
        perror("mqtt wake eventfd write");
    }
}

static void handle_zone_control_payload(const char *payload) {
    int confirm_enabled = 0;
    int blocked = 0;
    int detect_request = 0;
    int redetect_request = 0;

    if (payload_contains(payload, "zone_confirm_yes")) {
        confirm_enabled = 1;
        blocked = 0;
    } else if (payload_contains(payload, "zone_confirm_no") ||
               payload_contains(payload, "zone_timeout")) {
        confirm_enabled = 0;
        blocked = 1;
    } else if (payload_contains(payload, "zone_redetect_request")) {
        detect_request = 1;
        redetect_request = 1;
    } else if (payload_contains(payload, "zone_detect_request")) {
        detect_request = 1;
    } else if (payload_contains(payload, "zone_need_confirm")) {
        confirm_enabled = 0;
        blocked = 0;
    }

    pthread_mutex_lock(&g_zone_lock);
    if (payload_contains(payload, "zone_confirm_yes") ||
        payload_contains(payload, "zone_confirm_no") ||
        payload_contains(payload, "zone_timeout") ||
        payload_contains(payload, "zone_need_confirm")) {
        g_zone_confirm_enabled = confirm_enabled;
        g_zone_blocked = blocked;
    }
    if (detect_request) {
        g_zone_detect_request_pending = 1;
    }
    g_zone_last_cmd_ms = current_time_ms();
    pthread_mutex_unlock(&g_zone_lock);

    if (redetect_request) {
        zone_runtime_reset();
    }

    printf("[Aliyun] zone_control received: %s\n", payload ? payload : "");
    mqtt_wakeup_reporter_internal();
}

static int mqtt_message_arrived(void *context,
                                char *topic_name,
                                int topic_len,
                                MQTTClient_message *message) {
    (void)context;
    (void)topic_len;

    if (topic_name != NULL && message != NULL && message->payload != NULL) {
        char payload[512];
        int copy_len = message->payloadlen;
        if (copy_len >= (int)sizeof(payload)) {
            copy_len = (int)sizeof(payload) - 1;
        }
        memcpy(payload, message->payload, (size_t)copy_len);
        payload[copy_len] = '\0';

        if (strcmp(topic_name, ZONE_CONTROL_TOPIC) == 0) {
            handle_zone_control_payload(payload);
        }
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topic_name);
    return 1;
}



static void read_sensor_snapshot(SensorSnapshot *snapshot) {
    SafetyStm32Snapshot stm32_snapshot;
    int has_stm32_snapshot = 0;

    pthread_mutex_lock(&g_sensor_data.lock);

    snapshot->ppm = g_sensor_data.ppm;

    snapshot->temp = g_sensor_data.temp;

    snapshot->alarm_status = g_sensor_data.alarm_status;

    pthread_mutex_unlock(&g_sensor_data.lock);

    snapshot->power_switch = snapshot->alarm_status ? 0 : 1;
    snapshot->power_switch1 = snapshot->power_switch;
    snapshot->power_switch2 = snapshot->power_switch;
    snapshot->fan_status = snapshot->alarm_status ? 1 : 0;
    snapshot->combustible_gas = (float)snapshot->ppm;
    snapshot->smoke_concentration = (float)snapshot->ppm;
    snapshot->ai_detect_valid = 0;
    snapshot->ai_detect_state = 0;
    snapshot->ai_confidence = 0;
    snapshot->fire_confidence = 0;

    if (g_safety_client != NULL &&
        safety_client_get_snapshot(g_safety_client, &stm32_snapshot) == 0) {
        has_stm32_snapshot = 1;
    }

    if (has_stm32_snapshot && stm32_snapshot.actuator_feedback_valid) {
        snapshot->power_switch1 =
            (stm32_snapshot.actuator_flags & SAFETY_ACT_POWER1_ON) ? 1 : 0;
        snapshot->power_switch2 =
            (stm32_snapshot.actuator_flags & SAFETY_ACT_POWER2_ON) ? 1 : 0;
        snapshot->power_switch = (snapshot->power_switch1 && snapshot->power_switch2) ? 1 : 0;
        snapshot->fan_status =
            (stm32_snapshot.actuator_flags & SAFETY_ACT_FAN_ON) ? 1 : 0;
        snapshot->alarm_status =
            (stm32_snapshot.actuator_flags & SAFETY_ACT_ALARM_ON) ? 1 : 0;
    } else if (has_stm32_snapshot && stm32_snapshot.expected_actuator_valid) {
        snapshot->power_switch1 =
            (stm32_snapshot.expected_actuator_flags & SAFETY_ACT_POWER1_ON) ? 1 : 0;
        snapshot->power_switch2 =
            (stm32_snapshot.expected_actuator_flags & SAFETY_ACT_POWER2_ON) ? 1 : 0;
        snapshot->power_switch = (snapshot->power_switch1 && snapshot->power_switch2) ? 1 : 0;
        snapshot->fan_status =
            (stm32_snapshot.expected_actuator_flags & SAFETY_ACT_FAN_ON) ? 1 : 0;
        snapshot->alarm_status =
            (stm32_snapshot.expected_actuator_flags & SAFETY_ACT_ALARM_ON) ? 1 : 0;
    }

    if (has_stm32_snapshot && stm32_snapshot.online) {
        snapshot->combustible_gas = (float)stm32_snapshot.gas;
        snapshot->smoke_concentration = (float)stm32_snapshot.smoke;
        snapshot->temp = (float)stm32_snapshot.temperature_x10 / 10.0f;
        if (stm32_snapshot.fault_code != 0 ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_INTERLOCK ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_EMERGENCY_STOP ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_FAULT ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_RESET_WAIT ||
            (stm32_snapshot.actuator_flags & SAFETY_ACT_RESET_WAIT)) {
            snapshot->alarm_status = 1;
        }
    }
    if (has_stm32_snapshot && stm32_snapshot.ai_detect_valid) {
        snapshot->ai_detect_valid = 1;
        snapshot->ai_detect_state = stm32_snapshot.ai_detect_state;
        snapshot->ai_confidence = stm32_snapshot.ai_confidence;
        if (snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_WORK_ZONE ||
            snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_OUT_ZONE) {
            snapshot->fire_confidence = stm32_snapshot.ai_confidence;
        }
    }

    pthread_mutex_lock(&g_immediate_ai_lock);
    if (g_immediate_ai_valid) {
        snapshot->ai_detect_valid = 1;
        snapshot->ai_detect_state = g_immediate_ai_state;
        if (snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_WORK_ZONE ||
            snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_OUT_ZONE) {
            snapshot->fire_confidence = snapshot->ai_confidence;
        }
        g_immediate_ai_valid = 0;
    }
    pthread_mutex_unlock(&g_immediate_ai_lock);

    if (snapshot->ai_detect_valid &&
        mqtt_ai_state_should_alarm(snapshot->ai_detect_state)) {
        snapshot->alarm_status = 1;
    }

    if (snapshot->ai_detect_valid &&
        (snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_WORK_ZONE ||
         snapshot->ai_detect_state == MQTT_AI_STATE_FIRE_OUT_ZONE)) {
        snapshot->power_switch = 0;
        snapshot->power_switch1 = 0;
        snapshot->power_switch2 = 0;
        snapshot->fan_status = 0;
        snapshot->alarm_status = 1;
    }

}



static void build_debug_snapshot(SensorSnapshot *snapshot, int seq) {

    snapshot->ppm = 123 + seq;

    snapshot->temp = 25.5f + (float)seq * 0.1f;

    snapshot->alarm_status = seq % 2;

    snapshot->power_switch = snapshot->alarm_status ? 0 : 1;
    snapshot->power_switch1 = snapshot->power_switch;
    snapshot->power_switch2 = snapshot->power_switch;

    snapshot->fan_status = snapshot->alarm_status ? 1 : 0;

    snapshot->combustible_gas = (float)snapshot->ppm;

    snapshot->smoke_concentration = (float)snapshot->ppm;

    snapshot->ai_detect_valid = 1;

    snapshot->ai_detect_state = seq % 8;

    snapshot->ai_confidence = 80;
    snapshot->fire_confidence = 0;

}

static int build_sensor_payload(char *payload,

                                size_t payload_size,

                                int64_t created_at_ms,

                                const SensorSnapshot *snapshot) {

    int alarm_state = snapshot->alarm_status ? 1 : 0;
    int power_switch = snapshot->power_switch ? 1 : 0;
    int power_switch1 = snapshot->power_switch1 ? 1 : 0;
    int power_switch2 = snapshot->power_switch2 ? 1 : 0;
    int fan_status = snapshot->fan_status ? 1 : 0;
    int ai_detect_state = snapshot->ai_detect_valid ? snapshot->ai_detect_state : 0;
    int fire_value = (ai_detect_state == MQTT_AI_STATE_FIRE_WORK_ZONE ||
                      ai_detect_state == MQTT_AI_STATE_FIRE_OUT_ZONE) ?
                     snapshot->fire_confidence : 0;

    int len = snprintf(payload,

                       payload_size,

                       "{"

                       "\"id\":\"%lld\","

                       "\"version\":\"1.0\","

                       "\"params\":{"

                       "\"PowerSwitch\":%d,"
                       "\"PowerSwitch1\":%d,"
                       "\"PowerSwitch2\":%d,"

                       "\"Fanstatus\":%d,"

                       "\"CombustibleGasCheck\":%.2f,"

                       "\"AlarmState\":%d,"

                       "\"smokeconcentration\":%.2f,"
                       "\"temperature\":%.2f,"

                       "\"AiDetectState\":%d,"
                       "\"fire\":%d"

                       "},"

                       "\"method\":\"thing.event.property.post\""

                       "}",

                       (long long)created_at_ms,

                       power_switch,
                       power_switch1,
                       power_switch2,

                       fan_status,

                       (double)snapshot->combustible_gas,

                       alarm_state,

                       (double)snapshot->smoke_concentration,
                       (double)snapshot->temp,
                       ai_detect_state,
                       fire_value);
    g_last_reported_ai_state = ai_detect_state;
    g_last_reported_fire_value = fire_value;
    if (ai_detect_state == 0) {
        g_last_ai_none_report_ms = created_at_ms;
    } else {
        printf("[闃块噷浜慮 AiDetectState=%d will be reported\n", ai_detect_state);
    }



    if (len < 0 || (size_t)len >= payload_size) {

        return ENOSPC;

    }



    return 0;

}



static int publish_payload(MQTTClient client, const char *topic, const char *payload) {

    MQTTClient_message pubmsg = MQTTClient_message_initializer;

    MQTTClient_deliveryToken token;

    int rc;



    pubmsg.payload = (void *)payload;

    pubmsg.payloadlen = (int)strlen(payload);

    pubmsg.qos = 0;

    pubmsg.retained = 0;



    rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token);

    if (rc != MQTTCLIENT_SUCCESS) {

        return rc;

    }



    rc = MQTTClient_waitForCompletion(client, token, 1000L);

    return rc;

}



static int mqtt_connect_client(MQTTClient client) {

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    int rc;



    conn_opts.keepAliveInterval = 60;

    conn_opts.cleansession = 1;

    conn_opts.username = USERNAME;

    conn_opts.password = PASSWORD;



    rc = MQTTClient_connect(client, &conn_opts);

    if (rc == MQTTCLIENT_SUCCESS) {

        printf("[闃块噷浜慮 宸茶繛鎺ワ紝寮€濮嬩笂浼犲苟琛ュ彂绂荤嚎鏁版嵁銆俓n");

        rc = MQTTClient_subscribe(client, ZONE_CONTROL_TOPIC, 0);
        if (rc == MQTTCLIENT_SUCCESS) {
            printf("[Aliyun] subscribed topic=%s\n", ZONE_CONTROL_TOPIC);
        } else {
            printf("[Aliyun] subscribe failed topic=%s rc=%d\n", ZONE_CONTROL_TOPIC, rc);
        }

        return 0;

    }



    printf("[闃块噷浜戦敊璇痌 杩炴帴澶辫触锛岃繑鍥炵爜: %d\n", rc);

    return rc;

}



static void mqtt_disconnect_client(MQTTClient client, int *is_connected) {

    if (*is_connected) {

        MQTTClient_disconnect(client, 1000);

        *is_connected = 0;

    }

}



static int enqueue_offline_record(LocalStore *store,

                                  int store_ready,

                                  int64_t created_at_ms,

                                  const SensorSnapshot *snapshot,

                                  const char *payload) {

    int rc;



    if (!store_ready) {

        fprintf(stderr, "[绂荤嚎缂撳瓨] 涓嶅彲鐢紝褰撳墠鏁版嵁鏃犳硶钀界洏銆俓n");

        return ENOSYS;

    }



    rc = local_store_enqueue(store,

                             created_at_ms,

                             TOPIC,

                             payload,

                             snapshot->ppm,

                             snapshot->temp,

                             0.0,

                             snapshot->alarm_status,

                             "",

                             "");

    if (rc != 0) {

        fprintf(stderr, "[绂荤嚎缂撳瓨] 鍐欏叆澶辫触: %d\n", rc);

        return rc;

    }



    printf("[绂荤嚎缂撳瓨] 宸蹭繚瀛樹竴鏉＄绾胯褰曘€俓n");

    return 0;

}



static int flush_offline_records(LocalStore *store, MQTTClient client) {

    LocalStoreRecord records[MQTT_OFFLINE_FLUSH_BATCH];

    int rc;

    int count = 0;



    rc = local_store_fetch_batch(store, records, MQTT_OFFLINE_FLUSH_BATCH, &count);

    if (rc != 0) {

        fprintf(stderr, "[绂荤嚎缂撳瓨] 璇诲彇寰呰ˉ鍙戣褰曞け璐? %d\n", rc);

        return 0;

    }



    for (int i = 0; i < count; ++i) {

        rc = publish_payload(client, records[i].topic, records[i].payload);

        if (rc != MQTTCLIENT_SUCCESS) {

            printf("[绂荤嚎琛ュ彂] 琛ュ彂澶辫触锛岃繑鍥炵爜: %d\n", rc);

            return -1;

        }



        rc = local_store_delete(store, records[i].id);

        if (rc != 0) {

            fprintf(stderr, "[绂荤嚎缂撳瓨] 鍒犻櫎宸茶ˉ鍙戣褰曞け璐?id=%lld, err=%d\n",

                    (long long)records[i].id, rc);

            return 0;

        }



        printf("[绂荤嚎琛ュ彂] 鎴愬姛琛ュ彂 id=%lld\n", (long long)records[i].id);

    }



    return 0;

}



static int send_or_enqueue_payload(LocalStore *store,

                                   int store_ready,

                                   MQTTClient client,

                                   int *mqtt_connected,

                                   int64_t created_at_ms,

                                   const SensorSnapshot *snapshot,

                                   const char *topic,

                                   const char *payload) {

    int rc;



    if (g_mqtt_force_offline) {

        printf("[MQTTDebug] force_offline=1锛屽綋鍓嶆秷鎭洿鎺ュ啓鍏ョ绾跨紦瀛樸€俓n");

        return enqueue_offline_record(store, store_ready, created_at_ms, snapshot, payload);

    }



    if (client != NULL && !*mqtt_connected) {

        if (mqtt_connect_client(client) == 0) {

            *mqtt_connected = 1;

        }

    }



    if (*mqtt_connected) {

        rc = publish_payload(client, topic, payload);

        if (rc == MQTTCLIENT_SUCCESS) {

           // printf("[闃块噷浜慮 褰撳墠鏁版嵁涓婃姤鎴愬姛銆俓n");

            return 0;

        }



        printf("[闃块噷浜戦敊璇痌 褰撳墠鏁版嵁涓婃姤澶辫触锛岃繑鍥炵爜: %d\n", rc);

        mqtt_disconnect_client(client, mqtt_connected);

    }



    return enqueue_offline_record(store, store_ready, created_at_ms, snapshot, payload);

}

static void publish_heartbeat(MQTTClient client,
                              int mqtt_connected,
                              int64_t created_at_ms) {
    SafetyStm32Snapshot stm32_snapshot;
    int stm32_online = 0;
    int alarm_state = mqtt_ai_state_should_alarm(g_last_reported_ai_state) ? 1 : 0;
    int rc;
    char payload[256];

    if (!mqtt_connected || client == NULL) {
        return;
    }

    if (g_safety_client != NULL &&
        safety_client_get_snapshot(g_safety_client, &stm32_snapshot) == 0) {
        stm32_online = stm32_snapshot.online ? 1 : 0;
        if (stm32_snapshot.fault_code != 0 ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_INTERLOCK ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_EMERGENCY_STOP ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_FAULT ||
            stm32_snapshot.last_event_type == SAFETY_STM32_CODE_RESET_WAIT ||
            (stm32_snapshot.actuator_flags & SAFETY_ACT_ALARM_ON) ||
            (stm32_snapshot.actuator_flags & SAFETY_ACT_RESET_WAIT)) {
            alarm_state = 1;
        }
    }

    snprintf(payload, sizeof(payload),
             "{\"cmd\":\"heartbeat\",\"online\":1,"
             "\"stm32_online\":%d,\"AiDetectState\":%d,\"AlarmState\":%d,\"fire\":%d,\"ts\":%lld}",
             stm32_online,
             g_last_reported_ai_state,
             alarm_state,
             g_last_reported_fire_value,
             (long long)created_at_ms);

    rc = publish_payload(client, HEARTBEAT_TOPIC, payload);
    if (rc == MQTTCLIENT_SUCCESS) {
        printf("[Aliyun] heartbeat published topic=%s stm32_online=%d\n",
               HEARTBEAT_TOPIC,
               stm32_online);
    } else {
        printf("[Aliyun] heartbeat publish failed topic=%s rc=%d\n",
               HEARTBEAT_TOPIC,
               rc);
    }
}

static void publish_zone_detect_result_if_needed(MQTTClient client,
                                                 int mqtt_connected,
                                                 int64_t created_at_ms) {
    int pending = 0;
    char payload[256];

    if (!mqtt_connected || client == NULL) {
        return;
    }

    pthread_mutex_lock(&g_zone_lock);
    pending = g_zone_detect_request_pending;
    g_zone_detect_request_pending = 0;
    pthread_mutex_unlock(&g_zone_lock);

    if (!pending) {
        return;
    }

    int ok;

    pthread_mutex_lock(&g_zone_lock);
    ok = g_zone_detect_ok;
    pthread_mutex_unlock(&g_zone_lock);

    snprintf(payload, sizeof(payload),
             "{\"cmd\":\"zone_detect_result\",\"ok\":%d,"
             "\"msg\":\"%s\",\"ts\":%lld}",
             ok,
             ok ? "zone detected" : "zone not detected",
             (long long)created_at_ms);

    int rc = publish_payload(client, ZONE_RESULT_TOPIC, payload);
    if (rc == MQTTCLIENT_SUCCESS) {
        printf("[Aliyun] zone result published ok=%d topic=%s\n",
               ok,
               ZONE_RESULT_TOPIC);
    } else {
        printf("[Aliyun] zone result publish failed ok=%d topic=%s rc=%d\n",
               ok,
               ZONE_RESULT_TOPIC,
               rc);
    }
}



static int mqtt_debug_enqueue_fake_record_internal(const char *root_dir,

                                                   const char *payload_in,

                                                   int seq) {

    LocalStore store;

    MQTTClient client = NULL;

    SensorSnapshot snapshot;

    char payload[LOCAL_STORE_MAX_PAYLOAD_LEN];

    int64_t created_at_ms;

    int mqtt_connected = 0;

    int store_ready = 0;

    int rc;



    memset(&store, 0, sizeof(store));

    build_debug_snapshot(&snapshot, seq);

    created_at_ms = current_time_ms();



    rc = local_store_open(&store, root_dir);

    if (rc != 0) {

        printf("[MQTTDebug] local_store_open failed: %d\n", rc);

        return rc;

    }

    store_ready = 1;



    if (payload_in != NULL && payload_in[0] != '\0') {

        rc = snprintf(payload, sizeof(payload), "%s", payload_in);

        if (rc < 0 || (size_t)rc >= sizeof(payload)) {

            local_store_close(&store);

            return ENOSPC;

        }

    } else {

        rc = build_sensor_payload(payload, sizeof(payload), created_at_ms, &snapshot);

        if (rc != 0) {

            local_store_close(&store);

            return rc;

        }

    }



    if (!g_mqtt_force_offline) {

        rc = MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

        if (rc != MQTTCLIENT_SUCCESS) {

            printf("[MQTTDebug] MQTTClient_create failed: %d\n", rc);

            client = NULL;

        }

    }



    rc = send_or_enqueue_payload(&store,

                                 store_ready,

                                 client,

                                 &mqtt_connected,

                                 created_at_ms,

                                 &snapshot,

                                 TOPIC,

                                 payload);



    mqtt_disconnect_client(client, &mqtt_connected);

    if (client != NULL) {

        MQTTClient_destroy(&client);

    }

    local_store_close(&store);

    return rc;

}



static int mqtt_debug_flush_offline_once_internal(const char *root_dir) {

    LocalStore store;

    MQTTClient client = NULL;

    int mqtt_connected = 0;

    int rc;



    memset(&store, 0, sizeof(store));



    if (g_mqtt_force_offline) {

        printf("[MQTTDebug] force_offline=1锛岃烦杩囪ˉ鍙戙€俓n");

        return 0;

    }



    rc = local_store_open(&store, root_dir);

    if (rc != 0) {

        printf("[MQTTDebug] local_store_open failed: %d\n", rc);

        return rc;

    }



    rc = MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

    if (rc != MQTTCLIENT_SUCCESS) {

        printf("[MQTTDebug] MQTTClient_create failed: %d\n", rc);

        local_store_close(&store);

        return EIO;

    }



    rc = mqtt_connect_client(client);

    if (rc == 0) {

        mqtt_connected = 1;

        rc = flush_offline_records(&store, client);

        if (rc < 0) {

            rc = EIO;

        } else {

            rc = 0;

        }

    } else {

        rc = EIO;

    }



    mqtt_disconnect_client(client, &mqtt_connected);

    if (client != NULL) {

        MQTTClient_destroy(&client);

    }

    local_store_close(&store);

    return rc;

}



static void *mqtt_thread_func(void *arg) {

    (void)arg;



    MQTTClient client = NULL;

    LocalStore store;

    struct pollfd fds[3];

    int store_ready = 0;

    int mqtt_connected = 0;

    int timer_fd = -1;

    int rc;



    memset(&store, 0, sizeof(store));



    printf("[闃块噷浜慮 MQTT 绾跨▼鍚姩锛屽噯澶囪繛鎺?..\n");



    rc = local_store_open(&store, NULL);

    if (rc != 0) {

        fprintf(stderr, "[绂荤嚎缂撳瓨] 鍒濆鍖栧け璐? %d\n", rc);

    } else {

        store_ready = 1;

        printf("[绂荤嚎缂撳瓨] SQLite 宸插氨缁? %s\n", store.db_path);

    }



    rc = MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

    if (rc != MQTTCLIENT_SUCCESS) {

        fprintf(stderr, "[闃块噷浜戦敊璇痌 鍒涘缓瀹㈡埛绔け璐ワ紝杩斿洖鐮? %d\n", rc);

        client = NULL;

    } else {

        MQTTClient_setCallbacks(client, NULL, NULL, mqtt_message_arrived, NULL);

    }



    timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);

    if (timer_fd < 0) {

        perror("timerfd_create");

        goto cleanup;

    }



    struct itimerspec its;

    memset(&its, 0, sizeof(its));

    its.it_value.tv_sec = 1;

    its.it_interval.tv_sec = MQTT_REPORT_INTERVAL_SEC;



    if (timerfd_settime(timer_fd, 0, &its, NULL) < 0) {

        perror("timerfd_settime");

        goto cleanup;

    }



    printf("[闃块噷浜慮 瀹氭椂涓婃姤宸插惎鍔紝姣?d绉掕Е鍙戜竴娆°€俓n", MQTT_REPORT_INTERVAL_SEC);



    fds[0].fd = timer_fd;

    fds[0].events = POLLIN;

    fds[1].fd = g_mqtt_stop_fd;

    fds[1].events = POLLIN;

    fds[2].fd = g_mqtt_wakeup_fd;

    fds[2].events = POLLIN;



    while (1) {

        rc = poll(fds, 3, -1);

        if (rc < 0) {

            if (errno == EINTR) {

                continue;

            }

            perror("poll");

            break;

        }



        if (fds[1].revents & POLLIN) {

            uint64_t stop_val;



            if (read(g_mqtt_stop_fd, &stop_val, sizeof(stop_val)) < 0 && errno != EAGAIN) {

                perror("eventfd read");

            }

            break;

        }



        if ((fds[0].revents & POLLIN) || (fds[2].revents & POLLIN)) {

            uint64_t expirations = 0;

            SensorSnapshot snapshot;

            int64_t created_at_ms;

            char payload[LOCAL_STORE_MAX_PAYLOAD_LEN];



            if ((fds[0].revents & POLLIN) &&
                read(timer_fd, &expirations, sizeof(expirations)) != (ssize_t)sizeof(expirations)) {

                if (errno != EINTR) {

                    perror("timerfd read");

                }

                continue;

            }

            if (fds[2].revents & POLLIN) {
                while (read(g_mqtt_wakeup_fd, &expirations, sizeof(expirations)) == (ssize_t)sizeof(expirations)) {
                }
                if (errno != EAGAIN && errno != EINTR) {
                    perror("mqtt wake eventfd read");
                }
            }



            if (!is_running) {

                break;

            }



            read_sensor_snapshot(&snapshot);

            created_at_ms = current_time_ms();

            rc = build_sensor_payload(payload, sizeof(payload), created_at_ms, &snapshot);

            if (rc != 0) {

                fprintf(stderr, "[闃块噷浜戦敊璇痌 鏋勯€?payload 澶辫触: %d\n", rc);

                continue;

            }



            if (!g_mqtt_force_offline && client != NULL && !mqtt_connected) {

                if (mqtt_connect_client(client) == 0) {

                    mqtt_connected = 1;

                }

            }



            if (mqtt_connected && store_ready) {

                rc = flush_offline_records(&store, client);

                if (rc < 0) {

                    printf("[闃块噷浜慮 琛ュ彂杩囩▼涓帀绾匡紝褰撳墠鏁版嵁杞叆绂荤嚎缂撳瓨銆俓n");

                    mqtt_disconnect_client(client, &mqtt_connected);

                }

            }



            rc = send_or_enqueue_payload(&store,

                                         store_ready,

                                         client,

                                         &mqtt_connected,

                                         created_at_ms,

                                         &snapshot,

                                         TOPIC,

                                         payload);

            if (rc != 0) {

                fprintf(stderr, "[闃块噷浜戦敊璇痌 褰撳墠鏁版嵁鍐欏叆鍙戦€侀摼璺け璐? %d\n", rc);

            }

            if (mqtt_connected) {
                publish_heartbeat(client, mqtt_connected, created_at_ms);
                publish_zone_detect_result_if_needed(client,
                                                     mqtt_connected,
                                                     created_at_ms);
            }

        }

    }



cleanup:

    if (timer_fd >= 0) {

        close(timer_fd);

    }

    mqtt_disconnect_client(client, &mqtt_connected);

    if (client != NULL) {

        MQTTClient_destroy(&client);

    }

    if (store_ready) {

        local_store_close(&store);

    }

    return NULL;

}



int start_mqtt_reporter(void) {

    int rc;



    if (g_mqtt_started) {

        return 0;

    }



    g_mqtt_stop_fd = eventfd(0, EFD_CLOEXEC);

    if (g_mqtt_stop_fd < 0) {

        return errno;

    }

    g_mqtt_wakeup_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);

    if (g_mqtt_wakeup_fd < 0) {

        rc = errno;
        close(g_mqtt_stop_fd);
        g_mqtt_stop_fd = -1;
        return rc;

    }



    rc = pthread_create(&g_mqtt_tid, NULL, mqtt_thread_func, NULL);

    if (rc != 0) {

        close(g_mqtt_stop_fd);

        g_mqtt_stop_fd = -1;
        close(g_mqtt_wakeup_fd);
        g_mqtt_wakeup_fd = -1;

        return rc;

    }



    g_mqtt_started = 1;

    return 0;

}

void set_mqtt_safety_client(SafetyInterlockClient *client) {

    g_safety_client = client;

}



void stop_mqtt_reporter(void) {

    uint64_t one = 1;



    if (!g_mqtt_started) {

        return;

    }



    if (write(g_mqtt_stop_fd, &one, sizeof(one)) < 0 && errno != EAGAIN) {

        perror("eventfd write");

    }



    pthread_join(g_mqtt_tid, NULL);

    close(g_mqtt_stop_fd);
    close(g_mqtt_wakeup_fd);



    g_mqtt_stop_fd = -1;
    g_mqtt_wakeup_fd = -1;

    g_mqtt_started = 0;

}

void mqtt_request_immediate_ai_report(uint8_t ai_detect_state) {

    if (ai_detect_state == 0) {

        return;

    }

    pthread_mutex_lock(&g_immediate_ai_lock);
    g_immediate_ai_valid = 1;
    g_immediate_ai_state = ai_detect_state;
    pthread_mutex_unlock(&g_immediate_ai_lock);

    mqtt_request_immediate_report();
}

void mqtt_request_immediate_report(void) {

    uint64_t one = 1;

    if (!g_mqtt_started || g_mqtt_wakeup_fd < 0) {

        return;

    }

    if (write(g_mqtt_wakeup_fd, &one, sizeof(one)) < 0 &&
        errno != EAGAIN) {

        perror("mqtt wake eventfd write");

    }

}

void mqtt_update_zone_detection_result(int ok) {
    int changed;
    int64_t now_ms;
    int should_publish = 0;

    pthread_mutex_lock(&g_zone_lock);
    ok = ok ? 1 : 0;
    now_ms = current_time_ms();
    changed = (g_zone_detect_ok != ok);
    g_zone_detect_ok = ok;
    if (changed ||
        g_zone_last_publish_request_ms == 0 ||
        now_ms - g_zone_last_publish_request_ms >= MQTT_ZONE_STATUS_REPEAT_MS) {
        g_zone_detect_request_pending = 1;
        g_zone_last_publish_request_ms = now_ms;
        should_publish = 1;
    }
    pthread_mutex_unlock(&g_zone_lock);

    if (should_publish) {
        mqtt_wakeup_reporter_internal();
    }
}

int mqtt_get_zone_confirm_enabled(void) {
    int enabled;
    pthread_mutex_lock(&g_zone_lock);
    enabled = g_zone_confirm_enabled;
    pthread_mutex_unlock(&g_zone_lock);
    return enabled;
}

int mqtt_get_zone_blocked(void) {
    int blocked;
    pthread_mutex_lock(&g_zone_lock);
    blocked = g_zone_blocked;
    pthread_mutex_unlock(&g_zone_lock);
    return blocked;
}



int mqtt_debug_set_force_offline(int enabled) {

    g_mqtt_force_offline = enabled ? 1 : 0;

    printf("[MQTTDebug] force_offline=%d\n", g_mqtt_force_offline);

    return 0;

}



int mqtt_debug_enqueue_fake_record(const char *root_dir, const char *payload) {

    return mqtt_debug_enqueue_fake_record_internal(root_dir, payload, 1);

}



int mqtt_debug_flush_offline_once(const char *root_dir) {

    return mqtt_debug_flush_offline_once_internal(root_dir);

}



int mqtt_debug_run_end_to_end_test(const char *root_dir) {

    int rc;



    printf("[MQTTDebug] ===== end-to-end test start =====\n");



    rc = mqtt_debug_set_force_offline(1);

    if (rc != 0) {

        return rc;

    }



    rc = mqtt_debug_enqueue_fake_record_internal(root_dir, NULL, 1);

    if (rc != 0) {

        return rc;

    }



    rc = mqtt_debug_enqueue_fake_record_internal(root_dir, NULL, 2);

    if (rc != 0) {

        return rc;

    }



    rc = local_store_debug_dump(root_dir, 10);

    if (rc != 0) {

        return rc;

    }



    rc = mqtt_debug_set_force_offline(0);

    if (rc != 0) {

        return rc;

    }



    rc = mqtt_debug_flush_offline_once_internal(root_dir);

    if (rc != 0) {

        return rc;

    }



    rc = local_store_debug_dump(root_dir, 10);

    if (rc != 0) {

        return rc;

    }



    printf("[MQTTDebug] ===== end-to-end test done =====\n");

    return 0;

}

#else

int start_mqtt_reporter(void) {

    return ENOSYS;

}

void set_mqtt_safety_client(SafetyInterlockClient *client) {

    (void)client;

}



void stop_mqtt_reporter(void) {

}

void mqtt_request_immediate_report(void) {

}

void mqtt_request_immediate_ai_report(uint8_t ai_detect_state) {

    (void)ai_detect_state;

}

void mqtt_update_zone_detection_result(int ok) {
    pthread_mutex_lock(&g_zone_lock);
    g_zone_detect_ok = ok ? 1 : 0;
    pthread_mutex_unlock(&g_zone_lock);
}

int mqtt_get_zone_confirm_enabled(void) {

    return 0;

}

int mqtt_get_zone_blocked(void) {

    return 0;

}



int mqtt_debug_set_force_offline(int enabled) {

    (void)enabled;

    return ENOSYS;

}



int mqtt_debug_enqueue_fake_record(const char *root_dir, const char *payload) {

    (void)root_dir;

    (void)payload;

    return ENOSYS;

}



int mqtt_debug_flush_offline_once(const char *root_dir) {

    (void)root_dir;

    return ENOSYS;

}



int mqtt_debug_run_end_to_end_test(const char *root_dir) {

    (void)root_dir;

    return ENOSYS;

}

#endif
