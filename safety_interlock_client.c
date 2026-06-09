#include "safety_interlock_client.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define SAFETY_HEARTBEAT_INTERVAL_MS 500
#define SAFETY_RECONNECT_INTERVAL_MS 1000
#define SAFETY_STM32_OFFLINE_MS 2000
#define SAFETY_SIMPLE_FRAME_SIZE 3
#define SAFETY_SENSOR_FRAME_SIZE 9
#define SAFETY_ACTUATOR_FRAME_SIZE 6

static int64_t safety_mono_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static uint16_t safety_get_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}


static speed_t safety_baud_to_speed(int baud) {
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
    }
}

static int safety_configure_serial(int fd, int baud) {
    struct termios tty;
    speed_t speed = safety_baud_to_speed(baud);

    if (tcgetattr(fd, &tty) != 0) {
        return errno;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return errno;
    }

    tcflush(fd, TCIOFLUSH);
    return 0;
}

static int safety_open_serial(SafetyInterlockClient *client) {
    int fd;
    int rc;

    fd = open(client->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return errno;
    }

    rc = safety_configure_serial(fd, client->baud);
    if (rc != 0) {
        close(fd);
        return rc;
    }

    client->fd = fd;
    client->rx_len = 0;
    printf("[Safety] STM32 serial opened: %s baud=%d\n",
           client->device, client->baud);
    return 0;
}

static void safety_close_serial(SafetyInterlockClient *client) {
    if (client->fd >= 0) {
        close(client->fd);
        client->fd = -1;
        printf("[Safety] STM32 serial closed\n");
    }
}

static int safety_send_simple_code(SafetyInterlockClient *client,
                                   uint8_t code) {
    uint8_t frame[SAFETY_SIMPLE_FRAME_SIZE];
    ssize_t written;

    if (client == NULL || client->fd < 0) {
        return ENOTCONN;
    }

    frame[0] = SAFETY_FRAME_HEAD;
    frame[1] = code;
    frame[2] = SAFETY_FRAME_TAIL;

    written = write(client->fd, frame, sizeof(frame));
    if (written < 0) {
        return errno;
    }
    if ((size_t)written != sizeof(frame)) {
        return EIO;
    }

    return 0;
}

static uint16_t safety_expected_flags_from_code(uint8_t code) {
    switch (code) {
    case SAFETY_CODE_SAFE:
    case SAFETY_CODE_FIRE_WORK_ZONE:
        return SAFETY_ACT_DEVICE_POWER_ON;
    case SAFETY_CODE_FIRE_OUT_ZONE:
    case SAFETY_CODE_ENV_DANGER:
        return (uint16_t)(SAFETY_ACT_FAN_ON | SAFETY_ACT_ALARM_ON);
    case SAFETY_CODE_PPE_DENY:
    case SAFETY_CODE_INTRUSION:
    case SAFETY_CODE_FAULT:
    default:
        return SAFETY_ACT_ALARM_ON;
    }
}

static void safety_update_expected_actuator(SafetyInterlockClient *client,
                                            uint8_t code) {
    if (client == NULL) {
        return;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.expected_actuator_valid = 1;
    client->snapshot.expected_actuator_flags = safety_expected_flags_from_code(code);
    pthread_mutex_unlock(&client->lock);
}

static void safety_handle_stm32_code(SafetyInterlockClient *client,
                                     uint8_t code) {
    uint16_t flags = 0;

    pthread_mutex_lock(&client->lock);
    client->snapshot.online = 1;

    switch (code) {
    case SAFETY_STM32_CODE_SAFE:
        client->snapshot.work_state = 0;
        client->snapshot.fault_code = 0;
        client->snapshot.actuator_flags = 0;
        break;
    case SAFETY_STM32_CODE_WORKING:
        client->snapshot.work_state = 1;
        client->snapshot.actuator_flags |= SAFETY_ACT_DEVICE_POWER_ON;
        client->snapshot.actuator_flags &= (uint16_t)~(SAFETY_ACT_FAN_ON |
                                                       SAFETY_ACT_ALARM_ON |
                                                       SAFETY_ACT_INTERLOCK |
                                                       SAFETY_ACT_RESET_WAIT);
        break;
    case SAFETY_STM32_CODE_POWER_OFF:
        client->snapshot.work_state = 0;
        client->snapshot.actuator_flags &= (uint16_t)~SAFETY_ACT_DEVICE_POWER_ON;
        client->snapshot.actuator_flags &= (uint16_t)~(SAFETY_ACT_FAN_ON |
                                                       SAFETY_ACT_ALARM_ON);
        break;
    case SAFETY_STM32_CODE_INTERLOCK:
        flags |= SAFETY_ACT_INTERLOCK;
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_ENV_DANGER:
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_EMERGENCY_STOP:
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_FAULT:
        client->snapshot.fault_code = 1;
        client->snapshot.last_fault_code = code;
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_RESET_WAIT:
        flags |= SAFETY_ACT_RESET_WAIT;
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_RESET_OK:
        client->snapshot.fault_code = 0;
        client->snapshot.last_event_type = code;
        break;
    case SAFETY_STM32_CODE_FAN_ON:
        flags |= SAFETY_ACT_FAN_ON;
        break;
    case SAFETY_STM32_CODE_ALARM_ON:
        flags |= SAFETY_ACT_ALARM_ON;
        break;
    case SAFETY_STM32_CODE_START_REQUEST:
    case SAFETY_STM32_CODE_RESET_REQUEST:
        client->snapshot.last_event_type = code;
        break;
    default:
        break;
    }

    if (flags != 0) {
        client->snapshot.actuator_flags |= flags;
    }
    client->last_rx_mono_ms = safety_mono_now_ms();
    pthread_mutex_unlock(&client->lock);
}

static void safety_handle_sensor_values(SafetyInterlockClient *client,
                                        const uint8_t *payload) {
    pthread_mutex_lock(&client->lock);
    client->snapshot.online = 1;
    client->snapshot.smoke = safety_get_le16(&payload[0]);
    client->snapshot.gas = safety_get_le16(&payload[2]);
    client->snapshot.temperature_x10 = (int16_t)safety_get_le16(&payload[4]);
    client->last_rx_mono_ms = safety_mono_now_ms();
    pthread_mutex_unlock(&client->lock);
}

static void safety_handle_actuator_values(SafetyInterlockClient *client,
                                          const uint8_t *payload) {
    uint16_t flags = 0;

    if (payload[0]) {
        flags |= SAFETY_ACT_DEVICE_POWER_ON;
    }
    if (payload[1]) {
        flags |= SAFETY_ACT_FAN_ON;
    }
    if (payload[2]) {
        flags |= SAFETY_ACT_ALARM_ON;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.online = 1;
    client->snapshot.actuator_feedback_valid = 1;
    client->snapshot.actuator_flags &= (uint16_t)~(SAFETY_ACT_DEVICE_POWER_ON |
                                                  SAFETY_ACT_FAN_ON |
                                                  SAFETY_ACT_ALARM_ON);
    client->snapshot.actuator_flags |= flags;
    client->snapshot.work_state =
        (flags & SAFETY_ACT_DEVICE_POWER_ON) ? 1 : 0;
    client->last_rx_mono_ms = safety_mono_now_ms();
    pthread_mutex_unlock(&client->lock);
}

static void safety_parse_rx(SafetyInterlockClient *client) {
    size_t pos = 0;

    while (client->rx_len - pos >= SAFETY_SIMPLE_FRAME_SIZE) {
        if (client->rx_buf[pos] != SAFETY_FRAME_HEAD) {
            pos++;
            continue;
        }

        if (client->rx_buf[pos + 1] == SAFETY_SENSOR_FRAME_CODE) {
            if (client->rx_len - pos < SAFETY_SENSOR_FRAME_SIZE) {
                break;
            }
            if (client->rx_buf[pos + SAFETY_SENSOR_FRAME_SIZE - 1] == SAFETY_FRAME_TAIL) {
                safety_handle_sensor_values(client, &client->rx_buf[pos + 2]);
                pos += SAFETY_SENSOR_FRAME_SIZE;
                continue;
            }
            pos++;
            continue;
        }

        if (client->rx_buf[pos + 1] == SAFETY_ACTUATOR_FRAME_CODE) {
            if (client->rx_len - pos < SAFETY_ACTUATOR_FRAME_SIZE) {
                break;
            }
            if (client->rx_buf[pos + SAFETY_ACTUATOR_FRAME_SIZE - 1] == SAFETY_FRAME_TAIL) {
                safety_handle_actuator_values(client, &client->rx_buf[pos + 2]);
                pos += SAFETY_ACTUATOR_FRAME_SIZE;
                continue;
            }
            pos++;
            continue;
        }

        if (client->rx_buf[pos + 2] == SAFETY_FRAME_TAIL) {
            safety_handle_stm32_code(client, client->rx_buf[pos + 1]);
            pos += SAFETY_SIMPLE_FRAME_SIZE;
            continue;
        }

        pos++;
    }

    if (pos > 0) {
        memmove(client->rx_buf, &client->rx_buf[pos], client->rx_len - pos);
        client->rx_len -= pos;
    }
    if (client->rx_len == sizeof(client->rx_buf)) {
        client->rx_len = 0;
    }
}

static void safety_read_serial(SafetyInterlockClient *client) {
    uint8_t tmp[128];

    for (;;) {
        ssize_t n = read(client->fd, tmp, sizeof(tmp));
        if (n > 0) {
            size_t copy_len = (size_t)n;
            if (copy_len > sizeof(client->rx_buf) - client->rx_len) {
                client->rx_len = 0;
            }
            memcpy(&client->rx_buf[client->rx_len], tmp, copy_len);
            client->rx_len += copy_len;
            safety_parse_rx(client);
            continue;
        }
        if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }
        if (errno == EINTR) {
            continue;
        }

        fprintf(stderr, "[Safety] STM32 serial read failed: %s\n", strerror(errno));
        safety_close_serial(client);
        break;
    }
}

static void safety_update_online_state(SafetyInterlockClient *client,
                                       int64_t now_ms) {
    pthread_mutex_lock(&client->lock);
    if (client->last_rx_mono_ms == 0 ||
        now_ms - client->last_rx_mono_ms > SAFETY_STM32_OFFLINE_MS) {
        client->snapshot.online = 0;
    }
    pthread_mutex_unlock(&client->lock);
}

static void *safety_thread_main(void *arg) {
    SafetyInterlockClient *client = (SafetyInterlockClient *)arg;

    while (client->running) {
        int64_t now_ms = safety_mono_now_ms();

        if (client->fd < 0 &&
            (client->last_connect_attempt_ms == 0 ||
             now_ms - client->last_connect_attempt_ms >= SAFETY_RECONNECT_INTERVAL_MS)) {
            int rc;
            client->last_connect_attempt_ms = now_ms;
            rc = safety_open_serial(client);
            if (rc != 0) {
                fprintf(stderr, "[Safety] open %s failed: %s\n",
                        client->device, strerror(rc));
            }
        }

        if (client->fd >= 0) {
            struct pollfd pfd;
            pfd.fd = client->fd;
            pfd.events = POLLIN;
            pfd.revents = 0;
            if (poll(&pfd, 1, 20) > 0 && (pfd.revents & POLLIN)) {
                safety_read_serial(client);
            }
        } else {
            usleep(100000);
        }

        safety_update_online_state(client, now_ms);
    }

    safety_close_serial(client);
    return NULL;
}

int safety_client_init(SafetyInterlockClient *client,
                       const char *device,
                       int baud) {
    const char *env_device;
    const char *env_baud;
    int rc;

    if (client == NULL) {
        return EINVAL;
    }

    memset(client, 0, sizeof(*client));
    client->fd = -1;
    client->baud = baud > 0 ? baud : SAFETY_STM32_BAUD_DEFAULT;
    env_device = getenv(SAFETY_STM32_DEV_ENV);
    env_baud = getenv(SAFETY_STM32_BAUD_ENV);
    if (env_baud != NULL && env_baud[0] != '\0') {
        int parsed_baud = atoi(env_baud);
        if (parsed_baud > 0) {
            client->baud = parsed_baud;
        }
    }
    if (device != NULL && device[0] != '\0') {
        snprintf(client->device, sizeof(client->device), "%s", device);
    } else if (env_device != NULL && env_device[0] != '\0') {
        snprintf(client->device, sizeof(client->device), "%s", env_device);
    } else {
        snprintf(client->device, sizeof(client->device), "%s", SAFETY_STM32_DEV_DEFAULT);
    }

    rc = pthread_mutex_init(&client->lock, NULL);
    if (rc != 0) {
        return rc;
    }

    client->running = 1;
    client->start_mono_ms = safety_mono_now_ms();
    rc = pthread_create(&client->thread, NULL, safety_thread_main, client);
    if (rc != 0) {
        client->running = 0;
        pthread_mutex_destroy(&client->lock);
        return rc;
    }

    client->started = 1;
    printf("[Safety] STM32 serial client started dev=%s baud=%d\n",
           client->device, client->baud);
    return 0;
}

void safety_client_stop(SafetyInterlockClient *client) {
    if (client == NULL || !client->started) {
        return;
    }

    client->running = 0;
    pthread_join(client->thread, NULL);
    client->started = 0;
    pthread_mutex_destroy(&client->lock);
    printf("[Safety] STM32 serial client stopped\n");
}

int safety_client_get_snapshot(SafetyInterlockClient *client,
                               SafetyStm32Snapshot *snapshot_out) {
    if (client == NULL || snapshot_out == NULL) {
        return EINVAL;
    }

    pthread_mutex_lock(&client->lock);
    *snapshot_out = client->snapshot;
    pthread_mutex_unlock(&client->lock);
    return 0;
}

int safety_client_update_ai_detect_state(SafetyInterlockClient *client,
                                         uint8_t ai_detect_state,
                                         uint8_t ai_confidence,
                                         int64_t update_ms) {
    if (client == NULL) {
        return EINVAL;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.ai_detect_valid = 1;
    client->snapshot.ai_detect_state = ai_detect_state;
    client->snapshot.ai_confidence = ai_confidence;
    client->snapshot.ai_detect_update_ms = update_ms;
    pthread_mutex_unlock(&client->lock);
    return 0;
}

int safety_client_send_ai_status(SafetyInterlockClient *client,
                                 uint16_t ai_flags,
                                 uint8_t ai_confidence) {
    (void)client;
    (void)ai_flags;
    (void)ai_confidence;
    return 0;
}

static uint8_t safety_fusion_to_simple_code(uint8_t permit_decision,
                                            uint8_t risk_level,
                                            uint8_t risk_type,
                                            uint8_t stm32_action,
                                            uint8_t explain_code) {
    (void)risk_level;
    (void)stm32_action;

    if (risk_type == SAFETY_RISK_TYPE_INTRUSION) {
        return SAFETY_CODE_INTRUSION;
    }
    if (risk_type == SAFETY_RISK_TYPE_FIRE) {
        return explain_code == 42 ?
               SAFETY_CODE_FIRE_OUT_ZONE :
               SAFETY_CODE_FIRE_WORK_ZONE;
    }
    if (risk_type == SAFETY_RISK_TYPE_ENV) {
        return SAFETY_CODE_ENV_DANGER;
    }
    if (risk_type == SAFETY_RISK_TYPE_STM32_FAULT) {
        return SAFETY_CODE_FAULT;
    }
    if (risk_type == SAFETY_RISK_TYPE_STM32_LINK) {
        return SAFETY_CODE_FAULT;
    }
    if (risk_type == SAFETY_RISK_TYPE_PPE) {
        return SAFETY_CODE_PPE_DENY;
    }
    if (permit_decision == SAFETY_PERMIT_ALLOW ||
        risk_type == SAFETY_RISK_TYPE_NONE) {
        return SAFETY_CODE_SAFE;
    }

    return SAFETY_CODE_RESERVED;
}

int safety_client_send_fusion_decision(SafetyInterlockClient *client,
                                       uint8_t permit_decision,
                                       uint8_t risk_level,
                                       uint8_t risk_type,
                                       uint8_t voice_action,
                                       uint8_t stm32_action,
                                       uint8_t explain_code) {
    uint8_t code;

    (void)voice_action;
    code = safety_fusion_to_simple_code(permit_decision,
                                        risk_level,
                                        risk_type,
                                        stm32_action,
                                        explain_code);
    {
        int rc = safety_send_simple_code(client, code);
        if (rc == 0) {
            safety_update_expected_actuator(client, code);
        }
        return rc;
    }
}
