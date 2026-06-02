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
#define SAFETY_MIN_FRAME_SIZE 9

static int64_t safety_mono_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static uint16_t safety_get_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t safety_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void safety_put_le16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xff);
    p[1] = (uint8_t)((value >> 8) & 0xff);
}

static void safety_put_le32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value & 0xff);
    p[1] = (uint8_t)((value >> 8) & 0xff);
    p[2] = (uint8_t)((value >> 16) & 0xff);
    p[3] = (uint8_t)((value >> 24) & 0xff);
}

static uint16_t safety_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xffff;
    size_t i;

    for (i = 0; i < len; i++) {
        int bit;
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (uint16_t)((crc >> 1) ^ 0xa001);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
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

static int safety_send_frame(SafetyInterlockClient *client,
                             uint8_t type,
                             const uint8_t *payload,
                             uint8_t payload_len) {
    uint8_t frame[SAFETY_MIN_FRAME_SIZE + SAFETY_PAYLOAD_MAX];
    uint16_t crc;
    size_t frame_len;
    ssize_t written;

    if (client == NULL || client->fd < 0) {
        return ENOTCONN;
    }
    if (payload_len > 0 && payload == NULL) {
        return EINVAL;
    }

    frame[0] = SAFETY_FRAME_HEAD1;
    frame[1] = SAFETY_FRAME_HEAD2;
    frame[2] = SAFETY_FRAME_VERSION;
    frame[3] = type;
    frame[4] = client->tx_seq++;
    frame[5] = payload_len;
    if (payload_len > 0) {
        memcpy(&frame[6], payload, payload_len);
    }
    crc = safety_crc16(&frame[2], (size_t)4 + payload_len);
    safety_put_le16(&frame[6 + payload_len], crc);
    frame[8 + payload_len] = SAFETY_FRAME_TAIL;
    frame_len = (size_t)SAFETY_MIN_FRAME_SIZE + payload_len;

    written = write(client->fd, frame, frame_len);
    if (written < 0) {
        return errno;
    }
    if ((size_t)written != frame_len) {
        return EIO;
    }

    return 0;
}

static int safety_send_event_ack(SafetyInterlockClient *client,
                                 uint16_t event_id,
                                 uint8_t ack_result) {
    uint8_t payload[3];
    safety_put_le16(payload, event_id);
    payload[2] = ack_result;
    return safety_send_frame(client, SAFETY_MSG_EVENT_ACK, payload, sizeof(payload));
}

static void safety_mark_rx(SafetyInterlockClient *client) {
    pthread_mutex_lock(&client->lock);
    client->last_rx_mono_ms = safety_mono_now_ms();
    client->snapshot.online = 1;
    pthread_mutex_unlock(&client->lock);
}

static void safety_handle_stm32_state(SafetyInterlockClient *client,
                                      const uint8_t *payload,
                                      uint8_t len) {
    if (len < 4) {
        return;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.work_state = payload[0];
    client->snapshot.stm32_flags = safety_get_le16(&payload[1]);
    client->snapshot.fault_code = payload[3];
    pthread_mutex_unlock(&client->lock);
    safety_mark_rx(client);
}

static void safety_handle_sensor_data(SafetyInterlockClient *client,
                                      const uint8_t *payload,
                                      uint8_t len) {
    if (len < 6) {
        return;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.smoke = safety_get_le16(&payload[0]);
    client->snapshot.gas = safety_get_le16(&payload[2]);
    client->snapshot.temperature_x10 = (int16_t)safety_get_le16(&payload[4]);
    pthread_mutex_unlock(&client->lock);
    safety_mark_rx(client);
}

static void safety_handle_actuator_state(SafetyInterlockClient *client,
                                         const uint8_t *payload,
                                         uint8_t len) {
    if (len < 2) {
        return;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.actuator_flags = safety_get_le16(payload);
    pthread_mutex_unlock(&client->lock);
    safety_mark_rx(client);
}

static void safety_handle_interlock_event(SafetyInterlockClient *client,
                                          const uint8_t *payload,
                                          uint8_t len) {
    uint16_t event_id;

    if (len < 8) {
        return;
    }

    event_id = safety_get_le16(&payload[0]);
    pthread_mutex_lock(&client->lock);
    client->snapshot.last_event_id = event_id;
    client->snapshot.last_event_type = payload[2];
    client->snapshot.last_event_reason = payload[3];
    client->snapshot.last_event_time_s = safety_get_le32(&payload[4]);
    pthread_mutex_unlock(&client->lock);
    safety_mark_rx(client);

    printf("[Safety] STM32 event id=%u type=%u reason=%u time_s=%u\n",
           event_id, payload[2], payload[3], safety_get_le32(&payload[4]));
    safety_send_event_ack(client, event_id, SAFETY_ACK_OK);
}

static void safety_handle_fault_event(SafetyInterlockClient *client,
                                      const uint8_t *payload,
                                      uint8_t len) {
    if (len < 3) {
        return;
    }

    pthread_mutex_lock(&client->lock);
    client->snapshot.last_fault_code = payload[0];
    client->snapshot.last_fault_level = payload[1];
    client->snapshot.last_fault_detail = payload[2];
    client->snapshot.fault_code = payload[0];
    pthread_mutex_unlock(&client->lock);
    safety_mark_rx(client);

    printf("[Safety] STM32 fault code=%u level=%u detail=%u\n",
           payload[0], payload[1], payload[2]);
}

static void safety_handle_frame(SafetyInterlockClient *client,
                                uint8_t type,
                                const uint8_t *payload,
                                uint8_t len) {
    switch (type) {
    case SAFETY_MSG_STM32_STATE:
        safety_handle_stm32_state(client, payload, len);
        break;
    case SAFETY_MSG_SENSOR_DATA:
        safety_handle_sensor_data(client, payload, len);
        break;
    case SAFETY_MSG_ACTUATOR_STATE:
        safety_handle_actuator_state(client, payload, len);
        break;
    case SAFETY_MSG_INTERLOCK_EVENT:
        safety_handle_interlock_event(client, payload, len);
        break;
    case SAFETY_MSG_FAULT_EVENT:
        safety_handle_fault_event(client, payload, len);
        break;
    default:
        printf("[Safety] Unsupported STM32 message type=0x%02x len=%u\n",
               type, len);
        safety_mark_rx(client);
        break;
    }
}

static void safety_parse_rx(SafetyInterlockClient *client) {
    size_t pos = 0;

    while (client->rx_len - pos >= SAFETY_MIN_FRAME_SIZE) {
        uint8_t len;
        size_t frame_len;
        uint16_t crc_expected;
        uint16_t crc_actual;

        if (client->rx_buf[pos] != SAFETY_FRAME_HEAD1 ||
            client->rx_buf[pos + 1] != SAFETY_FRAME_HEAD2) {
            pos++;
            continue;
        }

        len = client->rx_buf[pos + 5];
        frame_len = (size_t)SAFETY_MIN_FRAME_SIZE + len;
        if (client->rx_len - pos < frame_len) {
            break;
        }

        if (client->rx_buf[pos + frame_len - 1] != SAFETY_FRAME_TAIL) {
            pos++;
            continue;
        }
        if (client->rx_buf[pos + 2] != SAFETY_FRAME_VERSION) {
            pos += frame_len;
            continue;
        }

        crc_expected = safety_get_le16(&client->rx_buf[pos + 6 + len]);
        crc_actual = safety_crc16(&client->rx_buf[pos + 2], (size_t)4 + len);
        if (crc_expected != crc_actual) {
            printf("[Safety] CRC error type=0x%02x seq=%u\n",
                   client->rx_buf[pos + 3], client->rx_buf[pos + 4]);
            pos += frame_len;
            continue;
        }

        safety_handle_frame(client,
                            client->rx_buf[pos + 3],
                            &client->rx_buf[pos + 6],
                            len);
        pos += frame_len;
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

static void safety_send_heartbeat_if_needed(SafetyInterlockClient *client,
                                            int64_t now_ms) {
    uint8_t payload[5];
    uint32_t uptime_s;
    int rc;

    if (client->fd < 0) {
        return;
    }
    if (client->last_heartbeat_ms != 0 &&
        now_ms - client->last_heartbeat_ms < SAFETY_HEARTBEAT_INTERVAL_MS) {
        return;
    }

    uptime_s = (uint32_t)((now_ms - client->start_mono_ms) / 1000);
    payload[0] = SAFETY_RK_STATE_RUNNING;
    safety_put_le32(&payload[1], uptime_s);
    rc = safety_send_frame(client, SAFETY_MSG_HEARTBEAT, payload, sizeof(payload));
    if (rc != 0) {
        fprintf(stderr, "[Safety] heartbeat send failed: %s\n", strerror(rc));
        safety_close_serial(client);
        return;
    }

    client->last_heartbeat_ms = now_ms;
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
            safety_send_heartbeat_if_needed(client, now_ms);
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

int safety_client_send_ai_status(SafetyInterlockClient *client,
                                 uint16_t ai_flags,
                                 uint8_t ai_confidence) {
    uint8_t payload[3];

    if (ai_confidence > 100) {
        ai_confidence = 100;
    }
    safety_put_le16(payload, ai_flags);
    payload[2] = ai_confidence;
    return safety_send_frame(client, SAFETY_MSG_AI_STATUS, payload, sizeof(payload));
}

int safety_client_send_fusion_decision(SafetyInterlockClient *client,
                                       uint8_t permit_decision,
                                       uint8_t risk_level,
                                       uint8_t risk_type,
                                       uint8_t voice_action,
                                       uint8_t stm32_action,
                                       uint8_t explain_code) {
    uint8_t payload[6];

    payload[0] = permit_decision;
    payload[1] = risk_level;
    payload[2] = risk_type;
    payload[3] = voice_action;
    payload[4] = stm32_action;
    payload[5] = explain_code;
    return safety_send_frame(client, SAFETY_MSG_FUSION_DECISION, payload, sizeof(payload));
}
