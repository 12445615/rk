#include "audio_alert.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define AUDIO_COOLDOWN_MS 5000
#define AUDIO_FIFO_PATH "/tmp/audio_fifo"

static int64_t g_last_audio_ms = 0;
static int g_fifo_fd = -1;

void audio_alert_init(void) {
    mkfifo(AUDIO_FIFO_PATH, 0666);
    g_fifo_fd = open(AUDIO_FIFO_PATH, O_RDWR | O_NONBLOCK);

    if (g_fifo_fd >= 0) {
        printf("[Audio Alert] IPC Pipe initialized. Cooldown: %d ms\n", AUDIO_COOLDOWN_MS);
    } else {
        perror("[Audio Alert] Failed to open audio FIFO");
    }
}

void audio_alert_deinit(void) {
    if (g_fifo_fd >= 0) {
        close(g_fifo_fd);
        g_fifo_fd = -1;
    }
}

void audio_alert_send_command(int command, int64_t now_ms) {
    char buf[2];

    if (command <= 0 || command > 9) {
        return;
    }
    if (now_ms - g_last_audio_ms <= AUDIO_COOLDOWN_MS) {
        return;
    }

    g_last_audio_ms = now_ms;

    if (command == 1) printf("[Audio Alert] Fire Warning!\n");
    else if (command == 2) printf("[Audio Alert] No Vest Warning!\n");
    else if (command == 3) printf("[Audio Alert] No Helmet Warning!\n");
    else if (command == 4) printf("[Audio Alert] Intrusion Warning!\n");
    else printf("[Audio Alert] Command %d\n", command);

    if (g_fifo_fd >= 0) {
        buf[0] = (char)('0' + command);
        buf[1] = '\n';
        write(g_fifo_fd, buf, sizeof(buf));
    }
}

void audio_alert_update(DetectSharedState *shared, int64_t now_ms) {
    uint32_t before, after;
    DetectSharedState snapshot;
    int valid_read = 0;
    int has_fire = 0;
    int no_vest = 0;
    int no_helmet = 0;
    int target_alert = 0;

    if (shared == NULL || !shared->valid) return;

    for (int tries = 0; tries < 3; tries++) {
        before = shared->version;
        if ((before & 1U) != 0U) continue;
        __sync_synchronize();
        memcpy(&snapshot, shared, sizeof(snapshot));
        __sync_synchronize();
        after = shared->version;
        if (before == after && (after & 1U) == 0U && snapshot.valid) {
            valid_read = 1;
            break;
        }
    }

    if (!valid_read) return;
    if (snapshot.timestamp_ms <= 0 || now_ms - snapshot.timestamp_ms > 1500) return;

    for (int i = 0; i < snapshot.box_count; i++) {
        char *name = coco_cls_to_name(snapshot.boxes[i].class_id);
        if (strcmp(name, "fire") == 0) has_fire = 1;
        else if (strcmp(name, "no-vest") == 0) no_vest = 1;
        else if (strcmp(name, "no-helmet") == 0) no_helmet = 1;
    }

    if (has_fire) target_alert = 1;
    else if (no_vest) target_alert = 2;
    else if (no_helmet) target_alert = 3;

    audio_alert_send_command(target_alert, now_ms);
}
