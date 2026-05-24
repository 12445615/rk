#include "audio_alert.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define AUDIO_COOLDOWN_MS 5000 // 冷却时间：5秒内不重复播报

static int64_t g_last_audio_ms = 0; 
static int g_fifo_fd = -1;

void audio_alert_init(void) {
    // 创建命名管道 (FIFO)
    mkfifo("/tmp/audio_fifo", 0666);
    // 以读写+非阻塞模式打开，绝不会卡死主程序
    g_fifo_fd = open("/tmp/audio_fifo", O_RDWR | O_NONBLOCK);
    
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

void audio_alert_update(DetectSharedState *shared, int64_t now_ms) {
    uint32_t before, after;
    DetectSharedState snapshot;
    int valid_read = 0;

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

    int has_fire = 0, no_vest = 0, no_helmet = 0;

    for (int i = 0; i < snapshot.box_count; i++) {
        char *name = coco_cls_to_name(snapshot.boxes[i].class_id);
        if (strcmp(name, "fire") == 0) has_fire = 1;
        else if (strcmp(name, "yelek yok") == 0) no_vest = 1;
        else if (strcmp(name, "kask yok") == 0) no_helmet = 1;
    }

    // 确定优先级 (火灾 > 没穿衣 > 没戴帽)
    int target_alert = 0;
    if (has_fire) target_alert = 1;
    else if (no_vest) target_alert = 2;
    else if (no_helmet) target_alert = 3;

    if (target_alert > 0) {
        // 判断冷却时间是否已过
        if (now_ms - g_last_audio_ms > AUDIO_COOLDOWN_MS) {
            g_last_audio_ms = now_ms;
            
            if (target_alert == 1) printf("[Audio Alert] Fire Warning!\n");
            else if (target_alert == 2) printf("[Audio Alert] No Vest Warning!\n");
            else if (target_alert == 3) printf("[Audio Alert] No Helmet Warning!\n");

            // 纳秒级写入管道，通知外部脚本发声
            if (g_fifo_fd >= 0) {
                char buf[2] = {'0' + target_alert, '\n'};
                write(g_fifo_fd, buf, 2);
            }
        }
    }
}