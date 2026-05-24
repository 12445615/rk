#include "sensor_modbus.h"
#include <poll.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>

// 初始化全局传感器数据结构体
SensorData g_sensor_data = {0, 0.0f, 0.0f, 0, 0, PTHREAD_MUTEX_INITIALIZER};

#if defined(__has_include)
#if __has_include(<modbus/modbus.h>)
#define HAVE_LIBMODBUS 1
#endif
#endif

#ifndef HAVE_LIBMODBUS
#define HAVE_LIBMODBUS 0
#endif

#if HAVE_LIBMODBUS
#include <modbus/modbus.h>

extern volatile sig_atomic_t is_running;

#define SENSOR_READ_FAIL_RECONNECT_THRESHOLD 3
#define SENSOR_RECONNECT_WAIT_MS 1000
#define SENSOR_READ_WAIT_MS 500
#define SENSOR_WARN_EVERY_N 10

static void sensor_clear_alarm_status(void) {
    pthread_mutex_lock(&g_sensor_data.lock);
    g_sensor_data.alarm_status = 0;
    pthread_mutex_unlock(&g_sensor_data.lock);
}

static int sensor_modbus_connect_once(modbus_t *ctx) {
    int rc;

    rc = modbus_connect(ctx);
    if (rc == -1) {
        return -1;
    }

    modbus_set_response_timeout(ctx, 0, 500000);
    return 0;
}

static void sensor_log_read_failure(const char *label,
                                    int err,
                                    int consecutive_failures,
                                    int *warn_count) {
    if (warn_count == NULL) {
        return;
    }

    (*warn_count)++;
    if (*warn_count == 1 || (*warn_count % SENSOR_WARN_EVERY_N) == 0) {
        printf("[传感器警告] %s失败: %s (连续失败=%d, 累计警告=%d)\n",
               label,
               modbus_strerror(err),
               consecutive_failures,
               *warn_count);
    }
}


static void* sensor_thread_func(void* arg) {
    const char* path = (const char*)arg;
    int connected = 0;
    int consecutive_failures = 0;
    int read_warn_count = 0;
    int reconnect_warn_count = 0;
    
    // ==========================================
    // 【新增】30秒滑动窗口，用于计算温湿度/烟雾骤升
    // 每次读取间隔约 500ms (SENSOR_READ_WAIT_MS)，30秒需要 60 个历史点
    // ==========================================
    #define SENSOR_RISE_WINDOW_SEC 30
    #define SENSOR_HISTORY_LEN ((SENSOR_RISE_WINDOW_SEC * 1000) / SENSOR_READ_WAIT_MS)
    float temp_history[SENSOR_HISTORY_LEN] = {0};
    int ppm_history[SENSOR_HISTORY_LEN] = {0};
    int history_idx = 0;
    int history_count = 0;
    
    // 1. 初始化 Modbus 句柄
    modbus_t *ctx = modbus_new_rtu(path, 9600, 'N', 8, 1); 
    if (ctx == NULL) {
        printf("[传感器错误] 无法创建 Modbus 句柄\n");
        return NULL;
    }
    
    modbus_set_slave(ctx, 1); // 默认从机地址为1

    // 尝试首次连接
    if (sensor_modbus_connect_once(ctx) == -1) {
        printf("[传感器致命错误] 串口 %s 连接失败: %s\n", path, modbus_strerror(errno));
        modbus_free(ctx);
        return NULL;
    }
    connected = 1;
    printf("[传感器] 成功连接到串口%s\n", path);

    while (is_running) {
        uint16_t regs[10] = {0};
        int hw_alarm = 0;
        int concentration_ok = 0;
        int reconnect_needed = 0;
        int rc;

        // ==========================================
        // 断线重连逻辑
        // ==========================================
        if (!connected) {
            sensor_clear_alarm_status();
            if (sensor_modbus_connect_once(ctx) == -1) {
                reconnect_warn_count++;
                if (reconnect_warn_count == 1 ||
                    (reconnect_warn_count % SENSOR_WARN_EVERY_N) == 0) {
                    printf("[传感器警告] 串口 %s 重连失败: %s (累计=%d)\n",
                           path,
                           modbus_strerror(errno),
                           reconnect_warn_count);
                }
                poll(NULL, 0, SENSOR_RECONNECT_WAIT_MS);
                continue;
            }

            connected = 1;
            consecutive_failures = 0;
            read_warn_count = 0;
            reconnect_warn_count = 0;
            // 连接成功后清空历史数据，防止产生错误的“骤升”计算
            history_count = 0; 
            history_idx = 0;
            printf("[传感器] 串口 %s 重连成功\n", path);
        }

        // ==========================================
        // 1. 读取浓度与硬件报警位
        // ==========================================
        rc = modbus_read_registers(ctx, 0x0016, 6, regs);
        if (rc != -1) {
            pthread_mutex_lock(&g_sensor_data.lock);
            g_sensor_data.ppm = regs[0];  
            hw_alarm = regs[5];            
            pthread_mutex_unlock(&g_sensor_data.lock);
            concentration_ok = 1;
            consecutive_failures = 0;
            read_warn_count = 0;
        } else {
            int saved_errno = errno;
            consecutive_failures++;
            sensor_log_read_failure("读取浓度(0x0016)",
                                    saved_errno,
                                    consecutive_failures,
                                    &read_warn_count);
        }

        // ==========================================
        // 2. 读取运行/预热状态
        // ==========================================
        if (consecutive_failures < SENSOR_READ_FAIL_RECONNECT_THRESHOLD) {
            rc = modbus_read_registers(ctx, 0x001C, 1, regs);
            if (rc != -1) {
                pthread_mutex_lock(&g_sensor_data.lock);
                g_sensor_data.run_status = regs[0];
                pthread_mutex_unlock(&g_sensor_data.lock);
                consecutive_failures = 0;
                read_warn_count = 0;
            } else {
                int saved_errno = errno;
                consecutive_failures++;
                sensor_log_read_failure("读取运行状态(0x001C)",
                                        saved_errno,
                                        consecutive_failures,
                                        &read_warn_count);
            }
        }

        // ==========================================
        // 3. 读取温湿度
        // ==========================================
        if (consecutive_failures < SENSOR_READ_FAIL_RECONNECT_THRESHOLD) {
            rc = modbus_read_registers(ctx, 0x001E, 2, regs);
            if (rc != -1) {
                pthread_mutex_lock(&g_sensor_data.lock);
                g_sensor_data.temp = (float)regs[0] * 0.1f; 
                g_sensor_data.humi = (float)regs[1] * 0.1f; 
                pthread_mutex_unlock(&g_sensor_data.lock);
                consecutive_failures = 0;
                read_warn_count = 0;
            } else {
                int saved_errno = errno;
                consecutive_failures++;
                sensor_log_read_failure("读取温湿度(0x001E)",
                                        saved_errno,
                                        consecutive_failures,
                                        &read_warn_count);
            }
        }

        // 判断是否需要重连
        if (consecutive_failures >= SENSOR_READ_FAIL_RECONNECT_THRESHOLD) {
            reconnect_needed = 1;
        }

        if (reconnect_needed) {
            printf("[传感器警告] 连续读取失败达到 %d 次，关闭并准备重连串口 %s\n",
                   consecutive_failures,
                   path);
            sensor_clear_alarm_status();
            modbus_close(ctx);
            connected = 0;
            consecutive_failures = 0;
            poll(NULL, 0, SENSOR_READ_WAIT_MS);
            continue;
        }

        // ==========================================
        // 4. 报警判断逻辑 (带滑动窗口检测骤升)
        // ==========================================
        int is_dangerous = 0;
        if (concentration_ok) {
            pthread_mutex_lock(&g_sensor_data.lock);

            // 记录当前数据到历史滑动窗口
            temp_history[history_idx] = g_sensor_data.temp;
            ppm_history[history_idx] = g_sensor_data.ppm;
            history_idx = (history_idx + 1) % SENSOR_HISTORY_LEN;
            if (history_count < SENSOR_HISTORY_LEN) history_count++;

            // 找出过去这段时间内的最低温度和最低烟雾浓度
            float min_temp = g_sensor_data.temp;
            int min_ppm = g_sensor_data.ppm;
            for (int i = 0; i < history_count; i++) {
                if (temp_history[i] < min_temp) min_temp = temp_history[i];
                if (ppm_history[i] < min_ppm) min_ppm = ppm_history[i];
            }

            // 计算骤升幅度
            float temp_rise = g_sensor_data.temp - min_temp;
            int ppm_rise = g_sensor_data.ppm - min_ppm;

            // 核心判断规则：
            // 1. 温度短时间内骤升 >= 10.0 度
            // 2. 烟雾浓度短时间内飙升 >= 50
            // 3. 绝对温度超过 55.0 度
            // 4. 绝对烟雾浓度超过 300
            // 5. 传感器硬件报警位触发
            if (temp_rise >= 10.0f || 
                ppm_rise >= 50 || 
                g_sensor_data.temp > 55.0 || 
                g_sensor_data.ppm > 300 || 
                hw_alarm == 1) {
                is_dangerous = 1;
            }

            g_sensor_data.alarm_status = is_dangerous; 
            pthread_mutex_unlock(&g_sensor_data.lock);
        } else {
            sensor_clear_alarm_status();
        }

        // 使用 poll 做可中断等待
        poll(NULL, 0, SENSOR_READ_WAIT_MS);
    }
    
    modbus_close(ctx);
    modbus_free(ctx);
    return NULL;
}



int start_sensor_collector(const char* device_path) {
    pthread_t tid;
    int rc = pthread_create(&tid, NULL, sensor_thread_func, (void*)device_path);
    if (rc != 0) {
        return rc;
    }

    pthread_detach(tid);
    return 0;
}
#else
int start_sensor_collector(const char* device_path) {
    (void)device_path;
    return ENOSYS;
}
#endif
