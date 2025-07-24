#ifndef BRAKE_STATUS_MONITOR_H
#define BRAKE_STATUS_MONITOR_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class CarStatusMonitor {
public:
    struct Status {
        bool brake_on;
        bool seatbelt_on;
    };

    CarStatusMonitor();
    ~CarStatusMonitor();

    Status GetStatus();                     // 获取当前状态
    bool WriteFrame(const uint8_t* data, size_t len);  // 向串口发送一帧数据
    bool SendStatusFrame(bool brake, bool l1, bool l2, bool l3, bool l4, bool l5, bool driver_light);
    // 更新当前状态（用于下发控制）
    void SetStatus(bool brake, bool light1, bool light2, bool light3, bool light4, bool light5, bool driver_light);

    // 读取当前状态（用于保持未变更项）
    void GetCurrentStatus(bool& brake, bool& light1, bool& light2, bool& light3, bool& light4, bool& light5, bool& driver_light);
private:
    void init_uart();
    void start_task();
    static void uart_task(void* arg);
    Status ParseFrame(const uint8_t* data, size_t len);

    Status current_status_;
    SemaphoreHandle_t status_mutex_;
    TaskHandle_t task_handle_;

    bool brake_;
    bool light1_;
    bool light2_;
    bool light3_;
    bool light4_;
    bool light5_;
    bool driver_light_;
};

#endif // BRAKE_STATUS_MONITOR_H
