#include "CarStatusMonitor.h"
#include <string.h>
#include "esp_log.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define UART_BAUD_RATE 115200

#define TAG "CarStatusMonitor"

CarStatusMonitor::CarStatusMonitor()
    : current_status_{false, false}, task_handle_(nullptr),
    brake_(false), light1_(false), light2_(false), light3_(false),
    light4_(false), light5_(false), driver_light_(false)
{
    status_mutex_ = xSemaphoreCreateMutex();
    // init_uart();
    // start_task();
}

CarStatusMonitor::~CarStatusMonitor() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
    }
    if (status_mutex_) {
        vSemaphoreDelete(status_mutex_);
    }
}

void CarStatusMonitor::init_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void CarStatusMonitor::start_task() {
    xTaskCreate(uart_task, "brake_status_task", 2048, this, 10, &task_handle_);
}

void CarStatusMonitor::uart_task(void* arg) {
    auto* monitor = static_cast<CarStatusMonitor*>(arg);
    uint8_t data[BUF_SIZE];

    while (true) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            CarStatusMonitor::Status new_status = monitor->ParseFrame(data, len);
            xSemaphoreTake(monitor->status_mutex_, portMAX_DELAY);
            monitor->current_status_ = new_status;
            xSemaphoreGive(monitor->status_mutex_);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 控制轮询频率
    }
}

CarStatusMonitor::Status CarStatusMonitor::ParseFrame(const uint8_t* data, size_t len) {
    Status s{false, false};
    if (len >= 8 && data[0] == 0xFC && data[3] == 0x01 && data[7] == 0x0D) {
        s.brake_on     = (data[4] & 0x01) != 0;
        s.seatbelt_on  = (data[5] & 0x01) != 0;
    }
    return s;
}

CarStatusMonitor::Status CarStatusMonitor::GetStatus() {
    Status s;
    xSemaphoreTake(status_mutex_, portMAX_DELAY);
    s = current_status_;
    xSemaphoreGive(status_mutex_);
    ESP_LOGI(TAG,"[GetStatus] Brake: %s, Seatbelt: %s\n",
           s.brake_on ? "ON" : "OFF",
           s.seatbelt_on ? "ON" : "OFF");
    return s;
}

bool CarStatusMonitor::WriteFrame(const uint8_t* data, size_t len) {
    int written = uart_write_bytes(UART_NUM, reinterpret_cast<const char*>(data), len);
    return written == len;
}

bool CarStatusMonitor::SendStatusFrame(bool brake, bool l1, bool l2, bool l3, bool l4, bool l5, bool driver_light) {
    uint8_t frame[13] = {
        0xFA, // 帧头
        0x00, // 地址
        0x00, 0x08, // 帧长
        0x06,       // 帧命令
        static_cast<uint8_t>(brake        ? 0x11 : 0x00),
        static_cast<uint8_t>(l1           ? 0x11 : 0x00),
        static_cast<uint8_t>(l2           ? 0x11 : 0x00),
        static_cast<uint8_t>(l3           ? 0x11 : 0x00),
        static_cast<uint8_t>(l4           ? 0x11 : 0x00),
        static_cast<uint8_t>(l5           ? 0x11 : 0x00),
        static_cast<uint8_t>(driver_light ? 0x11 : 0x00),
        0x0D        // 结束码
    };
    ESP_LOGI(TAG,"[SendStatusFrame] Brake: %s, L1~L5: [%s %s %s %s %s], Driver Light: %s\n",
           brake        ? "ON" : "OFF",
           l1           ? "ON" : "OFF",
           l2           ? "ON" : "OFF",
           l3           ? "ON" : "OFF",
           l4           ? "ON" : "OFF",
           l5           ? "ON" : "OFF",
           driver_light ? "ON" : "OFF");

    // return WriteFrame(frame, sizeof(frame));
    return true;
}

void CarStatusMonitor::SetStatus(bool brake, bool light1, bool light2, bool light3, bool light4, bool light5, bool driver_light) {
    brake_ = brake;
    light1_ = light1;
    light2_ = light2;
    light3_ = light3;
    light4_ = light4;
    light5_ = light5;
    driver_light_ = driver_light;
}

void CarStatusMonitor::GetCurrentStatus(bool& brake, bool& light1, bool& light2, bool& light3, bool& light4, bool& light5, bool& driver_light) {
    brake = brake_;
    light1 = light1_;
    light2 = light2_;
    light3 = light3_;
    light4 = light4_;
    light5 = light5_;
    driver_light = driver_light_;
}
