#pragma once

#include <string>
#include <functional>
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

enum class BlufiState {
    IDLE,
    ADVERTISING,
    CONNECTED,
    PROVISIONING,
    PROVISIONED,
    FAILED
};


class BlufiProtocol {
public:

    static BlufiProtocol& GetInstance();

    // 启动Blufi
    bool Start(const std::string& device_name = "");

    // 停止Blufi
    void Stop();

    // 设置配网成功回调
    void OnProvisioned(std::function<void(const std::string&, const std::string&)> callback);

    // 设置状态变更回调
    void OnStateChanged(std::function<void(BlufiState)> callback);

    // 获取当前状态
    BlufiState GetState() const;

    // 处理Blufi事件
    void HandleBlufiEvent(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

    // 处理GAP事件
    void HandleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    // 处理GATTS事件
    void HandleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

private:
    BlufiProtocol();
    ~BlufiProtocol();

    // 初始化BLE
    bool InitBle();

    // 启动广播
    void StartAdvertising();

    // 停止广播
    void StopAdvertising();

    // 处理WiFi配置
    void HandleWifiConfig(const std::string& ssid, const std::string& password);

    // 成员变量顺序必须与构造函数中的初始化顺序一致
    BlufiState state_;
    esp_gatt_if_t gatts_if_;
    uint16_t conn_id_;
    std::string device_name_;
    bool is_connected_;
    std::function<void(const std::string&, const std::string&)> provisioned_callback_;
    std::function<void(BlufiState)> state_changed_callback_;
};
