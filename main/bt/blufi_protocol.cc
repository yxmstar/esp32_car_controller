#include "blufi_protocol.h"
#include "esp_log.h"
#include <cstring>
#include "cJSON.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_blufi.h"
#include "esp_wifi.h"
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "78__esp-wifi-connect/include/wifi_station.h"
#include "78__esp-wifi-connect/include/ssid_manager.h"

static const char* TAG = "BlufiProtocol";

static const uint16_t MY_BLUFI_APP_UUID = 0;

static const uint8_t DEVICE_NAME_MAX_LEN = 29;

// 蓝牙Blufi回调函数
static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);
// 蓝牙GAP回调函数
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
// 蓝牙GATTS回调函数
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

BlufiProtocol& BlufiProtocol::GetInstance() {
    static BlufiProtocol instance;
    return instance;
}

BlufiProtocol::BlufiProtocol()
    : state_(BlufiState::IDLE),
      gatts_if_(ESP_GATT_IF_NONE),
      conn_id_(0),
      device_name_("Xiaozhi-BLE"),
      is_connected_(false),
      provisioned_callback_(nullptr),
      state_changed_callback_(nullptr) {
}

BlufiProtocol::~BlufiProtocol() {
    Stop();
}

bool BlufiProtocol::Start(const std::string& device_name) {
    ESP_LOGI(TAG, "Starting Blufi protocol...");
    if (state_ != BlufiState::IDLE) {
        ESP_LOGW(TAG, "Blufi protocol running, stop...");
        Stop();
        // 等待一小段时间以确保底层完全停止
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!device_name.empty()) {
        device_name_ = device_name;
    }
    ESP_LOGI(TAG, "Using device name: %s", device_name_.c_str());

    if (device_name_.length() > DEVICE_NAME_MAX_LEN) {
        device_name_.resize(DEVICE_NAME_MAX_LEN);
        ESP_LOGI(TAG, "Device name truncated to: %s", device_name_.c_str());
    }

    ESP_LOGI(TAG, "Initializing BLE stack...");
    if (!InitBle()) {
        ESP_LOGE(TAG, "Failed to initialize BLE");
        return false;
    }
    ESP_LOGI(TAG, "BLE stack initialized successfully");

    // 初始化Blufi
    esp_blufi_callbacks_t blufi_callbacks = {
        .event_cb = blufi_event_callback,
        .negotiate_data_handler = NULL,
        .encrypt_func = NULL,
        .decrypt_func = NULL,
        .checksum_func = NULL,
    };

    esp_err_t ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if (ret) {
        ESP_LOGE(TAG, "blufi register error, error code = %x", ret);
        return false;
    }

    // 注册GATTS回调
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return false;
    }

    // 注册GAP回调
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return false;
    }

    // 注册Blufi应用
    ret = esp_ble_gatts_app_register(MY_BLUFI_APP_UUID);
    if (ret) {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return false;
    }

    // 设置设备名称
    ret = esp_ble_gap_set_device_name(device_name_.c_str());
    if (ret) {
        ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
        return false;
    }

    state_ = BlufiState::ADVERTISING;
    if (state_changed_callback_) {
        state_changed_callback_(state_);
    }

    ESP_LOGI(TAG, "Blufi protocol started successfully");
    return true;
}

void BlufiProtocol::Stop() {
    if (state_ == BlufiState::IDLE) {
        return;
    }
    ESP_LOGI(TAG, "Stopping Blufi protocol...");

    esp_blufi_deinit();
    esp_ble_gap_stop_advertising();

    // 确保蓝牙控制器已完全禁用和反初始化
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bluedroid_disable();
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
        esp_bluedroid_deinit();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_bt_controller_deinit();
    }

    state_ = BlufiState::IDLE;
    if (state_changed_callback_) {
        state_changed_callback_(state_);
    }
    ESP_LOGI(TAG, "Blufi protocol stopped.");
}

void BlufiProtocol::OnProvisioned(std::function<void(const std::string&, const std::string&)> callback) {
    provisioned_callback_ = callback;
}

void BlufiProtocol::OnStateChanged(std::function<void(BlufiState)> callback) {
    state_changed_callback_ = callback;
}

BlufiState BlufiProtocol::GetState() const {
    return state_;
}

bool BlufiProtocol::InitBle() {
    ESP_LOGI(TAG, "Initializing BLE...");
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 释放蓝牙控制器内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    // 启用蓝牙控制器
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    // 初始化Bluedroid协议栈
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s bluedroid init failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s bluedroid enable failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    // 初始化Blufi
    ret = esp_blufi_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init blufi failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    return true;
}

void BlufiProtocol::StartAdvertising() {
    ESP_LOGI(TAG, "Configuring BLE advertising...");

    // 在配置广播前，强制再次设置设备名称
    esp_err_t ret = esp_ble_gap_set_device_name(device_name_.c_str());
    if (ret) {
        ESP_LOGE(TAG, "set device name failed in StartAdvertising, error code = %x", ret);
    }

    // 配置广播数据
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x0006, // 广播间隔最小值 (N * 0.625 ms)
        .max_interval = 0x0010, // 广播间隔最大值 (N * 0.625 ms)
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    // 配置扫描响应数据
    esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    // 设置广播参数
    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x100,
        .adv_int_max = 0x100,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    // 设置广播数据
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        return;
    }

    // 设置扫描响应数据
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret) {
        ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
        return;
    }

    // 启动广播
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "start advertising failed, error code = %x", ret);
        return;
    }

    ESP_LOGI(TAG, "Start advertising");
}

void BlufiProtocol::StopAdvertising() {
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "Stop advertising");
}

void BlufiProtocol::HandleWifiConfig(const std::string& ssid, const std::string& password) {
    ESP_LOGI(TAG, "Received WiFi config: SSID=%s, Password=%s", ssid.c_str(), password.c_str());

    state_ = BlufiState::PROVISIONING;
    if (state_changed_callback_) {
        state_changed_callback_(state_);
    }

    // 保存WiFi配置前，先清除所有旧的配置
    auto& ssid_manager = SsidManager::GetInstance();
    ESP_LOGI(TAG, "Clearing all previous WiFi configurations.");
    //ssid_manager.Clear();//不清配置
    ssid_manager.AddSsid(ssid, password);

    // 调用配网成功回调
    if (provisioned_callback_) {
        provisioned_callback_(ssid, password);
    }

    state_ = BlufiState::PROVISIONED;
    if (state_changed_callback_) {
        state_changed_callback_(state_);
    }
}

void BlufiProtocol::HandleBlufiEvent(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param) {
    switch (event) {
        case ESP_BLUFI_EVENT_INIT_FINISH:
            ESP_LOGI(TAG, "BLUFI init finish");
            break;
        case ESP_BLUFI_EVENT_DEINIT_FINISH:
            ESP_LOGI(TAG, "BLUFI deinit finish");
            break;
        case ESP_BLUFI_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "BLUFI ble connect");
            is_connected_ = true;
            state_ = BlufiState::CONNECTED;
            if (state_changed_callback_) {
                state_changed_callback_(state_);
            }
            // 停止广播
            StopAdvertising();
            break;
        case ESP_BLUFI_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "BLUFI ble disconnect");
            is_connected_ = false;
            if (state_ != BlufiState::PROVISIONED) {
                state_ = BlufiState::ADVERTISING;
                if (state_changed_callback_) {
                    state_changed_callback_(state_);
                }
                // 重新开始广播
                StartAdvertising();
            }
            break;
        case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
            ESP_LOGI(TAG, "BLUFI Set WIFI opmode %d", param->wifi_mode.op_mode);
            break;
        case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
            ESP_LOGI(TAG, "BLUFI request wifi connect to AP");
            break;
        case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
            ESP_LOGI(TAG, "BLUFI request wifi disconnect from AP");
            break;
        case ESP_BLUFI_EVENT_RECV_STA_SSID:
            ESP_LOGI(TAG, "BLUFI receive sta ssid %s", param->sta_ssid.ssid);
            // 保存SSID
            static std::string ssid;
            ssid = std::string(reinterpret_cast<const char*>(param->sta_ssid.ssid), param->sta_ssid.ssid_len);
            break;
        case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
            ESP_LOGI(TAG, "BLUFI receive sta password %s", param->sta_passwd.passwd);
            // 保存密码
            static std::string password;
            password = std::string(reinterpret_cast<const char*>(param->sta_passwd.passwd), param->sta_passwd.passwd_len);
            // 处理WiFi配置
            if (!ssid.empty()) {
                HandleWifiConfig(ssid, password);
            }
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
            wifi_mode_t mode = WIFI_MODE_STA;
            if (mode == WIFI_MODE_STA) {
                esp_blufi_extra_info_t info;
                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
            }
            break;
        }
        case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
            ESP_LOGI(TAG, "BLUFI receive custom data %d bytes", param->custom_data.data_len);
            // 打印接收到的数据内容
            if (param->custom_data.data && param->custom_data.data_len > 0) {
                ESP_LOGI(TAG, "Custom data content:");
                for (uint32_t i = 0; i < param->custom_data.data_len; i++) {
                    if (i % 16 == 0) {
                        ESP_LOGI(TAG, "%04X: ", i);
                    }
                    ESP_LOGI(TAG, "%02X ", param->custom_data.data[i]);
                    if ((i + 1) % 16 == 0 || i == param->custom_data.data_len - 1) {
                        ESP_LOGI(TAG, "\n");
                    }
                }

                ESP_LOGI(TAG, "Custom len:%d , data:%.*s", param->custom_data.data_len, param->custom_data.data_len, param->custom_data.data);
            }
            break;
        case ESP_BLUFI_EVENT_RECV_USERNAME:
            ESP_LOGI(TAG, "BLUFI receive username %s", param->username.name);
            break;
        case ESP_BLUFI_EVENT_RECV_CA_CERT:
            ESP_LOGI(TAG, "BLUFI receive CA certificate");
            break;
        case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
            ESP_LOGI(TAG, "BLUFI receive client certificate");
            break;
        case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
            ESP_LOGI(TAG, "BLUFI receive server certificate");
            break;
        case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
            ESP_LOGI(TAG, "BLUFI receive client private key");
            break;
        case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
            ESP_LOGI(TAG, "BLUFI receive server private key");
            break;
        case ESP_BLUFI_EVENT_REPORT_ERROR:
            ESP_LOGE(TAG, "BLUFI report error, error code %d", param->report_error.state);
            break;
        default:
            break;
    }
}

void BlufiProtocol::HandleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed, error status = %x", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising start success");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed, error status = %x", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stop success");
            }
            break;
        default:
            break;
    }
}

void BlufiProtocol::HandleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, status = %d, app_id = %d", param->reg.status, param->reg.app_id);
            if (param->reg.status == ESP_GATT_OK) {
                gatts_if_ = gatts_if;
                esp_err_t ret = esp_blufi_profile_init();
                if (ret) {
                    ESP_LOGE(TAG, "blufi profile init error, error code = %x", ret);
                    return;
                }
                StartAdvertising();
            } else {
                ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
                return;
            }
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            conn_id_ = param->connect.conn_id;
            is_connected_ = true;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            is_connected_ = false;
            break;
        default:
            break;
    }
}

// 全局回调函数
static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param) {
    BlufiProtocol::GetInstance().HandleBlufiEvent(event, param);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    BlufiProtocol::GetInstance().HandleGapEvent(event, param);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    BlufiProtocol::GetInstance().HandleGattsEvent(event, gatts_if, param);
}
