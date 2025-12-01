/*
* A complete gateway solution for monitoring CyberPower UPS devices via USB HID
* and providing data access through HTTP web interface and ModbusRTU protocol.
*/

// Standard C libraries
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// ESP-IDF core
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"

// Network and WiFi
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

// USB HID
#include "usb/usb_host.h"
#include "usb/hid_host.h"

// Peripherals
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "led_strip.h"

// Modbus module
#include "modbus_ups.h"
#include "wifi_manager.h"
#include "cJSON.h"



/* It is use for change beep status */
#define APP_QUIT_PIN GPIO_NUM_0
#define RGB_LED_PIN GPIO_NUM_48

static led_strip_handle_t led_strip;

static const char *TAG = "ups";
QueueHandle_t hid_host_event_queue;
QueueHandle_t timer_queue;


typedef struct
{
    uint64_t event_count;
} timer_queue_element_t;

bool user_shutdown = false;
bool UPS_DEV_CONNECTED = false;
hid_host_device_handle_t latest_hid_device_handle;

// Type definitions
typedef struct {
    char name[64];
    uint8_t offset;
    uint8_t size;
    uint8_t bit;
    uint8_t type; // 0: U8, 1: U16_LE, 2: I16_LE, 3: BOOL
} hid_field_t;

typedef struct {
    uint8_t report_id;
    uint8_t report_type; // 1: INPUT, 3: FEATURE
    uint8_t length;
    hid_field_t *fields;
    uint8_t field_count;
} report_desc_t;


// Forward declarations for HID functions
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
    const hid_host_interface_event_t event,
    void *arg);
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
const hid_host_driver_event_t event,
void *arg);

// Global UPS data (defined in modbus_ups.c)
extern UpsData current_ups_data;

// Forward function declarations
 bool ups_read_report(uint8_t report_id, uint8_t *buffer, size_t len);
 void ups_update_data_from_buffer(UpsData *dst, const uint8_t *buffer, const report_desc_t *report_desc);
 bool ups_poll_all(UpsData *data);
 void init_hid_mapping();
 
 report_desc_t *report_mapping = NULL;
 uint8_t report_count = 0;
 
 /**
  * @brief HID Host event
  *
  * This event is used for delivering the HID Host event from callback to a task.
  */
 typedef struct
 {
     hid_host_device_handle_t hid_device_handle;
     hid_host_driver_event_t event;
     void *arg;
 } hid_host_event_queue_t;
  
 
 // HTTP Server handlers
 extern const char index_html_start[] asm("_binary_index_html_start");
 extern const char index_html_end[] asm("_binary_index_html_end");
 
// HTTP GET handler for root path
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

// HTTP GET handler for UPS data API
static esp_err_t api_ups_data_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/api/ups-data requested");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[128];

    // Start JSON
    httpd_resp_sendstr_chunk(req, "{");
 
     // Battery
     snprintf(buf, sizeof(buf),
              "\"battery.charge\":%u,",
              (unsigned)current_ups_data.battery_charge);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
          "\"ups.dev_connected\":%d,",
          UPS_DEV_CONNECTED ? 1 : 0);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.charge.low\":%u,",
              (unsigned)current_ups_data.battery_charge_low);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.charge.warning\":%u,",
              (unsigned)current_ups_data.battery_charge_warning);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.runtime\":%u,",
              (unsigned)current_ups_data.battery_runtime_s);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.runtime.low\":%u,",
              (unsigned)current_ups_data.battery_runtime_low_s);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.voltage\":%.1f,",
              current_ups_data.battery_voltage_dV > 30 ? current_ups_data.battery_voltage_dV * 0.0941f : (current_ups_data.battery_voltage_dV * 0.0941f) + current_ups_data.battery_voltage_nominal_dV / 10.0f);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"battery.voltage.nominal\":%.1f,",
              current_ups_data.battery_voltage_nominal_dV / 10.0f);
     httpd_resp_sendstr_chunk(req, buf);
 
     // Input
     snprintf(buf, sizeof(buf),
              "\"input.voltage\":%u,",
              (unsigned)current_ups_data.input_voltage_V);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"input.voltage.nominal\":%u,",
              (unsigned)current_ups_data.input_voltage_nominal_V);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"input.transfer.low\":%u,",
              (unsigned)current_ups_data.input_transfer_low_V);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"input.transfer.high\":%u,",
              (unsigned)current_ups_data.input_transfer_high_V);
     httpd_resp_sendstr_chunk(req, buf);
 
     // Output / UPS
     snprintf(buf, sizeof(buf),
              "\"output.voltage\":%u,",
              (unsigned)current_ups_data.output_voltage_V);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"ups.load\":%u,",
              (unsigned)current_ups_data.ups_load_percent);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"ups.realpower.nominal\":%u,",
              (unsigned)current_ups_data.realpower_nominal_W);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"ups.status\":\"%s\",",
              current_ups_data.status_ac_present ? "OL" : "OB");
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"ups.beeper.status\":\"%s\",",
              current_ups_data.beeper_mode == 2 ? "enabled" : "disabled");
     httpd_resp_sendstr_chunk(req, buf);
 
     // Status bits
     snprintf(buf, sizeof(buf),
              "\"status.ac_present\":%d,",
              current_ups_data.status_ac_present ? 1 : 0);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"status.charging\":%d,",
              current_ups_data.status_charging ? 1 : 0);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"status.discharging\":%d,",
              current_ups_data.status_discharging ? 1 : 0);
     httpd_resp_sendstr_chunk(req, buf);
 
     snprintf(buf, sizeof(buf),
              "\"status.overload\":%d,",
              current_ups_data.status_overload ? 1 : 0);
     httpd_resp_sendstr_chunk(req, buf);
 
    // Device / USB / Driver - static constants
    snprintf(buf, sizeof(buf),
             "\"device.mfr\":\"%s\",",
             "CyberPower");
    httpd_resp_sendstr_chunk(req, buf);

    snprintf(buf, sizeof(buf),
             "\"device.model\":\"%s\",",
             "CP1600EPFCLCD");
    httpd_resp_sendstr_chunk(req, buf);

    snprintf(buf, sizeof(buf),
             "\"ups.vendorid\":\"%s\",",
             "0764");
    httpd_resp_sendstr_chunk(req, buf);

    snprintf(buf, sizeof(buf),
             "\"ups.productid\":\"%s\",",
             "0601");
    httpd_resp_sendstr_chunk(req, buf);

    // Last field - no comma
    snprintf(buf, sizeof(buf),
             "\"driver.name\":\"%s\"",
             "usbhid-ups");
    httpd_resp_sendstr_chunk(req, buf);

    // End JSON
    httpd_resp_sendstr_chunk(req, "}");
    httpd_resp_sendstr_chunk(req, NULL);
 
    return ESP_OK;
}

// HTTP GET handler for Modbus configuration
static esp_err_t api_modbus_config_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/api/modbus-config GET requested");
    
    ModbusConfig config;
    esp_err_t err = modbus_config_load(&config);
    
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_send(req, "Failed to load configuration", -1);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"slave_addr\":%d,\"baudrate\":%ld,\"parity\":%d,\"stop_bits\":%d}",
             config.slave_addr, config.baudrate, config.parity, config.stop_bits);
    
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// HTTP POST handler for Modbus configuration
static esp_err_t api_modbus_config_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/api/modbus-config POST requested");
    
    char buf[256];
    int ret, remaining = req->content_len;
    
    if (remaining >= sizeof(buf)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Content too large", -1);
        return ESP_FAIL;
    }
    
    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    ESP_LOGI(TAG, "Received config: %s", buf);
    
    // Parse JSON manually (simple key=value parsing)
    ModbusConfig config;
    modbus_config_set_defaults(&config);
    
    // Parse slave_addr
    char *p = strstr(buf, "\"slave_addr\":");
    if (p) {
        config.slave_addr = atoi(p + 13);
    }
    
    // Parse baudrate
    p = strstr(buf, "\"baudrate\":");
    if (p) {
        config.baudrate = atol(p + 11);
    }
    
    // Parse parity
    p = strstr(buf, "\"parity\":");
    if (p) {
        config.parity = atoi(p + 9);
    }
    
    // Parse stop_bits
    p = strstr(buf, "\"stop_bits\":");
    if (p) {
        config.stop_bits = atoi(p + 12);
    }
    
    ESP_LOGI(TAG, "Parsed: addr=%d, baud=%ld, parity=%d, stop=%d",
             config.slave_addr, config.baudrate, config.parity, config.stop_bits);
    
    // Save configuration
    esp_err_t err = modbus_config_save(&config);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_send(req, "Failed to save configuration", -1);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"Configuration saved. Please restart device.\"}", -1);
    
    return ESP_OK;
}

// HTTP POST handler for device reboot
static esp_err_t api_reboot_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/api/reboot POST requested");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"Rebooting device...\"}", -1);
    
    // Delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Restarting device...");
    esp_restart();
    
    return ESP_OK;
}

// HTTP GET handler for WiFi Scan
static esp_err_t api_wifi_scan_handler(httpd_req_t *req)
{
    wifi_scan_result_t results[MAX_SCAN_LIST_SIZE];
    uint16_t count = 0;
    
    esp_err_t err = wifi_manager_scan(results, MAX_SCAN_LIST_SIZE, &count);
    
    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char buf[128];
    httpd_resp_sendstr_chunk(req, "[");
    
    for (int i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}%s",
                 results[i].ssid, results[i].rssi, results[i].authmode,
                 (i < count - 1) ? "," : "");
        httpd_resp_sendstr_chunk(req, buf);
    }
    
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// HTTP POST handler for WiFi Connect
static esp_err_t api_wifi_connect_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;
    
    if (remaining >= sizeof(buf)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    
    // Simple manual JSON parsing
    char ssid[33] = {0};
    char password[65] = {0};
    
    char *p_ssid = strstr(buf, "\"ssid\":\"");
    if (p_ssid) {
        p_ssid += 8;
        char *end = strchr(p_ssid, '"');
        if (end) {
            int len = end - p_ssid;
            if (len < 33) strncpy(ssid, p_ssid, len);
        }
    }
    
    char *p_pass = strstr(buf, "\"password\":\"");
    if (p_pass) {
        p_pass += 12;
        char *end = strchr(p_pass, '"');
        if (end) {
            int len = end - p_pass;
            if (len < 65) strncpy(password, p_pass, len);
        }
    }
    
    if (strlen(ssid) > 0) {
        wifi_manager_save_credentials(ssid, password);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"Saved. Rebooting...\"}", -1);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        httpd_resp_send_500(req);
    }
    
    return ESP_OK;
}

// HTTP GET handler for WiFi Status
static esp_err_t api_wifi_status_handler(httpd_req_t *req)
{
    wifi_status_t status;
    wifi_manager_get_status(&status);
    
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"connected\":%d,\"ap_mode\":%d,\"ssid\":\"%s\",\"ip\":\"%s\"}",
             status.is_connected, status.is_ap_mode, status.ssid, status.ip_addr);
             
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}


// HTTP server URI handlers
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_ups_data_uri = {
    .uri       = "/api/ups-data",
    .method    = HTTP_GET,
    .handler   = api_ups_data_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_modbus_config_get_uri = {
    .uri       = "/api/modbus-config",
    .method    = HTTP_GET,
    .handler   = api_modbus_config_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_modbus_config_post_uri = {
    .uri       = "/api/modbus-config",
    .method    = HTTP_POST,
    .handler   = api_modbus_config_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_reboot_post_uri = {
    .uri       = "/api/reboot",
    .method    = HTTP_POST,
    .handler   = api_reboot_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_wifi_scan_uri = {
    .uri       = "/api/wifi/scan",
    .method    = HTTP_GET,
    .handler   = api_wifi_scan_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_wifi_connect_uri = {
    .uri       = "/api/wifi/connect",
    .method    = HTTP_POST,
    .handler   = api_wifi_connect_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_wifi_status_uri = {
    .uri       = "/api/wifi/status",
    .method    = HTTP_GET,
    .handler   = api_wifi_status_handler,
    .user_ctx  = NULL
};
 
// Start HTTP web server
static httpd_handle_t start_webserver(void)
 {
     httpd_handle_t server = NULL;
     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
     config.lru_purge_enable = true;
     config.server_port = 80;
 
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &api_ups_data_uri);
        httpd_register_uri_handler(server, &api_modbus_config_get_uri);
        httpd_register_uri_handler(server, &api_modbus_config_post_uri);
        httpd_register_uri_handler(server, &api_modbus_config_post_uri);
        httpd_register_uri_handler(server, &api_reboot_post_uri);
        httpd_register_uri_handler(server, &api_wifi_scan_uri);
        httpd_register_uri_handler(server, &api_wifi_connect_uri);
        httpd_register_uri_handler(server, &api_wifi_status_uri);
        return server;
    }
 
     ESP_LOGE(TAG, "Error starting HTTP server!");
    return NULL;
}
 
 /**
  * @brief HID Host main task
  *
  * Creates queue and get new event from the queue
  *
  * @param[in] pvParameters Not used
  */
 void hid_host_task(void *pvParameters)
 {
     hid_host_event_queue_t evt_queue;
     // Create queue
     hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));
 
     // Wait queue
     while (!user_shutdown)
         {
         if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50)))
         {
             hid_host_device_event(evt_queue.hid_device_handle,
                                   evt_queue.event,
                                   evt_queue.arg);
         }
     }
 
     xQueueReset(hid_host_event_queue);
     vQueueDelete(hid_host_event_queue);
     vTaskDelete(NULL);
 }
 
 /**
  * @brief HID Host Device callback
  *
  * Puts new HID Device event to the queue
  *
  * @param[in] hid_device_handle HID Device handle
  * @param[in] event             HID Device event
  * @param[in] arg               Not used
  */
 void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                               const hid_host_driver_event_t event,
                               void *arg)
 {
     const hid_host_event_queue_t evt_queue = {
         .hid_device_handle = hid_device_handle,
         .event = event,
         .arg = arg};
     xQueueSend(hid_host_event_queue, &evt_queue, 0);
 }
 
 /**
  * @brief USB HID Host Device event
  *
  * @param[in] hid_device_handle  HID Device handle
  * @param[in] event              HID Host Device event
  * @param[in] arg                Pointer to arguments, does not used
  */
 void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                            const hid_host_driver_event_t event,
                            void *arg)
 {
     hid_host_dev_params_t dev_params;
     ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
 
     switch (event)
     {
     case HID_HOST_DRIVER_EVENT_CONNECTED:
         ESP_LOGI(TAG, "HID Device CONNECTED");
         
         const hid_host_device_config_t dev_config = {
             .callback = hid_host_interface_callback,
             .callback_arg = NULL};
             
         ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
         ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
         latest_hid_device_handle = hid_device_handle;
         UPS_DEV_CONNECTED = true;
         break;
         
     default:
         break;
     }
 }
 
 /**
  * @brief HID Host interface callback
  *
  * @param[in] hid_device_handle  HID Device handle
  * @param[in] event              HID Host interface event
  * @param[in] arg                Pointer to arguments, does not used
  */
 void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_interface_event_t event,
                                  void *arg)
 {
     uint8_t data[64] = {0};
     size_t data_length = 0;
     hid_host_dev_params_t dev_params;
     ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
 
     switch (event)
     {
     case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
         ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                                   data,
                                                                   64,
                                                                   &data_length));
 
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
        {
            // Keyboard/mouse processing - not needed for UPS
        }
        else
        {
            // Generic HID processing for UPS
            // UPS device data will be processed here
            ESP_LOGI(TAG, "Received HID data from UPS");
        }
         break;
         
     case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
         UPS_DEV_CONNECTED = false;
         ESP_LOGI(TAG, "HID Device DISCONNECTED");
         ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
         break;
         
     case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
         ESP_LOGI(TAG, "HID Device TRANSFER_ERROR");
         break;
         
     default:
         ESP_LOGE(TAG, "Unhandled HID event");
         break;
     }
 }
 
 /**
  * @brief USB Library task - handle USB host events
  */
 static void usb_lib_task(void *arg)
 {
     const gpio_config_t input_pin = {
         .pin_bit_mask = BIT64(APP_QUIT_PIN),
         .mode = GPIO_MODE_INPUT,
         .pull_up_en = GPIO_PULLUP_ENABLE,
     };
     ESP_ERROR_CHECK(gpio_config(&input_pin));
 
     const usb_host_config_t host_config = {
         .skip_phy_setup = false,
         .intr_flags = ESP_INTR_FLAG_LEVEL1,
     };
 
     ESP_ERROR_CHECK(usb_host_install(&host_config));
     xTaskNotifyGive(arg);
 
     while (true) {
         uint32_t event_flags;
         usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
 
         if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
             usb_host_device_free_all();
             ESP_LOGI(TAG, "USB Event: NO_CLIENTS");
         }
         if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
             ESP_LOGI(TAG, "USB Event: ALL_FREE");
         }
     }
     
     user_shutdown = true;
     ESP_LOGI(TAG, "USB shutdown");
     vTaskDelay(10);
     ESP_ERROR_CHECK(usb_host_uninstall());
     vTaskDelete(NULL);
 }
 
 /**
  * @brief Timer task - periodically update UPS status
  */
void timer_task(void *pvParameters)
{
    timer_queue_element_t evt_queue;

    while (true)
    {
        if (xQueueReceive(timer_queue, &evt_queue, pdMS_TO_TICKS(50)))
        {
            if (UPS_DEV_CONNECTED)
            {
                // Update UPS data
                if (ups_poll_all(&current_ups_data)) {
                    // Update Modbus registers <<<<<< ДОБАВЕТЕ ТОВА
                    modbus_ups_update_registers(&current_ups_data);
                    
                    // LED indication
                    if (current_ups_data.status_ac_present) {
                        led_strip_set_pixel(led_strip, 0, 0, 0x03, 0);  // Green - AC present
                    } else {
                        led_strip_set_pixel(led_strip, 0, 0x10, 0, 0);  // Red - on battery
                    }
                    led_strip_refresh(led_strip);
                    
                    // Debug log with all UPS data
                    ESP_LOGI(TAG, "Battery: %d%% (Low:%d%%, Warn:%d%%), Runtime: %ds, Load: %d%%, "
                                  "Input: %dV (Nom:%dV, Transfer:%d-%dV), Output: %dV, "
                                  "BatV: %.1fV (Nom:%.1fV, Raw:%d/%d), Power: %dW, "
                                  "Status: AC=%d CHG=%d DIS=%d OVL=%d, Beeper: %d",
                             current_ups_data.battery_charge,
                             current_ups_data.battery_charge_low,
                             current_ups_data.battery_charge_warning,
                             current_ups_data.battery_runtime_s,
                             current_ups_data.ups_load_percent,
                             current_ups_data.input_voltage_V,
                             current_ups_data.input_voltage_nominal_V,
                             current_ups_data.input_transfer_low_V,
                             current_ups_data.input_transfer_high_V,
                             current_ups_data.output_voltage_V,
                             current_ups_data.battery_voltage_dV > 30 ? current_ups_data.battery_voltage_dV * 0.0941f : (current_ups_data.battery_voltage_dV * 0.0941f) + (current_ups_data.battery_voltage_nominal_dV / 10.0f),
                             current_ups_data.battery_voltage_nominal_dV / 10.0,
                             current_ups_data.battery_voltage_dV,
                             current_ups_data.battery_voltage_nominal_dV,
                             current_ups_data.realpower_nominal_W,
                             current_ups_data.status_ac_present,
                             current_ups_data.status_charging,
                             current_ups_data.status_discharging,
                             current_ups_data.status_overload,
                             current_ups_data.beeper_mode);
                }
            }
            else
            {
                // UPS disconnected - orange LED
                led_strip_set_pixel(led_strip, 0, 0x08, 0x05, 0);
                led_strip_refresh(led_strip);
                ESP_LOGI(TAG, "UPS disconnected");
            }
        }
    }
    
    xQueueReset(timer_queue);
    vQueueDelete(timer_queue);
    vTaskDelete(NULL);
}

// == Timer ==

static bool IRAM_ATTR timer_on_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    timer_queue_element_t ele = {
        .event_count = edata->alarm_value};
    
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

// == HID Report Functions ==

// Read HID report from UPS device
bool ups_read_report(uint8_t report_id, uint8_t *buffer, size_t len) {
    if (!UPS_DEV_CONNECTED || !latest_hid_device_handle) {
        ESP_LOGW(TAG, "UPS not connected");
        return false;
    }
    
    // Find report_desc for this report_id
    report_desc_t *desc = NULL;
    for (uint8_t i = 0; i < report_count; i++) {
        if (report_mapping[i].report_id == report_id) {
            desc = &report_mapping[i];
            break;
        }
    }
    
    if (!desc) {
        ESP_LOGW(TAG, "Report ID %d not found in mapping (report_count=%d, report_mapping=%p)", 
                 report_id, report_count, report_mapping);
        return false;
    }
    
    // Check buffer length
    if (len < desc->length) {
        ESP_LOGW(TAG, "Buffer too small for report ID %d", report_id);
        return false;
    }
    
    // Call HID GET_REPORT
     size_t actual_len = desc->length;
     esp_err_t err = hid_class_request_get_report(
         latest_hid_device_handle,
         desc->report_type,
         report_id,
         buffer,
         &actual_len
     );
     
     if (err != ESP_OK) {
         ESP_LOGW(TAG, "Failed to read report ID %d: %s", report_id, esp_err_to_name(err));
         return false;
     }
     
    return true;
}

// Update UpsData structure from HID report buffer
void ups_update_data_from_buffer(UpsData *dst, const uint8_t *buffer, const report_desc_t *report_desc) {
    if (!dst || !buffer || !report_desc) {
        return;
    }
    
    for (uint8_t i = 0; i < report_desc->field_count; i++) {
        const hid_field_t *field = &report_desc->fields[i];
        
        // Parse according to field type
         switch (field->type) {
             case 0: // U8
                 {
                     uint8_t value = buffer[field->offset];
                     if (strstr(field->name, "battery.charge") && !strstr(field->name, "warning") && !strstr(field->name, "low")) {
                         dst->battery_charge = value;
                     } else if (strstr(field->name, "battery.design_capacity")) {
                         dst->battery_design_capacity = value;
                     } else if (strstr(field->name, "battery.charge.warning")) {
                         dst->battery_charge_warning = value;
                     } else if (strstr(field->name, "battery.charge.low")) {
                         dst->battery_charge_low = value;
                     } else if (strstr(field->name, "battery.voltage_dV")) {
                         dst->battery_voltage_dV = value;
                     } else if (strstr(field->name, "battery.voltage_nominal_dV")) {
                         dst->battery_voltage_nominal_dV = value;
                     } else if (strstr(field->name, "ups.load_percent")) {
                         dst->ups_load_percent = value;
                     } else if (strstr(field->name, "input.voltage_V") && !strstr(field->name, "nominal")) {
                         dst->input_voltage_V = value;
                     } else if (strstr(field->name, "input.voltage_nominal_V")) {
                         dst->input_voltage_nominal_V = value;
                     } else if (strstr(field->name, "output.voltage_V")) {
                         dst->output_voltage_V = value;
                     } else if (strstr(field->name, "ups.beeper.mode")) {
                         dst->beeper_mode = value;
                     } else if (strstr(field->name, "ups.test.code")) {
                         dst->test_code = value;
                     }
                 }
                 break;
                 
             case 1: // U16_LE
                 {
                     uint16_t value = buffer[field->offset] | (buffer[field->offset + 1] << 8);
                     if (strstr(field->name, "battery.runtime_s")) {
                         dst->battery_runtime_s = value;
                     } else if (strstr(field->name, "battery.runtime_low")) {
                         dst->battery_runtime_low_s = value;
                     } else if (strstr(field->name, "input.transfer_low_V")) {
                         dst->input_transfer_low_V = value;
                     } else if (strstr(field->name, "input.transfer_high_V")) {
                         dst->input_transfer_high_V = value;
                     } else if (strstr(field->name, "ups.realpower_nominal_W")) {
                         dst->realpower_nominal_W = value;
                     }
                 }
                 break;
                 
             case 2: // I16_LE
                 {
                     int16_t value = (int16_t)(buffer[field->offset] | (buffer[field->offset + 1] << 8));
                     if (strstr(field->name, "ups.timer_shutdown_s")) {
                         dst->timer_shutdown_s = value;
                     } else if (strstr(field->name, "ups.timer_start_s")) {
                         dst->timer_start_s = value;
                     }
                 }
                 break;
                 
             case 3: // BOOL
                 {
                     uint8_t byte_val = buffer[field->offset];
                     bool bit_val = (byte_val & (1 << field->bit)) != 0;
                     
                     if (strstr(field->name, "status.ac_present")) {
                         dst->status_ac_present = bit_val;
                     } else if (strstr(field->name, "status.charging")) {
                         dst->status_charging = bit_val;
                     } else if (strstr(field->name, "status.discharging")) {
                         dst->status_discharging = bit_val;
                     } else if (strstr(field->name, "status.below_capacity_limit")) {
                         dst->status_below_capacity_limit = bit_val;
                     } else if (strstr(field->name, "status.fully_charged")) {
                         dst->status_fully_charged = bit_val;
                     } else if (strstr(field->name, "status.time_limit_expired")) {
                         dst->status_time_limit_expired = bit_val;
                     } else if (strstr(field->name, "status.boost")) {
                         dst->status_boost = bit_val;
                     } else if (strstr(field->name, "status.overload")) {
                         dst->status_overload = bit_val;
                     }
                 }
                 break;
         }
    }
}

// Read all important HID reports from UPS
bool ups_poll_all(UpsData *data) {
    if (!data) {
        return false;
    }
    
    // List of important report IDs from protocol.json
    const uint8_t important_reports[] = {7, 8, 9, 10, 11, 14, 15, 16, 18, 19, 20, 21, 22, 23, 24};
    const uint8_t report_count_local = sizeof(important_reports) / sizeof(important_reports[0]);
    
    uint8_t buffer[64];
    bool all_ok = true;
    
    for (uint8_t i = 0; i < report_count_local; i++) {
        uint8_t report_id = important_reports[i];
        
        memset(buffer, 0, sizeof(buffer));
        if (ups_read_report(report_id, buffer, sizeof(buffer))) {
            // Find report description
             for (uint8_t j = 0; j < report_count; j++) {
                 if (report_mapping[j].report_id == report_id) {
                     ups_update_data_from_buffer(data, buffer, &report_mapping[j]);
                     break;
                 }
             }
         } else {
             ESP_LOGW(TAG, "Failed to read report ID %d", report_id);
             all_ok = false;
         }
     }
     
    return all_ok;
}

// Initialize HID mapping from protocol.json
void init_hid_mapping() {
    ESP_LOGI(TAG, "Initializing HID mapping...");
    
    // For simplicity, define mapping statically
    // In real implementation, could be loaded from SPIFFS or embedded as const string
    
    // Full static definition of all important reports from protocol.json
     static hid_field_t fields_7[] = {
         {"battery.design_capacity", 1, 1, 0, 0},     // U8
         {"battery.charge.warning", 4, 1, 0, 0},      // U8
         {"battery.charge.low", 5, 1, 0, 0}           // U8
     };
     
     static hid_field_t fields_8[] = {
         {"battery.charge", 1, 1, 0, 0},              // U8
         {"battery.runtime_s", 2, 2, 0, 1},           // U16_LE
         {"battery.runtime_low", 4, 2, 0, 1}          // U16_LE
     };
     
     static hid_field_t fields_9[] = {
         {"battery.voltage_nominal_dV", 1, 1, 0, 0}   // U8
     };
     
     static hid_field_t fields_10[] = {
         {"battery.voltage_dV", 1, 1, 0, 0}           // U8
     };
     
     static hid_field_t fields_11[] = {
         {"status.ac_present", 1, 0, 0, 3},           // BOOL, bit 0
         {"status.charging", 1, 0, 1, 3},             // BOOL, bit 1
         {"status.discharging", 1, 0, 2, 3},          // BOOL, bit 2
         {"status.below_capacity_limit", 1, 0, 3, 3}, // BOOL, bit 3
         {"status.fully_charged", 1, 0, 4, 3}         // BOOL, bit 4
     };
     
     static hid_field_t fields_12[] = {
         {"ups.beeper.mode", 1, 1, 0, 0}              // U8
     };
     
     static hid_field_t fields_14[] = {
         {"input.voltage_nominal_V", 1, 1, 0, 0}      // U8
     };
     
     static hid_field_t fields_15[] = {
         {"input.voltage_V", 1, 1, 0, 0}              // U8
     };
     
     static hid_field_t fields_16[] = {
         {"input.transfer_low_V", 1, 2, 0, 1},        // U16_LE
         {"input.transfer_high_V", 3, 2, 0, 1}        // U16_LE
     };
     
     static hid_field_t fields_18[] = {
         {"output.voltage_V", 1, 1, 0, 0}             // U8
     };
     
     static hid_field_t fields_19[] = {
         {"ups.load_percent", 1, 1, 0, 0}             // U8
     };
     
     static hid_field_t fields_20[] = {
         {"ups.test.code", 1, 1, 0, 0}                // U8
     };
     
     static hid_field_t fields_21[] = {
         {"ups.timer_shutdown_s", 1, 2, 0, 2}         // I16_LE
     };
     
     static hid_field_t fields_22[] = {
         {"ups.timer_start_s", 1, 2, 0, 2}            // I16_LE
     };
     
     static hid_field_t fields_23[] = {
         {"status.boost", 1, 0, 0, 3},                // BOOL, bit 0
         {"status.overload", 1, 0, 1, 3}              // BOOL, bit 1
     };
     
    static hid_field_t fields_24[] = {
        {"ups.realpower_nominal_W_raw", 1, 2, 0, 1}  // U16_LE
    };
    
    // Allocate mapping array for all 16 important reports (including 24)
    report_count = 16;
    report_mapping = (report_desc_t*)malloc(report_count * sizeof(report_desc_t));
    
    if (!report_mapping) {
        ESP_LOGE(TAG, "Failed to allocate report mapping");
        return;
    }
    
    // Define all reports according to protocol.json
     report_mapping[0] = (report_desc_t){7, 3, 7, fields_7, 3};      // battery_capacity_thresholds
     report_mapping[1] = (report_desc_t){8, 3, 6, fields_8, 3};      // battery_charge_and_runtime
     report_mapping[2] = (report_desc_t){9, 3, 2, fields_9, 1};      // battery_nominal_voltage
     report_mapping[3] = (report_desc_t){10, 3, 2, fields_10, 1};    // battery_voltage
     report_mapping[4] = (report_desc_t){11, 1, 2, fields_11, 5};    // power_summary_status_bits
     report_mapping[5] = (report_desc_t){12, 3, 2, fields_12, 1};    // beeper_mode
     report_mapping[6] = (report_desc_t){14, 3, 2, fields_14, 1};    // input_nominal_voltage
     report_mapping[7] = (report_desc_t){15, 3, 2, fields_15, 1};    // input_voltage
     report_mapping[8] = (report_desc_t){16, 3, 5, fields_16, 2};    // input_transfer_limits
     report_mapping[9] = (report_desc_t){18, 3, 2, fields_18, 1};    // output_voltage
     report_mapping[10] = (report_desc_t){19, 3, 2, fields_19, 1};   // output_percent_load
     report_mapping[11] = (report_desc_t){20, 3, 2, fields_20, 1};   // battery_test_status
     report_mapping[12] = (report_desc_t){21, 3, 3, fields_21, 1};   // delay_before_shutdown
     report_mapping[13] = (report_desc_t){22, 3, 3, fields_22, 1};   // delay_before_startup
     report_mapping[14] = (report_desc_t){23, 3, 2, fields_23, 2};   // output_flags_boost_overload
     report_mapping[15] = (report_desc_t){24, 3, 3, fields_24, 1};   // config_active_power
     
    ESP_LOGI(TAG, "HID mapping initialized with %d reports", report_count);
    
    // Log all report IDs
    ESP_LOGI(TAG, "Available report IDs:");
    for (uint8_t i = 0; i < report_count; i++) {
        ESP_LOGI(TAG, "  [%d] Report ID %d (type: %d, length: %d, fields: %d)", 
                 i, report_mapping[i].report_id, report_mapping[i].report_type,
                 report_mapping[i].length, report_mapping[i].field_count);
    }
}

 static void configure_led(void)
 {
     /* LED strip initialization with the GPIO and pixels number*/
     led_strip_config_t strip_config = {
         .strip_gpio_num = RGB_LED_PIN,
         .max_leds = 1, // at least one LED on board
     };
     led_strip_rmt_config_t rmt_config = {
         .resolution_hz = 10 * 1000 * 1000, // 10MHz
     };
     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
     /* Set all LED off to clear all pixels */
     led_strip_clear(led_strip);
 }
 
 void app_main(void)
 {
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ESP_ERROR_CHECK(nvs_flash_init());
     } else {
         ESP_ERROR_CHECK(ret);
     }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Configure LED (must be done before WiFi)
    configure_led();
    
    // LED indication - purple for initialization
    led_strip_set_pixel(led_strip, 0, 0x08, 0, 0x08);
    led_strip_refresh(led_strip);
    
    // WiFi initialization (STA with AP fallback)
    ESP_LOGI(TAG, "Initializing WiFi Manager...");
    wifi_manager_init();
    
    // Start HTTP web server
    httpd_handle_t server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
    }

    // Initialize HID mapping
    init_hid_mapping();
    
    // Initialize current_ups_data
    memset(&current_ups_data, 0, sizeof(current_ups_data));

    // Initialize Modbus RTU slave <<<<<< ДОБАВЕТЕ ТОВА
    ESP_LOGI(TAG, "Initializing Modbus RTU slave...");
    esp_err_t mb_err = modbus_ups_slave_init();
    if (mb_err != ESP_OK) {
        ESP_LOGE(TAG, "Modbus initialization failed: %s", esp_err_to_name(mb_err));
        ESP_LOGW(TAG, "Continuing without Modbus functionality");
    }

    // == Timer Setup ==
     // Create queue
     timer_queue = xQueueCreate(10, sizeof(timer_queue_element_t));
     gptimer_handle_t gptimer = NULL;
     gptimer_config_t timer_config = {
         .clk_src = GPTIMER_CLK_SRC_DEFAULT,
         .direction = GPTIMER_COUNT_UP,
         .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
     };
     ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
 
     gptimer_alarm_config_t alarm_config = {
         .reload_count = 0,                  // counter will reload with 0 on alarm event
         .alarm_count = 1000000,             // period = 1s @resolution 1MHz
         .flags.auto_reload_on_alarm = true, // enable auto-reload
     };
     ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
 
     gptimer_event_callbacks_t cbs = {
         .on_alarm = timer_on_alarm_callback,
     };
     ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue));
     ESP_ERROR_CHECK(gptimer_enable(gptimer));
     ESP_ERROR_CHECK(gptimer_start(gptimer));
 
     BaseType_t task_created;
 
     /*
      * Create usb_lib_task to:
      * - initialize USB Host library
      * - Handle USB Host events while APP pin in in HIGH state
      */
     task_created = xTaskCreatePinnedToCore(usb_lib_task,
                                            "usb_events",
                                            4096,
                                            xTaskGetCurrentTaskHandle(),
                                            2, NULL, 0);
     assert(task_created == pdTRUE);
 
     // Wait for notification from usb_lib_task to proceed
     ulTaskNotifyTake(false, 1000);
 
     /*
      * HID host driver configuration
      * - create background task for handling low level event inside the HID driver
      * - provide the device callback to get new HID Device connection event
      */
     const hid_host_driver_config_t hid_host_driver_config = {
         .create_background_task = true,
         .task_priority = 5,
         .stack_size = 4096,
         .core_id = 0,
         .callback = hid_host_device_callback,
         .callback_arg = NULL};
 
     ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
 
     // Task is working until the devices are gone (while 'user_shutdown' if false)
     user_shutdown = false;
 
     /*
      * Create HID Host task process for handle events
      * IMPORTANT: Task is necessary here while there is no possibility to interact
      * with USB device from the callback.
      */
     task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
 
     // to receive queue sent from timer, and the actual thing is done at timer_task (for example, recheck ups status)
     task_created = xTaskCreate(&timer_task, "timer_task", 4 * 1024, NULL, 8, NULL);
     assert(task_created == pdTRUE);
 }