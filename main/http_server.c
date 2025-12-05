#include "http_server.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_system.h"
#include "modbus_ups.h"
#include "ups_data.h"
#include "wifi_manager.h"

static const char *TAG = "http_server";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

// HTTP GET handler for root path
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
  return ESP_OK;
}

// HTTP GET handler for UPS data API
static esp_err_t api_ups_data_handler(httpd_req_t *req) {
  // ESP_LOGI(TAG, "/api/ups-data requested"); // Reduced logging

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // Protect data access
  if (xSemaphoreTake(ups_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Battery
    cJSON_AddNumberToObject(root, "battery.charge",
                            current_ups_data.battery_charge);
    cJSON_AddBoolToObject(root, "ups.dev_connected", UPS_DEV_CONNECTED);
    cJSON_AddNumberToObject(root, "battery.charge.low",
                            current_ups_data.battery_charge_low);
    cJSON_AddNumberToObject(root, "battery.charge.warning",
                            current_ups_data.battery_charge_warning);
    cJSON_AddNumberToObject(root, "battery.runtime",
                            current_ups_data.battery_runtime_s);
    cJSON_AddNumberToObject(root, "battery.runtime.low",
                            current_ups_data.battery_runtime_low_s);

    float bat_voltage =
        current_ups_data.battery_voltage_dV > 30
            ? current_ups_data.battery_voltage_dV * 0.0941f
            : (current_ups_data.battery_voltage_dV * 0.0941f) +
                  current_ups_data.battery_voltage_nominal_dV / 10.0f;
    cJSON_AddNumberToObject(root, "battery.voltage", bat_voltage);
    cJSON_AddNumberToObject(root, "battery.voltage.nominal",
                            current_ups_data.battery_voltage_nominal_dV /
                                10.0f);

    // Input
    cJSON_AddNumberToObject(root, "input.voltage",
                            current_ups_data.input_voltage_V);
    cJSON_AddNumberToObject(root, "input.voltage.nominal",
                            current_ups_data.input_voltage_nominal_V);
    cJSON_AddNumberToObject(root, "input.transfer.low",
                            current_ups_data.input_transfer_low_V);
    cJSON_AddNumberToObject(root, "input.transfer.high",
                            current_ups_data.input_transfer_high_V);

    // Output / UPS
    cJSON_AddNumberToObject(root, "output.voltage",
                            current_ups_data.output_voltage_V);
    cJSON_AddNumberToObject(root, "ups.load",
                            current_ups_data.ups_load_percent);
    cJSON_AddNumberToObject(root, "ups.realpower.nominal",
                            current_ups_data.realpower_nominal_W);
    cJSON_AddStringToObject(root, "ups.status",
                            current_ups_data.status_ac_present ? "OL" : "OB");
    cJSON_AddStringToObject(root, "ups.beeper.status",
                            current_ups_data.beeper_mode == 2 ? "enabled"
                                                              : "disabled");

    // Status bits
    cJSON_AddBoolToObject(root, "status.ac_present",
                          current_ups_data.status_ac_present);
    cJSON_AddBoolToObject(root, "status.charging",
                          current_ups_data.status_charging);
    cJSON_AddBoolToObject(root, "status.discharging",
                          current_ups_data.status_discharging);
    cJSON_AddBoolToObject(root, "status.overload",
                          current_ups_data.status_overload);

    xSemaphoreGive(ups_data_mutex);
  } else {
    cJSON_Delete(root);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // Device / USB / Driver - static constants
  cJSON_AddStringToObject(root, "device.mfr", "CyberPower");
  cJSON_AddStringToObject(root, "device.model", "CP1600EPFCLCD");
  cJSON_AddStringToObject(root, "ups.vendorid", "0764");
  cJSON_AddStringToObject(root, "ups.productid", "0601");
  cJSON_AddStringToObject(root, "driver.name", "usbhid-ups");

  const char *json_str = cJSON_PrintUnformatted(root);
  if (json_str) {
    httpd_resp_sendstr(req, json_str);
    free((void *)json_str);
  } else {
    httpd_resp_send_500(req);
  }

  cJSON_Delete(root);
  return ESP_OK;
}

// HTTP GET handler for Modbus configuration
static esp_err_t api_modbus_config_get_handler(httpd_req_t *req) {
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
  snprintf(
      buf, sizeof(buf),
      "{\"slave_addr\":%d,\"baudrate\":%ld,\"parity\":%d,\"stop_bits\":%d}",
      config.slave_addr, config.baudrate, config.parity, config.stop_bits);

  httpd_resp_send(req, buf, strlen(buf));
  return ESP_OK;
}

// HTTP POST handler for Modbus configuration
static esp_err_t api_modbus_config_post_handler(httpd_req_t *req) {
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

  // Parse JSON using cJSON
  cJSON *root = cJSON_Parse(buf);
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to parse JSON");
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_send(req, "Invalid JSON", -1);
    return ESP_FAIL;
  }

  ModbusConfig config;
  modbus_config_set_defaults(&config);

  // Parse slave_addr
  cJSON *slave_addr = cJSON_GetObjectItem(root, "slave_addr");
  if (cJSON_IsNumber(slave_addr)) {
    config.slave_addr = slave_addr->valueint;
  }

  // Parse baudrate
  cJSON *baudrate = cJSON_GetObjectItem(root, "baudrate");
  if (cJSON_IsNumber(baudrate)) {
    config.baudrate = baudrate->valueint;
  }

  // Parse parity
  cJSON *parity = cJSON_GetObjectItem(root, "parity");
  if (cJSON_IsNumber(parity)) {
    config.parity = parity->valueint;
  }

  // Parse stop_bits
  cJSON *stop_bits = cJSON_GetObjectItem(root, "stop_bits");
  if (cJSON_IsNumber(stop_bits)) {
    config.stop_bits = stop_bits->valueint;
  }

  cJSON_Delete(root);

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
  httpd_resp_send(req,
                  "{\"status\":\"ok\",\"message\":\"Configuration saved. "
                  "Please restart device.\"}",
                  -1);

  return ESP_OK;
}

// HTTP POST handler for device reboot
static esp_err_t api_reboot_post_handler(httpd_req_t *req) {
  ESP_LOGI(TAG, "/api/reboot POST requested");

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(
      req, "{\"status\":\"ok\",\"message\":\"Rebooting device...\"}", -1);

  // Delay to allow response to be sent
  vTaskDelay(pdMS_TO_TICKS(500));

  ESP_LOGI(TAG, "Restarting device...");
  esp_restart();

  return ESP_OK;
}

// HTTP GET handler for WiFi Scan
static esp_err_t api_wifi_scan_handler(httpd_req_t *req) {
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
static esp_err_t api_wifi_connect_handler(httpd_req_t *req) {
  char buf[128];
  int ret, remaining = req->content_len;

  if (remaining >= sizeof(buf)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  ret = httpd_req_recv(req, buf, remaining);
  if (ret <= 0)
    return ESP_FAIL;
  buf[ret] = '\0';

  // Parse JSON using cJSON
  cJSON *root = cJSON_Parse(buf);
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to parse JSON");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  char ssid[33] = {0};
  char password[65] = {0};

  cJSON *ssid_item = cJSON_GetObjectItem(root, "ssid");
  if (cJSON_IsString(ssid_item) && ssid_item->valuestring != NULL) {
    strncpy(ssid, ssid_item->valuestring, sizeof(ssid) - 1);
  }

  cJSON *password_item = cJSON_GetObjectItem(root, "password");
  if (cJSON_IsString(password_item) && password_item->valuestring != NULL) {
    strncpy(password, password_item->valuestring, sizeof(password) - 1);
  }

  cJSON_Delete(root);

  if (strlen(ssid) > 0) {
    wifi_manager_save_credentials(ssid, password);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(
        req, "{\"status\":\"ok\",\"message\":\"Saved. Rebooting...\"}", -1);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
  } else {
    httpd_resp_send_500(req);
  }

  return ESP_OK;
}

// HTTP GET handler for WiFi Status
static esp_err_t api_wifi_status_handler(httpd_req_t *req) {
  wifi_status_t status;
  wifi_manager_get_status(&status);

  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"connected\":%d,\"ap_mode\":%d,\"ssid\":\"%s\",\"ip\":\"%s\"}",
           status.is_connected, status.is_ap_mode, status.ssid, status.ip_addr);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, buf, strlen(buf));
  return ESP_OK;
}

// HTTP server URI handlers
static const httpd_uri_t root_uri = {.uri = "/",
                                     .method = HTTP_GET,
                                     .handler = root_get_handler,
                                     .user_ctx = NULL};

static const httpd_uri_t api_ups_data_uri = {.uri = "/api/ups-data",
                                             .method = HTTP_GET,
                                             .handler = api_ups_data_handler,
                                             .user_ctx = NULL};

static const httpd_uri_t api_modbus_config_get_uri = {
    .uri = "/api/modbus-config",
    .method = HTTP_GET,
    .handler = api_modbus_config_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t api_modbus_config_post_uri = {
    .uri = "/api/modbus-config",
    .method = HTTP_POST,
    .handler = api_modbus_config_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t api_reboot_post_uri = {.uri = "/api/reboot",
                                                .method = HTTP_POST,
                                                .handler =
                                                    api_reboot_post_handler,
                                                .user_ctx = NULL};

static const httpd_uri_t api_wifi_scan_uri = {.uri = "/api/wifi/scan",
                                              .method = HTTP_GET,
                                              .handler = api_wifi_scan_handler,
                                              .user_ctx = NULL};

static const httpd_uri_t api_wifi_connect_uri = {.uri = "/api/wifi/connect",
                                                 .method = HTTP_POST,
                                                 .handler =
                                                     api_wifi_connect_handler,
                                                 .user_ctx = NULL};

static const httpd_uri_t api_wifi_status_uri = {.uri = "/api/wifi/status",
                                                .method = HTTP_GET,
                                                .handler =
                                                    api_wifi_status_handler,
                                                .user_ctx = NULL};

// Start HTTP web server
httpd_handle_t http_server_init(void) {
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
    httpd_register_uri_handler(server, &api_reboot_post_uri);
    httpd_register_uri_handler(server, &api_wifi_scan_uri);
    httpd_register_uri_handler(server, &api_wifi_connect_uri);
    httpd_register_uri_handler(server, &api_wifi_status_uri);
    return server;
  }

  ESP_LOGE(TAG, "Error starting HTTP server!");
  return NULL;
}
