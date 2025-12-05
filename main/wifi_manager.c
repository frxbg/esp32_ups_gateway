#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "nvs.h"

#include "wifi_manager.h"

static const char *TAG = "wifi_manager";

// Default AP Configuration
#define DEFAULT_AP_SSID "UPS_Gateway_AP"
#define DEFAULT_AP_PASS "admin1234"
#define DEFAULT_AP_CHANNEL 1
#define DEFAULT_AP_MAX_CONN 4

// Event Group Bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_is_ap_mode = false;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < 5) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " joined, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac),
             event->aid);
  }
}

static void start_ap_mode(void) {
  ESP_LOGI(TAG, "Starting AP Mode (APSTA)");
  s_is_ap_mode = true;

  wifi_config_t ap_config = {
      .ap = {.ssid = DEFAULT_AP_SSID,
             .ssid_len = strlen(DEFAULT_AP_SSID),
             .channel = DEFAULT_AP_CHANNEL,
             .password = DEFAULT_AP_PASS,
             .max_connection = DEFAULT_AP_MAX_CONN,
             .authmode = WIFI_AUTH_WPA2_PSK},
  };

  if (strlen(DEFAULT_AP_PASS) == 0) {
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  // Configure STA as well (empty) to allow scanning
  wifi_config_t sta_config = {
      .sta =
          {
              .ssid = "",
              .password = "",
              .scan_method = WIFI_ALL_CHANNEL_SCAN,
          },
  };

  // Stop WiFi to reset state machine (in case STA is connecting)
  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_NULL);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "AP Started. SSID: %s, Password: %s", DEFAULT_AP_SSID,
           DEFAULT_AP_PASS);
}

void wifi_manager_init(void) {
  s_wifi_event_group = xEventGroupCreate();

  // NVS is already initialized in main, but good to be safe if reused
  // esp_err_t ret = nvs_flash_init(); ...

  ESP_ERROR_CHECK(esp_netif_init());
  // Event loop created in main

  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  // Load credentials
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);

  char ssid[33] = {0};
  char password[65] = {0};
  size_t ssid_len = sizeof(ssid);
  size_t pass_len = sizeof(password);

  bool credentials_found = false;

  if (err == ESP_OK) {
    if (nvs_get_str(my_handle, "wifi_ssid", ssid, &ssid_len) == ESP_OK &&
        nvs_get_str(my_handle, "wifi_pass", password, &pass_len) == ESP_OK) {
      credentials_found = true;
      ESP_LOGI(TAG, "Found saved credentials for SSID: %s", ssid);
    }
    nvs_close(my_handle);
  }

  if (credentials_found) {
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password,
            sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(10000)); // 10 seconds timeout

    if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(TAG, "Connected to AP SSID:%s", ssid);
      s_is_ap_mode = false;
    } else if (bits & WIFI_FAIL_BIT) {
      ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid);
      start_ap_mode();
    } else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
      start_ap_mode();
    }
  } else {
    ESP_LOGI(TAG, "No saved credentials, starting AP mode");
    start_ap_mode();
  }
}

esp_err_t wifi_manager_save_credentials(const char *ssid,
                                        const char *password) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;

  err = nvs_set_str(my_handle, "wifi_ssid", ssid);
  if (err != ESP_OK) {
    nvs_close(my_handle);
    return err;
  }

  err = nvs_set_str(my_handle, "wifi_pass", password);
  if (err != ESP_OK) {
    nvs_close(my_handle);
    return err;
  }

  err = nvs_commit(my_handle);
  nvs_close(my_handle);
  return err;
}

esp_err_t wifi_manager_scan(wifi_scan_result_t *results, uint16_t size,
                            uint16_t *count) {
  wifi_scan_config_t scan_config = {
      .ssid = 0, .bssid = 0, .channel = 0, .show_hidden = true};

  // Scan works in STA or APSTA mode.
  // We assume start_ap_mode now sets APSTA, so we are good.

  ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true)); // Blocking scan

  uint16_t ap_count = 0;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

  if (ap_count > size) {
    ap_count = size;
  }

  wifi_ap_record_t *ap_list =
      (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
  if (!ap_list) {
    return ESP_ERR_NO_MEM;
  }

  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));

  for (int i = 0; i < ap_count; i++) {
    strncpy(results[i].ssid, (char *)ap_list[i].ssid, 33);
    results[i].rssi = ap_list[i].rssi;
    results[i].authmode = ap_list[i].authmode;
  }

  *count = ap_count;
  free(ap_list);

  return ESP_OK;
}

void wifi_manager_get_status(wifi_status_t *status) {
  if (!status)
    return;

  status->is_ap_mode = s_is_ap_mode;

  if (s_is_ap_mode) {
    strncpy(status->ssid, DEFAULT_AP_SSID, sizeof(status->ssid));
    status->is_connected = true; // AP is always "connected"

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif) {
      esp_netif_get_ip_info(netif, &ip_info);
      sprintf(status->ip_addr, IPSTR, IP2STR(&ip_info.ip));
    }
  } else {
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      status->is_connected = true;
      strncpy(status->ssid, (char *)ap_info.ssid, sizeof(status->ssid));

      esp_netif_ip_info_t ip_info;
      esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      if (netif) {
        esp_netif_get_ip_info(netif, &ip_info);
        sprintf(status->ip_addr, IPSTR, IP2STR(&ip_info.ip));
      }
    } else {
      status->is_connected = false;
      strcpy(status->ssid, "");
      strcpy(status->ip_addr, "0.0.0.0");
    }
  }
}
