#pragma once

#include "esp_err.h"
#include "esp_wifi.h"

// Maximum number of networks to scan
#define MAX_SCAN_LIST_SIZE 20

// WiFi Status Structure
typedef struct {
    bool is_connected;
    bool is_ap_mode;
    char ssid[33];
    char ip_addr[16];
} wifi_status_t;

// WiFi Scan Result Structure
typedef struct {
    char ssid[33];
    int8_t rssi;
    wifi_auth_mode_t authmode;
} wifi_scan_result_t;

/**
 * @brief Initialize WiFi Manager
 * Attempts to connect to saved WiFi. If fails, starts AP mode.
 */
void wifi_manager_init(void);

/**
 * @brief Save WiFi credentials to NVS
 * @param ssid SSID
 * @param password Password
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password);

/**
 * @brief Scan for available networks
 * @param results Pointer to array to store results
 * @param size Size of the array (max results)
 * @param count Pointer to store number of found networks
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_scan(wifi_scan_result_t *results, uint16_t size, uint16_t *count);

/**
 * @brief Get current WiFi status
 * @param status Pointer to status structure
 */
void wifi_manager_get_status(wifi_status_t *status);
