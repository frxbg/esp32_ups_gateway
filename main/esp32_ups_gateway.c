/*
 * A complete gateway solution for monitoring CyberPower UPS devices via USB HID
 * and providing data access through HTTP web interface and ModbusRTU protocol.
 */

// Standard C libraries
#include <stdbool.h>

#include <string.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"

#include "freertos/semphr.h"
#include "freertos/task.h"

// ESP-IDF core
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs_flash.h"

// Network and WiFi
#include "esp_netif.h"

// Peripherals
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "led_strip.h"

// Modules
#include "hid_worker.h"
#include "http_server.h"
#include "modbus_ups.h"
#include "ups_data.h"
#include "wifi_manager.h"

#define RGB_LED_PIN GPIO_NUM_48

static led_strip_handle_t led_strip;
static const char *TAG = "ups_main";

// Global Data
UpsData current_ups_data;
SemaphoreHandle_t ups_data_mutex = NULL;

static void configure_led(void) {
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = RGB_LED_PIN,
      .max_leds = 1, // at least one LED on board
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

// == Timer ==

static bool IRAM_ATTR timer_on_alarm_callback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
    void *user_ctx) {
  BaseType_t high_task_awoken = pdFALSE;
  TaskHandle_t task_handle = (TaskHandle_t)user_ctx;

  vTaskNotifyGiveFromISR(task_handle, &high_task_awoken);
  return high_task_awoken == pdTRUE;
}

/**
 * @brief Timer task - periodically update UPS status
 */
void timer_task(void *pvParameters) {
  while (true) {
    // Wait for notification from timer ISR
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0) {
      if (hid_worker_is_connected()) {
        // Update UPS data using double buffering to minimize lock time
        UpsData next_state = current_ups_data;

        if (hid_worker_poll_ups(&next_state)) {
          // Update Modbus registers
          xSemaphoreTake(ups_data_mutex, portMAX_DELAY);
          current_ups_data = next_state;
          modbus_ups_update_registers(&current_ups_data);
          xSemaphoreGive(ups_data_mutex);

          // LED indication
          if (current_ups_data.status_ac_present) {
            led_strip_set_pixel(led_strip, 0, 0, 0x03, 0); // Green - AC present
          } else {
            led_strip_set_pixel(led_strip, 0, 0x10, 0, 0); // Red - on battery
          }
          led_strip_refresh(led_strip);

          // Debug log with all UPS data
          ESP_LOGI(
              TAG,
              "Battery: %d%% (Low:%d%%, Warn:%d%%), Runtime: %ds, Load: %d%%, "
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
              current_ups_data.battery_voltage_dV > 30
                  ? current_ups_data.battery_voltage_dV * 0.0941f
                  : (current_ups_data.battery_voltage_dV * 0.0941f) +
                        (current_ups_data.battery_voltage_nominal_dV / 10.0f),
              current_ups_data.battery_voltage_nominal_dV / 10.0,
              current_ups_data.battery_voltage_dV,
              current_ups_data.battery_voltage_nominal_dV,
              current_ups_data.realpower_nominal_W,
              current_ups_data.status_ac_present,
              current_ups_data.status_charging,
              current_ups_data.status_discharging,
              current_ups_data.status_overload, current_ups_data.beeper_mode);
        }
      } else {
        // UPS disconnected - orange LED
        led_strip_set_pixel(led_strip, 0, 0x08, 0x05, 0);
        led_strip_refresh(led_strip);
        ESP_LOGI(TAG, "UPS disconnected");
      }
    }
  }
  vTaskDelete(NULL);
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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

  // Initialize current_ups_data
  memset(&current_ups_data, 0, sizeof(current_ups_data));

  // Initialize Mutex
  ups_data_mutex = xSemaphoreCreateMutex();
  assert(ups_data_mutex != NULL);

  // Start HTTP web server
  httpd_handle_t server = http_server_init();
  if (server == NULL) {
    ESP_LOGE(TAG, "Failed to start web server");
  }

  // Initialize Modbus RTU slave
  ESP_LOGI(TAG, "Initializing Modbus RTU slave...");
  esp_err_t mb_err = modbus_ups_slave_init();
  if (mb_err != ESP_OK) {
    ESP_LOGE(TAG, "Modbus initialization failed: %s", esp_err_to_name(mb_err));
    ESP_LOGW(TAG, "Continuing without Modbus functionality");
  }

  // Initialize HID Worker
  ESP_LOGI(TAG, "Initializing HID Worker...");
  hid_worker_init();

  // == Timer Setup ==
  // Create timer_task FIRST so we have the handle for the callback
  TaskHandle_t timer_task_handle = NULL;
  xTaskCreate(&timer_task, "timer_task", 4 * 1024, NULL, 8, &timer_task_handle);

  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_alarm_config_t alarm_config = {
      .reload_count = 0,      // counter will reload with 0 on alarm event
      .alarm_count = 1000000, // period = 1s @resolution 1MHz
      .flags.auto_reload_on_alarm = true, // enable auto-reload
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = timer_on_alarm_callback,
  };
  // Pass task handle instead of queue
  ESP_ERROR_CHECK(
      gptimer_register_event_callbacks(gptimer, &cbs, timer_task_handle));
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));
}