#include "hid_worker.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/hid_host.h"
#include "usb/usb_host.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "hid_worker";

#define APP_QUIT_PIN GPIO_NUM_0

static bool user_shutdown = false;
bool UPS_DEV_CONNECTED = false;
static hid_host_device_handle_t latest_hid_device_handle = NULL;
static TaskHandle_t hid_host_task_handle = NULL;

// Type definitions for HID parsing
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

static report_desc_t *report_mapping = NULL;
static uint8_t report_count = 0;

typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_t;

// Forward declarations
static void
hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                            const hid_host_interface_event_t event, void *arg);
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg);
static void init_hid_mapping(void);

// --- HID Host Task ---

static void hid_host_task(void *pvParameters) {
  hid_host_event_t *evt;
  uint32_t notification_value;

  while (!user_shutdown) {
    // Wait for notification, clearing value on exit (ULONG_MAX)
    if (xTaskNotifyWait(0, ULONG_MAX, &notification_value,
                        pdMS_TO_TICKS(1000)) == pdTRUE) {
      evt = (hid_host_event_t *)notification_value;
      if (evt) {
        hid_host_device_event(evt->hid_device_handle, evt->event, evt->arg);
        free(evt);
      }
    }
  }
  vTaskDelete(NULL);
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void *arg) {
  if (hid_host_task_handle) {
    hid_host_event_t *evt = malloc(sizeof(hid_host_event_t));
    if (evt) {
      evt->hid_device_handle = hid_device_handle;
      evt->event = event;
      evt->arg = arg;
      // Use eSetValueWithoutOverwrite to avoid leaking memory if previous event
      // wasn't processed
      if (xTaskNotify(hid_host_task_handle, (uint32_t)evt,
                      eSetValueWithoutOverwrite) != pdPASS) {
        ESP_LOGW(TAG, "HID Worker task busy, dropping event %d", event);
        free(evt);
      }
    }
  }
}

static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG, "HID Device CONNECTED");
    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback, .callback_arg = NULL};
    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    latest_hid_device_handle = hid_device_handle;
    UPS_DEV_CONNECTED = true;
    break;
  default:
    break;
  }
}

static void
hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                            const hid_host_interface_event_t event, void *arg) {
  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    // Data is polled explicitly, so we might not need to handle async input
    // reports here unless the device sends them spontaneously.
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
    break;
  }
}

static void usb_lib_task(void *arg) {
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
    }
  }

  // Cleanup code (unreachable in this loop but kept for completeness)
  user_shutdown = true;
  vTaskDelay(10);
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

// --- HID Parsing Logic ---

static bool ups_read_report(uint8_t report_id, uint8_t *buffer, size_t len) {
  if (!UPS_DEV_CONNECTED || !latest_hid_device_handle) {
    return false;
  }

  report_desc_t *desc = NULL;
  for (uint8_t i = 0; i < report_count; i++) {
    if (report_mapping[i].report_id == report_id) {
      desc = &report_mapping[i];
      break;
    }
  }

  if (!desc || len < desc->length) {
    return false;
  }

  size_t actual_len = desc->length;
  esp_err_t err =
      hid_class_request_get_report(latest_hid_device_handle, desc->report_type,
                                   report_id, buffer, &actual_len);
  return (err == ESP_OK);
}

static void ups_update_data_from_buffer(UpsData *dst, const uint8_t *buffer,
                                        const report_desc_t *report_desc) {
  if (!dst || !buffer || !report_desc)
    return;

  for (uint8_t i = 0; i < report_desc->field_count; i++) {
    const hid_field_t *field = &report_desc->fields[i];

    switch (field->type) {
    case 0: // U8
    {
      uint8_t value = buffer[field->offset];
      if (strstr(field->name, "battery.charge") &&
          !strstr(field->name, "warning") && !strstr(field->name, "low"))
        dst->battery_charge = value;
      else if (strstr(field->name, "battery.charge.low"))
        dst->battery_charge_low = value;
      else if (strstr(field->name, "battery.charge.warning"))
        dst->battery_charge_warning = value;
      else if (strstr(field->name, "battery.voltage_dV"))
        dst->battery_voltage_dV = value;
      else if (strstr(field->name, "battery.voltage_nominal_dV"))
        dst->battery_voltage_nominal_dV = value;
      else if (strstr(field->name, "battery.design_capacity"))
        dst->battery_design_capacity = value;
      else if (strstr(field->name, "input.voltage_V") &&
               !strstr(field->name, "nominal"))
        dst->input_voltage_V = value;
      else if (strstr(field->name, "input.voltage_nominal_V"))
        dst->input_voltage_nominal_V = value;
      else if (strstr(field->name, "output.voltage_V"))
        dst->output_voltage_V = value;
      else if (strstr(field->name, "ups.load_percent"))
        dst->ups_load_percent = value;
      else if (strstr(field->name, "ups.beeper.mode"))
        dst->beeper_mode = value;
      else if (strstr(field->name, "ups.test.code"))
        dst->test_code = value;
      break;
    }
    case 1: // U16_LE
    {
      uint16_t value = buffer[field->offset] | (buffer[field->offset + 1] << 8);
      if (strstr(field->name, "battery.runtime_s"))
        dst->battery_runtime_s = value;
      else if (strstr(field->name, "battery.runtime_low"))
        dst->battery_runtime_low_s = value;
      else if (strstr(field->name, "input.transfer_low_V"))
        dst->input_transfer_low_V = value;
      else if (strstr(field->name, "input.transfer_high_V"))
        dst->input_transfer_high_V = value;
      else if (strstr(field->name, "ups.realpower_nominal_W"))
        dst->realpower_nominal_W = value;
      break;
    }
    case 2: // I16_LE
    {
      int16_t value =
          (int16_t)(buffer[field->offset] | (buffer[field->offset + 1] << 8));
      if (strstr(field->name, "ups.timer_shutdown_s"))
        dst->timer_shutdown_s = value;
      else if (strstr(field->name, "ups.timer_start_s"))
        dst->timer_start_s = value;
      break;
    }
    case 3: // BOOL
    {
      uint8_t byte_val = buffer[field->offset];
      bool bit_val = (byte_val & (1 << field->bit)) != 0;
      if (strstr(field->name, "status.ac_present"))
        dst->status_ac_present = bit_val;
      else if (strstr(field->name, "status.charging"))
        dst->status_charging = bit_val;
      else if (strstr(field->name, "status.discharging"))
        dst->status_discharging = bit_val;
      else if (strstr(field->name, "status.below_capacity_limit"))
        dst->status_below_capacity_limit = bit_val;
      else if (strstr(field->name, "status.fully_charged"))
        dst->status_fully_charged = bit_val;
      else if (strstr(field->name, "status.time_limit_expired"))
        dst->status_time_limit_expired = bit_val;
      else if (strstr(field->name, "status.boost"))
        dst->status_boost = bit_val;
      else if (strstr(field->name, "status.overload"))
        dst->status_overload = bit_val;
      break;
    }
    }
  }
}

// --- Public API ---

void hid_worker_init(void) {
  init_hid_mapping();

  BaseType_t task_created;
  task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  ulTaskNotifyTake(false, 1000);

  // Create HID task FIRST to ensure handle is ready before callback
  // registration
  task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2,
                             &hid_host_task_handle);
  assert(task_created == pdTRUE);

  // Now install HID host - callback can safely use hid_host_task_handle
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
}

bool hid_worker_poll_ups(UpsData *data) {
  if (!data)
    return false;

  const uint8_t important_reports[] = {7,  8,  9,  10, 11, 14, 15, 16,
                                       18, 19, 20, 21, 22, 23, 24};
  const uint8_t report_count_local =
      sizeof(important_reports) / sizeof(important_reports[0]);

  uint8_t buffer[64];
  bool all_ok = true;

  for (uint8_t i = 0; i < report_count_local; i++) {
    uint8_t report_id = important_reports[i];
    memset(buffer, 0, sizeof(buffer));
    if (ups_read_report(report_id, buffer, sizeof(buffer))) {
      for (uint8_t j = 0; j < report_count; j++) {
        if (report_mapping[j].report_id == report_id) {
          ups_update_data_from_buffer(data, buffer, &report_mapping[j]);
          break;
        }
      }
    } else {
      all_ok = false;
    }
  }
  return all_ok;
}

bool hid_worker_is_connected(void) { return UPS_DEV_CONNECTED; }

static void init_hid_mapping(void) {
  // ... (Mapping initialization code copied from original file) ...
  // For brevity in this response, I will assume the mapping code is copied
  // here. In a real scenario, I would copy the full content. I will put the
  // full content here to be safe.

  static hid_field_t fields_7[] = {{"battery.design_capacity", 1, 1, 0, 0},
                                   {"battery.charge.warning", 4, 1, 0, 0},
                                   {"battery.charge.low", 5, 1, 0, 0}};
  static hid_field_t fields_8[] = {{"battery.charge", 1, 1, 0, 0},
                                   {"battery.runtime_s", 2, 2, 0, 1},
                                   {"battery.runtime_low", 4, 2, 0, 1}};
  static hid_field_t fields_9[] = {{"battery.voltage_nominal_dV", 1, 1, 0, 0}};
  static hid_field_t fields_10[] = {{"battery.voltage_dV", 1, 1, 0, 0}};
  static hid_field_t fields_11[] = {{"status.ac_present", 1, 0, 0, 3},
                                    {"status.charging", 1, 0, 1, 3},
                                    {"status.discharging", 1, 0, 2, 3},
                                    {"status.below_capacity_limit", 1, 0, 3, 3},
                                    {"status.fully_charged", 1, 0, 4, 3}};
  static hid_field_t fields_12[] = {{"ups.beeper.mode", 1, 1, 0, 0}};
  static hid_field_t fields_14[] = {{"input.voltage_nominal_V", 1, 1, 0, 0}};
  static hid_field_t fields_15[] = {{"input.voltage_V", 1, 1, 0, 0}};
  static hid_field_t fields_16[] = {{"input.transfer_low_V", 1, 2, 0, 1},
                                    {"input.transfer_high_V", 3, 2, 0, 1}};
  static hid_field_t fields_18[] = {{"output.voltage_V", 1, 1, 0, 0}};
  static hid_field_t fields_19[] = {{"ups.load_percent", 1, 1, 0, 0}};
  static hid_field_t fields_20[] = {{"ups.test.code", 1, 1, 0, 0}};
  static hid_field_t fields_21[] = {{"ups.timer_shutdown_s", 1, 2, 0, 2}};
  static hid_field_t fields_22[] = {{"ups.timer_start_s", 1, 2, 0, 2}};
  static hid_field_t fields_23[] = {{"status.boost", 1, 0, 0, 3},
                                    {"status.overload", 1, 0, 1, 3}};
  static hid_field_t fields_24[] = {
      {"ups.realpower_nominal_W_raw", 1, 2, 0, 1}};

  report_count = 16;
  report_mapping =
      (report_desc_t *)malloc(report_count * sizeof(report_desc_t));
  if (!report_mapping)
    return;

  report_mapping[0] = (report_desc_t){7, 3, 7, fields_7, 3};
  report_mapping[1] = (report_desc_t){8, 3, 6, fields_8, 3};
  report_mapping[2] = (report_desc_t){9, 3, 2, fields_9, 1};
  report_mapping[3] = (report_desc_t){10, 3, 2, fields_10, 1};
  report_mapping[4] = (report_desc_t){11, 1, 2, fields_11, 5};
  report_mapping[5] = (report_desc_t){12, 3, 2, fields_12, 1};
  report_mapping[6] = (report_desc_t){14, 3, 2, fields_14, 1};
  report_mapping[7] = (report_desc_t){15, 3, 2, fields_15, 1};
  report_mapping[8] = (report_desc_t){16, 3, 5, fields_16, 2};
  report_mapping[9] = (report_desc_t){18, 3, 2, fields_18, 1};
  report_mapping[10] = (report_desc_t){19, 3, 2, fields_19, 1};
  report_mapping[11] = (report_desc_t){20, 3, 2, fields_20, 1};
  report_mapping[12] = (report_desc_t){21, 3, 3, fields_21, 1};
  report_mapping[13] = (report_desc_t){22, 3, 3, fields_22, 1};
  report_mapping[14] = (report_desc_t){23, 3, 2, fields_23, 2};
  report_mapping[15] = (report_desc_t){24, 3, 3, fields_24, 1};

  ESP_LOGI(TAG, "HID mapping initialized");
}
