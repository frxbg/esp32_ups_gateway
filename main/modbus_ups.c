#include "modbus_ups.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "mbcontroller.h"
#include "nvs.h"
#include "string.h"

static const char *TAG = "modbus_ups";
static const char *NVS_NAMESPACE = "modbus_cfg";

// Holding registers memory area
static uint16_t holding_reg[MODBUS_REG_COUNT] = {0};

// Global Modbus slave handle
void *modbus_slave_handle = NULL;

// Initialize Modbus RTU slave
esp_err_t modbus_ups_slave_init(void) {
  esp_err_t err = ESP_OK;

  ESP_LOGI(TAG, "Initializing Modbus RTU slave...");

  // Load configuration from NVS
  ModbusConfig config;
  err = modbus_config_load(&config);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load config, using defaults");
    modbus_config_set_defaults(&config);
  }

  // Initialize Modbus controller
  err = mbc_slave_init(MB_PORT_SERIAL_SLAVE, &modbus_slave_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Modbus controller initialization failed: %s",
             esp_err_to_name(err));
    return err;
  }

  // Setup communication parameters according to official example
  mb_communication_info_t comm_info = {0};

#if defined(CONFIG_MB_COMM_MODE_RTU)
  comm_info.mode = MB_MODE_RTU;
#elif defined(CONFIG_MB_COMM_MODE_ASCII)
  comm_info.mode = MB_MODE_ASCII;
#else
  comm_info.mode = MB_MODE_RTU; // Default to RTU
#endif

  // Use configuration from NVS
  comm_info.slave_addr = config.slave_addr;
  comm_info.port = MODBUS_UART_PORT;
  comm_info.baudrate = config.baudrate;

  // Convert parity: 0=None, 1=Odd, 2=Even
  // Note: mb_communication_info_t uses uart_parity_t (UART constants)
  if (config.parity == 0) {
    comm_info.parity = UART_PARITY_DISABLE;
  } else if (config.parity == 1) {
    comm_info.parity = UART_PARITY_ODD;
  } else {
    comm_info.parity = UART_PARITY_EVEN;
  }

  err = mbc_slave_setup((void *)&comm_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Modbus setup failed: %s", esp_err_to_name(err));
    mbc_slave_destroy();
    return err;
  }

  // Set UART pins - Must be done after setup but before start
  err = uart_set_pin(MODBUS_UART_PORT, MODBUS_UART_TXD_PIN, MODBUS_UART_RXD_PIN,
                     MODBUS_UART_RTS_PIN, // RTS for RS485 direction control
                     UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "UART pin configuration failed: %s", esp_err_to_name(err));
    mbc_slave_destroy();
    return err;
  }

  // Configure register area descriptor for holding registers
  mb_register_area_descriptor_t reg_area = {.type = MB_PARAM_HOLDING,
                                            .start_offset = MODBUS_REG_START,
                                            .address = (void *)holding_reg,
                                            .size = sizeof(holding_reg)};

  err = mbc_slave_set_descriptor(reg_area);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set register descriptor: %s",
             esp_err_to_name(err));
    mbc_slave_destroy();
    return err;
  }

  // Start Modbus controller
  err = mbc_slave_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start Modbus slave: %s", esp_err_to_name(err));
    mbc_slave_destroy();
    return err;
  }

  const char *parity_str = "None";
  if (config.parity == 1)
    parity_str = "Odd";
  else if (config.parity == 2)
    parity_str = "Even";

  ESP_LOGI(TAG, "Modbus RTU slave initialized successfully");
  ESP_LOGI(TAG, "  Mode: %s", comm_info.mode == MB_MODE_RTU ? "RTU" : "ASCII");
  ESP_LOGI(TAG, "  UART Port: %d", MODBUS_UART_PORT);
  ESP_LOGI(TAG, "  Baud Rate: %ld", config.baudrate);
  ESP_LOGI(TAG, "  Parity: %s, Stop Bits: %d", parity_str, config.stop_bits);
  ESP_LOGI(TAG, "  TX Pin: %d, RX Pin: %d, RTS Pin: %d", MODBUS_UART_TXD_PIN,
           MODBUS_UART_RXD_PIN, MODBUS_UART_RTS_PIN);
  ESP_LOGI(TAG, "  Slave Address: %d", config.slave_addr);
  ESP_LOGI(TAG, "  Holding Registers: %d (offset %d)", MODBUS_REG_COUNT,
           MODBUS_REG_START);

  return ESP_OK;
}

// Update Modbus registers from UPS data
void modbus_ups_update_registers(const UpsData *data) {
  if (!data) {
    ESP_LOGE(TAG, "Invalid UPS data pointer");
    return;
  }

  // Map UPS data to Modbus holding registers
  holding_reg[0] = data->battery_charge;    // 0: Battery charge %
  holding_reg[1] = data->battery_runtime_s; // 1-2: Runtime seconds
  holding_reg[2] = data->input_voltage_V;   // 3: Input voltage
  holding_reg[3] = data->output_voltage_V;  // 4: Output voltage
  holding_reg[4] = data->ups_load_percent;  // 5: Load percent

  // Register 5: Status flags (bit field)
  holding_reg[5] = 0;
  if (data->status_ac_present)
    holding_reg[5] |= (1 << 0);
  if (data->status_charging)
    holding_reg[5] |= (1 << 1);
  if (data->status_discharging)
    holding_reg[5] |= (1 << 2);
  if (data->status_overload)
    holding_reg[5] |= (1 << 3);
  if (data->status_below_capacity_limit)
    holding_reg[5] |= (1 << 4);
  if (data->status_fully_charged)
    holding_reg[5] |= (1 << 5);
  if (data->status_boost)
    holding_reg[5] |= (1 << 6);
  if (data->status_time_limit_expired)
    holding_reg[5] |= (1 << 7);

  holding_reg[6] = data->battery_runtime_low_s;   // 6-7: Low runtime threshold
  holding_reg[7] = data->battery_design_capacity; // 8: Design capacity
  holding_reg[8] = data->battery_charge_warning;  // 9: Warning threshold
  holding_reg[9] = data->battery_charge_low;      // 10: Low threshold
  holding_reg[10] =
      data->battery_voltage_dV > 30
          ? data->battery_voltage_dV * 0.941f
          : (data->battery_voltage_dV * 0.941f) +
                data->battery_voltage_nominal_dV; // 11: Battery voltage (dV)
  holding_reg[11] =
      data->battery_voltage_nominal_dV;            // 12: Nominal voltage (dV)
  holding_reg[12] = data->input_voltage_nominal_V; // 13: Nominal input voltage
  holding_reg[13] = data->input_transfer_low_V;    // 14-15: Transfer low limit
  holding_reg[14] = data->input_transfer_high_V;   // 16-17: Transfer high limit
  holding_reg[15] = data->beeper_mode;             // 18: Beeper mode
  holding_reg[16] = data->test_code;               // 19: Test status code
  holding_reg[17] = (uint16_t)data->timer_shutdown_s; // 20-21: Shutdown timer
  holding_reg[18] = (uint16_t)data->timer_start_s;    // 22-23: Start timer
  holding_reg[19] = data->realpower_nominal_W; // 24-25: Real power nominal

  // В slave режим не е необходимо да уведомяваме стека
  // Modbus автоматично предоставя актуалните стойности при заявка от master

  ESP_LOGD(TAG, "Modbus registers updated");
}

// Set default Modbus configuration
void modbus_config_set_defaults(ModbusConfig *config) {
  if (!config)
    return;

  config->slave_addr = 1;
  config->baudrate = 115200;
  config->parity = 0; // None
  config->stop_bits = 1;
}

// Load Modbus configuration from NVS
esp_err_t modbus_config_load(ModbusConfig *config) {
  if (!config) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t err;

  // Set defaults first
  modbus_config_set_defaults(config);

  // Open NVS
  err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS not found, using defaults");
    return ESP_OK; // Not an error, just use defaults
  }

  // Load values
  uint8_t slave_addr;
  err = nvs_get_u8(nvs_handle, "slave_addr", &slave_addr);
  if (err == ESP_OK && slave_addr >= 1 && slave_addr <= 247) {
    config->slave_addr = slave_addr;
  }

  uint32_t baudrate;
  err = nvs_get_u32(nvs_handle, "baudrate", &baudrate);
  if (err == ESP_OK && baudrate >= 1200 && baudrate <= 115200) {
    config->baudrate = baudrate;
  }

  uint8_t parity;
  err = nvs_get_u8(nvs_handle, "parity", &parity);
  if (err == ESP_OK && parity <= 2) {
    config->parity = parity;
  }

  uint8_t stop_bits;
  err = nvs_get_u8(nvs_handle, "stop_bits", &stop_bits);
  if (err == ESP_OK && (stop_bits == 1 || stop_bits == 2)) {
    config->stop_bits = stop_bits;
  }

  nvs_close(nvs_handle);

  ESP_LOGI(TAG, "Loaded config: addr=%d, baud=%ld, parity=%d, stop=%d",
           config->slave_addr, config->baudrate, config->parity,
           config->stop_bits);

  return ESP_OK;
}

// Save Modbus configuration to NVS
esp_err_t modbus_config_save(const ModbusConfig *config) {
  if (!config) {
    return ESP_ERR_INVALID_ARG;
  }

  // Validate parameters
  if (config->slave_addr < 1 || config->slave_addr > 247) {
    ESP_LOGE(TAG, "Invalid slave address: %d", config->slave_addr);
    return ESP_ERR_INVALID_ARG;
  }

  if (config->baudrate < 1200 || config->baudrate > 115200) {
    ESP_LOGE(TAG, "Invalid baudrate: %ld", config->baudrate);
    return ESP_ERR_INVALID_ARG;
  }

  if (config->parity > 2) {
    ESP_LOGE(TAG, "Invalid parity: %d", config->parity);
    return ESP_ERR_INVALID_ARG;
  }

  if (config->stop_bits < 1 || config->stop_bits > 2) {
    ESP_LOGE(TAG, "Invalid stop bits: %d", config->stop_bits);
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t err;

  // Open NVS
  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
    return err;
  }

  // Save values
  err = nvs_set_u8(nvs_handle, "slave_addr", config->slave_addr);
  if (err != ESP_OK)
    goto error;

  err = nvs_set_u32(nvs_handle, "baudrate", config->baudrate);
  if (err != ESP_OK)
    goto error;

  err = nvs_set_u8(nvs_handle, "parity", config->parity);
  if (err != ESP_OK)
    goto error;

  err = nvs_set_u8(nvs_handle, "stop_bits", config->stop_bits);
  if (err != ESP_OK)
    goto error;

  // Commit changes
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK)
    goto error;

  nvs_close(nvs_handle);

  ESP_LOGI(TAG, "Saved config: addr=%d, baud=%ld, parity=%d, stop=%d",
           config->slave_addr, config->baudrate, config->parity,
           config->stop_bits);

  return ESP_OK;

error:
  nvs_close(nvs_handle);
  ESP_LOGE(TAG, "Error saving config: %s", esp_err_to_name(err));
  return err;
}