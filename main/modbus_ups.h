#ifndef MODBUS_UPS_H
#define MODBUS_UPS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Modbus RTU configuration от Kconfig
#define MODBUS_UART_PORT        (CONFIG_MB_UART_PORT_NUM)
#define MODBUS_UART_TXD_PIN     (CONFIG_MB_UART_TXD)
#define MODBUS_UART_RXD_PIN     (CONFIG_MB_UART_RXD)
#define MODBUS_UART_RTS_PIN     (CONFIG_MB_UART_RTS)
#define MODBUS_SLAVE_ADDR       (CONFIG_MB_SLAVE_ADDR)
#define MODBUS_UART_BAUDRATE    (CONFIG_MB_UART_BAUD_RATE)

// Parity configuration (using UART parity constants)
#if defined(CONFIG_MB_PARITY_NONE)
#define MODBUS_PARITY           UART_PARITY_DISABLE
#elif defined(CONFIG_MB_PARITY_ODD)
#define MODBUS_PARITY           UART_PARITY_ODD
#elif defined(CONFIG_MB_PARITY_EVEN)
#define MODBUS_PARITY           UART_PARITY_EVEN
#else
#define MODBUS_PARITY           UART_PARITY_DISABLE
#endif

#define MODBUS_REG_START        0
#define MODBUS_REG_COUNT        64

// UPS Data Structure
typedef struct {
    // Battery data
    uint8_t battery_charge;
    uint16_t battery_runtime_s;
    uint16_t battery_runtime_low_s;
    uint8_t battery_design_capacity;
    uint8_t battery_charge_warning;
    uint8_t battery_charge_low;
    uint8_t battery_voltage_dV;
    uint8_t battery_voltage_nominal_dV;
    
    // Input data
    uint16_t input_voltage_V;
    uint8_t input_voltage_nominal_V;
    uint16_t input_transfer_low_V;
    uint16_t input_transfer_high_V;
    
    // Output data
    uint16_t output_voltage_V;
    uint8_t ups_load_percent;
    
    // Status bits
    bool status_ac_present;
    bool status_charging;
    bool status_discharging;
    bool status_below_capacity_limit;
    bool status_fully_charged;
    bool status_time_limit_expired;
    bool status_boost;
    bool status_overload;
    
    // Other parameters
    uint8_t beeper_mode;
    uint8_t test_code;
    int16_t timer_shutdown_s;
    int16_t timer_start_s;
    uint16_t realpower_nominal_W;
} UpsData;

// Global declarations
extern void* modbus_slave_handle;
extern UpsData current_ups_data;

// Modbus Configuration Structure
typedef struct {
    uint8_t slave_addr;
    uint32_t baudrate;
    uint8_t parity;      // 0=None, 1=Odd, 2=Even
    uint8_t stop_bits;   // 1 or 2
} ModbusConfig;

// Function prototypes
esp_err_t modbus_ups_slave_init(void);
void modbus_ups_update_registers(const UpsData *data);

// Configuration functions
esp_err_t modbus_config_load(ModbusConfig *config);
esp_err_t modbus_config_save(const ModbusConfig *config);
void modbus_config_set_defaults(ModbusConfig *config);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_UPS_H