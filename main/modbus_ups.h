#ifndef MODBUS_UPS_H
#define MODBUS_UPS_H

#include "driver/uart.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Modbus RTU configuration от Kconfig
#define MODBUS_UART_PORT (CONFIG_MB_UART_PORT_NUM)
#define MODBUS_UART_TXD_PIN (CONFIG_MB_UART_TXD)
#define MODBUS_UART_RXD_PIN (CONFIG_MB_UART_RXD)
#define MODBUS_UART_RTS_PIN (CONFIG_MB_UART_RTS)
#define MODBUS_SLAVE_ADDR (CONFIG_MB_SLAVE_ADDR)
#define MODBUS_UART_BAUDRATE (CONFIG_MB_UART_BAUD_RATE)

// Parity configuration (using UART parity constants)
#if defined(CONFIG_MB_PARITY_NONE)
#define MODBUS_PARITY UART_PARITY_DISABLE
#elif defined(CONFIG_MB_PARITY_ODD)
#define MODBUS_PARITY UART_PARITY_ODD
#elif defined(CONFIG_MB_PARITY_EVEN)
#define MODBUS_PARITY UART_PARITY_EVEN
#else
#define MODBUS_PARITY UART_PARITY_DISABLE
#endif

#define MODBUS_REG_START 0
#define MODBUS_REG_COUNT 64

// UPS Data Structure
#include "ups_data.h"

// Global declarations
extern void *modbus_slave_handle;
extern UpsData current_ups_data;

// Modbus Configuration Structure
typedef struct {
  uint8_t slave_addr;
  uint32_t baudrate;
  uint8_t parity;    // 0=None, 1=Odd, 2=Even
  uint8_t stop_bits; // 1 or 2
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