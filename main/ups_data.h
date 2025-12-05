#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>


// UPS Data Structure
typedef struct {
  // Battery
  uint8_t battery_charge;              // %
  uint8_t battery_charge_low;          // %
  uint8_t battery_charge_warning;      // %
  uint16_t battery_runtime_s;          // seconds
  uint16_t battery_runtime_low_s;      // seconds
  uint16_t battery_voltage_dV;         // deci-Volts (0.1V)
  uint16_t battery_voltage_nominal_dV; // deci-Volts (0.1V)
  uint8_t battery_design_capacity;     // % (usually 100)

  // Input
  uint16_t input_voltage_V;         // Volts
  uint16_t input_voltage_nominal_V; // Volts
  uint16_t input_transfer_low_V;    // Volts
  uint16_t input_transfer_high_V;   // Volts

  // Output
  uint16_t output_voltage_V;    // Volts
  uint8_t ups_load_percent;     // %
  uint16_t realpower_nominal_W; // Watts

  // Status / Config
  uint8_t beeper_mode; // 2=enabled, other=disabled
  uint8_t test_code;   // Test status

  // Timers
  int16_t timer_shutdown_s; // seconds
  int16_t timer_start_s;    // seconds

  // Status Bits (Booleans)
  bool status_ac_present;
  bool status_charging;
  bool status_discharging;
  bool status_below_capacity_limit;
  bool status_fully_charged;
  bool status_time_limit_expired;
  bool status_boost;
  bool status_overload;

} UpsData;

extern UpsData current_ups_data;
extern SemaphoreHandle_t ups_data_mutex;
extern bool UPS_DEV_CONNECTED;
