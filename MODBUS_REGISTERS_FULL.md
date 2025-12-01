# Modbus Register Map - Full Documentation

## Modbus RTU Configuration

- **Slave Address**: 1
- **Baud Rate**: 19200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 2
- **TX Pin**: GPIO 17
- **RX Pin**: GPIO 18

## Holding Registers (Function Code 0x03 - Read, 0x06/0x10 - Write)

| Address | Name | Type | Description | Unit | Range |
|---------|------|------|-------------|------|-------|
| 0 | Battery Charge | R | Battery capacity remaining | % | 0-100 |
| 1 | Battery Runtime | R | Estimated battery runtime | seconds | 0-65535 |
| 2 | Input Voltage | R | Mains input voltage | V | 0-300 |
| 3 | Output Voltage | R | UPS output voltage | V | 0-300 |
| 4 | UPS Load | R | UPS load percentage | % | 0-100 |
| 5 | Status Word | R | Status bits (see below) | - | 0-65535 |
| 6 | Battery Runtime Low | R | Low runtime threshold | seconds | 0-65535 |
| 7 | Battery Design Capacity | R | Design capacity | % | 0-100 |
| 8 | Battery Charge Warning | R | Warning threshold | % | 0-100 |
| 9 | Battery Charge Low | R | Critical low threshold | % | 0-100 |
| 10 | Battery Voltage | R | Battery bank voltage | dV (x0.1V) | 0-255 |
| 11 | Battery Voltage Nominal | R | Nominal battery voltage | dV (x0.1V) | 0-255 |
| 12 | Input Voltage Nominal | R | Nominal input voltage | V | 0-300 |
| 13 | Input Transfer Low | R | Low transfer voltage threshold | V | 0-300 |
| 14 | Input Transfer High | R | High transfer voltage threshold | V | 0-300 |
| 15 | Beeper Mode | R | Beeper mode status | - | 1-3 |
| 16 | Test Code | R | Battery test result code | - | 0-255 |
| 17 | Timer Shutdown | R | Shutdown timer | seconds | -32768 to 32767 |
| 18 | Timer Start | R | Startup timer | seconds | -32768 to 32767 |
| 19 | Real Power Nominal | R | Nominal real power | W | 0-65535 |
| 32 | Beeper Control | R/W | Beeper control register | - | 1-3 |

## Status Word (Register 5) - Bit Fields

| Bit | Name | Description |
|-----|------|-------------|
| 0 | AC Present | AC power present (1) or not (0) |
| 1 | Charging | Battery charging (1) or not (0) |
| 2 | Discharging | Battery discharging (1) or not (0) |
| 3 | Overload | UPS overload (1) or not (0) |
| 4 | Below Capacity Limit | Below capacity limit (1) or not (0) |
| 5 | Fully Charged | Fully charged (1) or not (0) |
| 6 | Boost | Boost mode active (1) or not (0) |

## Beeper Mode (Register 15 & 32)

| Value | Description |
|-------|-------------|
| 1 | Disabled (Never beeps) |
| 2 | Enabled (Beeps on alarm) |
| 3 | Muted (Temporarily muted) |

## Notes

1. **Voltage Scaling**:
   - **Battery Voltage** is in deci-volts (dV). Divide by 10 to get Volts (e.g., 245 = 24.5V).
   - **Input/Output Voltage** are directly in Volts.

2. **Runtime**:
   - All runtime values are in **seconds**.

3. **Beeper Control**:
   - Writing to Register 32 changes the beeper mode.
   - Only values 1, 2, and 3 are valid.

4. **Status Word**:
   - Use bitwise operations to check individual status bits.

5. **Bulk Reading**:
   - You can read all parameters at once (Registers 0-20) for efficiency.
