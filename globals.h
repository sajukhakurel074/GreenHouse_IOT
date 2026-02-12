#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"


// ===============================
// Sensor Data Model
// ===============================
typedef struct {
  float temperature;    // Temperature in °C (Producer: Sensors task, Consumers: Display + MQTT)
  float humidity;       // Humidity in %RH (To be discussed: precision, filtering method)
  float lux;            // Ambient light in Lux (To be discussed: calibration / scaling)
  uint32_t ts_ms;       // Timestamp in milliseconds (millis()), helps detect stale data
} SensorData_t;


// ===============================
// System Mode Definition
// ===============================
typedef enum {
  MODE_OFF = 0,         // System fully disabled
  MODE_AUTO,            // Automatic control based on setpoints
  MODE_MANUAL           // Manual override mode
} Mode_t;               // Used by: Display (Producer), Logic (Consumer)


// ===============================
// Alarm Definition
// ===============================
typedef enum : uint8_t {
  ALARM_NONE = 0,
  ALARM_SENSOR_FAIL,    // Sensor not responding / invalid data
  ALARM_OVER_TEMP,      // Temperature exceeds safety threshold
  ALARM_OVER_HUM,       // Humidity exceeds safety threshold
  ALARM_RADIO_LOST      // MQTT / Communication failure
} Alarm_t;              // To be discussed: Which alarms are required in final implementation


// ===============================
// Mode Context (System State Snapshot)
// ===============================
typedef struct {
  Mode_t     mode;           // Current operating mode (Producer: Display, Consumer: Logic)
  Alarm_t    alarm;          // Current alarm state (Producer: Logic, Consumer: Display + MQTT)

  bool       fanOn;          // Current fan state (To be discussed: controlled by Logic only?)
  bool       heaterOn;       // Current heater state

  uint8_t    fanPwm;         // Fan power level (0–100 %) (To be discussed: range validation)
  uint8_t    heaterPwm;      // Heater power level (0–100 %)

  uint32_t   lastUpdate_ms;  // Timestamp of last mode update
  char       modeLabel[16];  // Optional display label (e.g., "AUTO", "MANUAL") (To be discussed: required or redundant)

  // Thresholds

} ModeCtx_t;                 // Snapshot queue: Overwrite by Display, Peek by Logic


// ===============================
// Encoder Input Definition
// ===============================
typedef enum {
  ENC_NONE = 0,
  ENC_UP,                    // Rotation clockwise
  ENC_DOWN,                  // Rotation counter-clockwise
  ENC_SELECT,                // Short press
  ENC_SELECT_LONG            // Long press
} EncodeEvent_t;             // Producer: Encoder controller, Consumer: Display
                             // To be discussed: debounce strategy and repeat speed


// ===============================
// Configuration Command Types
// ===============================
typedef enum : uint8_t {
  CFG_NONE = 0,
  CFG_SET_MODE,              // Change system mode (AUTO / MANUAL / OFF)
  CFG_SET_TEMP_TARGET,       // Update temperature setpoint
  CFG_SET_HUM_TARGET,        // Update humidity setpoint
  CFG_SET_LUX_TARGET,        // Update light threshold
  CFG_SET_MANUAL_FAN,        // Manual fan control (on/off + PWM)
  CFG_SET_MANUAL_HEATER,     // Manual heater control (on/off + PWM)
  CFG_SET_SAMPLING_PERIOD    // Change sensor sampling interval
} ConfigCmdType_t;           // To be discussed: Which commands are mandatory in MVP


typedef enum{
  Actuator_Alarm_None = 0,
  Fan_Saturation,
  Heater_Saturation,
  Control_Oscillation,
  Conflicting_Actuation,
  Safe_Mode_Active
}ActuatorAlarm_t;


// ===============================
// Configuration Snapshot
// ===============================
typedef struct {
  ConfigCmdType_t type;      // Type of configuration command (Producer: MQTT, Consumer: Sensors + Display + Logic)

  Mode_t          mode;      // Mode selection (Used when type == CFG_SET_MODE)

  float           tempTarget_C;   // Desired temperature setpoint (To be discussed: allowed range)
  float           humTarget_RH;   // Desired humidity setpoint
  float           luxTarget;      // Desired light threshold

  bool            fanOn;          // Manual override state
  bool            heaterOn;

  uint8_t         fanPwm;         // Manual PWM (0–100 %) (To be validated in Logic layer)
  uint8_t         heaterPwm;

  uint16_t        sampling_s;     // Sensor sampling interval in seconds (To be discussed: min/max limits)

  uint32_t        ts_ms;          // Timestamp of command reception
  uint32_t        seq;            // Optional sequence counter for tracking updates
} ConfigCmd_t;                    // Snapshot queue: Overwrite by MQTT, Peek by Sensors + Display controller
