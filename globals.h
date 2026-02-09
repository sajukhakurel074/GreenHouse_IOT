#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Arduino.h"

typedef struct {
  float temperature;
  float humidity;
  float lux;
  uint32_t ts_ms;
} SensorData_t;

typedef struct {
  bool fanOn;
  bool heaterOn;
  uint8_t fanPwm;
  uint8_t heaterPwm;
} ActuatorCmd_t;

typedef enum {
  MODE_OFF = 0,
  MODE_AUTO,
  MODE_MANUAL
} Mode_t;

typedef enum : uint8_t {
  ALARM_NONE = 0,
  ALARM_SENSOR_FAIL,
  ALARM_OVER_TEMP,
  ALARM_OVER_HUM,
  ALARM_RADIO_LOST
} Alarm_t;

typedef struct {
  Mode_t     mode;
  Alarm_t    alarm;
  bool       fanOn;
  bool       heaterOn;
  uint8_t    fanPwm;
  uint8_t    heaterPwm;
  uint32_t   lastUpdate_ms;
  char       modeLabel[16];
} ModeCtx_t;

typedef enum {
  ENC_NONE = 0,
  ENC_UP,
  ENC_DOWN,
  ENC_SELECT,
  ENC_SELECT_LONG
} EncodeEvent_t;

typedef enum : uint8_t {
  CFG_NONE = 0,
  CFG_SET_MODE,
  CFG_SET_TEMP_TARGET,
  CFG_SET_HUM_TARGET,
  CFG_SET_LUX_TARGET,
  CFG_SET_MANUAL_FAN,
  CFG_SET_MANUAL_HEATER,
  CFG_SET_SAMPLING_PERIOD
} ConfigCmdType_t;

typedef struct {
  ConfigCmdType_t type;
  Mode_t          mode;          // FIXED (was GhMode_t)

  float           tempTarget_C;
  float           humTarget_RH;
  float           luxTarget;

  bool            fanOn;
  bool            heaterOn;
  uint8_t         fanPwm;
  uint8_t         heaterPwm;

  uint16_t        sampling_s;
  uint32_t        ts_ms;
  uint32_t        seq;
} ConfigCmd_t;
