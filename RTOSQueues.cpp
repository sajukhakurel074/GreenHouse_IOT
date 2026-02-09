#include "RTOSQueues.h"

// ===============================
// Queue handles
// ===============================
QueueHandle_t qSensorsDataModel = nullptr;
QueueHandle_t qEncoder          = nullptr;
QueueHandle_t qConfig           = nullptr;
QueueHandle_t qModeCtx          = nullptr;
QueueHandle_t qActuatorCmd      = nullptr;

// ===============================
// Queue creation per component
// ===============================
bool queuesCreateSensors() {
  if (qSensorsDataModel) return true;
  qSensorsDataModel = xQueueCreate(1, sizeof(SensorData_t));   // latest snapshot
  return (qSensorsDataModel != nullptr);
}

bool queuesCreateEncoder() {
  if (qEncoder) return true;
  qEncoder = xQueueCreate(10, sizeof(EncodeEvent_t));          // events
  return (qEncoder != nullptr);
}

bool queuesCreateConfig() {
  if (qConfig) return true;
  qConfig = xQueueCreate(10, sizeof(ConfigCmd_t));             // commands
  return (qConfig != nullptr);
}

bool queuesCreateModeCtx() {
  if (qModeCtx) return true;
  qModeCtx = xQueueCreate(1, sizeof(ModeCtx_t));               // latest state for UI/radio
  return (qModeCtx != nullptr);
}

bool queuesCreateActuator() {
  if (qActuatorCmd) return true;
  qActuatorCmd = xQueueCreate(1, sizeof(ActuatorCmd_t));       // latest actuator command
  return (qActuatorCmd != nullptr);
}

bool queuesCreateAll() {
  return queuesCreateSensors() &&
         queuesCreateEncoder() &&
         queuesCreateConfig()  &&
         queuesCreateModeCtx() &&
         queuesCreateActuator();
}

bool queuesReadyAll() {
  return qSensorsDataModel && qEncoder && qConfig && qModeCtx && qActuatorCmd;
}

// ===============================
// APIs (safe wrappers)
// ===============================

// ---- Sensors DataModel ----
bool sensorsModelWrite(const SensorData_t &d) {
  if (!qSensorsDataModel) return false;
  return (xQueueOverwrite(qSensorsDataModel, &d) == pdTRUE);
}

bool sensorsModelPeek(SensorData_t &out, TickType_t to) {
  if (!qSensorsDataModel) return false;
  return (xQueuePeek(qSensorsDataModel, &out, to) == pdTRUE);
}

// ---- Encoder events ----
bool encoderSendEvent(EncodeEvent_t e, TickType_t to) {
  if (!qEncoder) return false;
  return (xQueueSend(qEncoder, &e, to) == pdTRUE);
}

bool encoderRecvEvent(EncodeEvent_t &out, TickType_t to) {
  if (!qEncoder) return false;
  return (xQueueReceive(qEncoder, &out, to) == pdTRUE);
}

// ---- Config commands ----
bool configSend(const ConfigCmd_t &cmd, TickType_t to) {
  if (!qConfig) return false;
  return (xQueueSend(qConfig, &cmd, to) == pdTRUE);
}

bool configRecv(ConfigCmd_t &out, TickType_t to) {
  if (!qConfig) return false;
  return (xQueueReceive(qConfig, &out, to) == pdTRUE);
}

// ---- Mode context ----
bool modeCtxWrite(const ModeCtx_t &ctx) {
  if (!qModeCtx) return false;
  return (xQueueOverwrite(qModeCtx, &ctx) == pdTRUE);
}

bool modeCtxPeek(ModeCtx_t &out, TickType_t to) {
  if (!qModeCtx) return false;
  return (xQueuePeek(qModeCtx, &out, to) == pdTRUE);
}

// ---- Actuator command ----
bool actuatorWrite(const ActuatorCmd_t &cmd) {
  if (!qActuatorCmd) return false;
  return (xQueueOverwrite(qActuatorCmd, &cmd) == pdTRUE);
}

bool actuatorPeek(ActuatorCmd_t &out, TickType_t to) {
  if (!qActuatorCmd) return false;
  return (xQueuePeek(qActuatorCmd, &out, to) == pdTRUE);
}
