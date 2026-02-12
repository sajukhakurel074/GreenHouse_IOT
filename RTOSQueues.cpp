#include "RTOSQueues.h"

// ===============================
// Queue handles (4 queues total as per professor architecture)
// ===============================
QueueHandle_t qEncoder          = nullptr;   // Encoder Queue  : Overwrite by Encoder controller, Peek by Display
QueueHandle_t qSensorsDataModel = nullptr;   // Sensor Queue   : Overwrite by Sensors, Peek by Display + MQTT
QueueHandle_t qConfig           = nullptr;   // Config Queue   : Overwrite by MQTT, Peek by Sensors + Display controller
QueueHandle_t qModeCtx          = nullptr;   // Mode Queue     : Overwrite by Display, Peek by Logic controller
QueueHandle_t qAlarm            = nullptr;   // Actuator Alarm Queue     : Overwrite by Logic Controller, Peek by MQTT


// ===============================
// Queue creation per component
// ===============================
bool queuesCreateEncoder() {
  if (qEncoder) return true;                                   // Checks if already exists
  qEncoder = xQueueCreate(1, sizeof(EncodeEvent_t));           // Snapshot queue (Overwrite + Peek) hence length = 1
                                                               // Only latest encoder state/event is needed for display update
  return (qEncoder != nullptr);
}

bool queuesCreateSensors() {
  if (qSensorsDataModel) return true;                          // Checks if already exists
  qSensorsDataModel = xQueueCreate(1, sizeof(SensorData_t));   // Snapshot queue (Overwrite + Peek) hence length = 1
                                                               // Only latest sensor snapshot is needed for display + mqtt
  return (qSensorsDataModel != nullptr);
}

bool queuesCreateConfig() {
  if (qConfig) return true;                                    // Checks if already exists
  qConfig = xQueueCreate(1, sizeof(ConfigCmd_t));              // Snapshot queue (Overwrite + Peek) hence length = 1
                                                               // Only latest config is relevant (new values overwrite old ones)
  return (qConfig != nullptr);
}

bool queuesCreateModeCtx() {
  if (qModeCtx) return true;                                   // Checks if already exists
  qModeCtx = xQueueCreate(1, sizeof(ModeCtx_t));               // Snapshot queue (Overwrite + Peek) hence length = 1
                                                               // Display publishes latest mode selection, logic controller reads it
  return (qModeCtx != nullptr);
}

bool queuesCreateActuatorAlarm() {
  if (qAlarm) return true;                                   // Checks if already exists
  qAlarm = xQueueCreate(1, sizeof(ActuatorAlarm_t));               // Snapshot queue (Overwrite + Peek) hence length = 1
                                                               // Logic controller publishes alarm for faults in actuator, MQTT reads it
  return (qAlarm != nullptr);
}

bool queuesCreateAll() {                // Creates all queues simultaneously, fails to create following queues if preceding one fails (fail case: Insufficient memory)
  return queuesCreateEncoder() &&
         queuesCreateSensors() &&
         queuesCreateConfig()  &&
         queuesCreateModeCtx() &&
         queuesCreateActuatorAlarm();
}

bool queuesReadyAll() {                 // Checks if all queues are created (!=0)
  return qEncoder && qSensorsDataModel && qConfig && qModeCtx && qAlarm;
}


// -------- Loading and Fetching data in Queue -------- //

// ---- Encoder Queue (Overwrite by Encoder controller, Peek by Display) ----
bool encoderSendEvent(EncodeEvent_t e, TickType_t to) {
  (void)to;                                                     // Not required for overwrite API
  if (!qEncoder) return false;                                  // Checks queue availability before adding data
  return (xQueueOverwrite(qEncoder, &e) == pdTRUE);             // Overwrites previous encoder value/event -> always keeps latest (Snapshot)
                                                               // Recommended: Producer >>> Encoder controller task
}

bool encoderRecvEvent(EncodeEvent_t &out, TickType_t to) {
  if (!qEncoder) return false;                                  // Checks queue availability before reading data
  return (xQueuePeek(qEncoder, &out, to) == pdTRUE);            // Reads latest encoder event without removing it (Peek)
                                                               // Recommended: Consumer >>> Display task
}


// ---- Sensors DataModel (Overwrite by Sensors, Peek by Display + MQTT) ----
bool sensorsModelWrite(const SensorData_t &d) {
  if (!qSensorsDataModel) return false;                         // Checks queue availability before adding data
  return (xQueueOverwrite(qSensorsDataModel, &d) == pdTRUE);    // Overwrites previous snapshot -> always keeps latest sensor values (Snapshot)
                                                               // Recommended: Producer >>> Sensors task
}

bool sensorsModelPeek(SensorData_t &out, TickType_t to) {
  if (!qSensorsDataModel) return false;                         // Checks queue availability before reading data
  return (xQueuePeek(qSensorsDataModel, &out, to) == pdTRUE);   // Reads latest snapshot without removing it (Peek)
                                                               // Recommended: Consumers >>> Display task + MQTT task
}


// ---- Config commands (Overwrite by MQTT, Peek by Sensors + Display controller) ----
bool configSend(const ConfigCmd_t &cmd, TickType_t to) {
  (void)to;                                                     // Not required for overwrite API
  if (!qConfig) return false;                                   // Checks queue availability before adding data
  return (xQueueOverwrite(qConfig, &cmd) == pdTRUE);            // Overwrites previous config -> always keeps latest config (Snapshot)
                                                               // Recommended: Producer >>> MQTT task
}

bool configRecv(ConfigCmd_t &out, TickType_t to) {
  if (!qConfig) return false;                                   // Checks queue availability before reading data
  return (xQueuePeek(qConfig, &out, to) == pdTRUE);             // Reads latest config without removing it (Peek)
                                                               // Recommended: Consumers >>> Sensors controller + Display controller
}


// ---- Mode context (Overwrite by Display, Peek by Logic controller) ----
bool modeCtxWrite(const ModeCtx_t &ctx) {
  if (!qModeCtx) return false;                                  // Checks queue availability before adding data
  return (xQueueOverwrite(qModeCtx, &ctx) == pdTRUE);           // Overwrites previous mode -> always keeps latest mode selection/state (Snapshot)
                                                               // Recommended: Producer >>> Display task
}

bool modeCtxPeek(ModeCtx_t &out, TickType_t to) {
  if (!qModeCtx) return false;                                  // Checks queue availability before reading data
  return (xQueuePeek(qModeCtx, &out, to) == pdTRUE);            // Reads latest mode without removing it (Peek)
                                                               // Recommended: Consumer >>> Logic controller task
}

// ---- Actuator Alarm (Overwrite by Logic Control, Peek by MQTT) ----
bool modeCtxWrite(const ActuatorAlarm_t &ctx) {
  if (!qAlarm) return false;                                  // Checks queue availability before adding data
  return (xQueueOverwrite(qAlarm, &ctx) == pdTRUE);           // Overwrites previous mode -> always keeps latest mode selection/state (Snapshot)
                                                               // Recommended: Producer >>> Logic Controller
}

bool modeCtxPeek(ActuatorAlarm_t &out, TickType_t to) {
  if (!qAlarm) return false;                                  // Checks queue availability before reading data
  return (xQueuePeek(qAlarm, &out, to) == pdTRUE);            // Reads latest mode without removing it (Peek)
                                                               // Recommended: Consumer >>> MQTT
}
