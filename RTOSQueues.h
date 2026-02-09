#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "globals.h"


extern QueueHandle_t qSensorsDataModel;  // SensorData_t (latest)
extern QueueHandle_t qEncoder;           // EncodeEvent_t (events)
extern QueueHandle_t qConfig;            // ConfigCmd_t (commands)
extern QueueHandle_t qModeCtx;           // ModeCtx_t (latest)
extern QueueHandle_t qActuatorCmd;       // ActuatorCmd_t (latest)


bool queuesCreateSensors();
bool queuesCreateEncoder();
bool queuesCreateConfig();
bool queuesCreateModeCtx();
bool queuesCreateActuator();


bool queuesCreateAll();
bool queuesReadyAll();


// ---- Sensors DataModel queue (length 1, overwrite latest) ----
bool sensorsModelWrite(const SensorData_t &d);                  // producer: SensorsTask
bool sensorsModelPeek(SensorData_t &out, TickType_t to = 0);    // consumer: Logic/Display/Radio

// ---- Encoder queue (events, length > 1) ----
bool encoderSendEvent(EncodeEvent_t e, TickType_t to = 0);      // producer: EncoderTask
bool encoderRecvEvent(EncodeEvent_t &out, TickType_t to);       // consumer: Display/Logic

// ---- Config queue (commands, length > 1) ----
bool configSend(const ConfigCmd_t &cmd, TickType_t to = 0);     // producer: Display/Radio
bool configRecv(ConfigCmd_t &out, TickType_t to);               // consumer: PLC Logic

// ---- ModeCtx queue (length 1, overwrite latest) ----
bool modeCtxWrite(const ModeCtx_t &ctx);                        // producer: PLC Logic
bool modeCtxPeek(ModeCtx_t &out, TickType_t to = 0);            // consumer: Display/Radio

// ---- ActuatorCmd queue (length 1, overwrite latest) ----
bool actuatorWrite(const ActuatorCmd_t &cmd);                   // producer: PLC Logic
bool actuatorPeek(ActuatorCmd_t &out, TickType_t to = 0);       // consumer: GPIO task (if split)
