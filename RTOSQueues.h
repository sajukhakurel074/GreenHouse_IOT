#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "globals.h"  


extern QueueHandle_t qSensorsDataModel;   // Queue DataModel (Sensors -> Display/Logic/MQTT) : Overwrite + Peek
extern QueueHandle_t qEncoder;            // Encoder Queue (Encoder -> Display/Logic)       : Send + Receive (Events)
extern QueueHandle_t qConfig;             // Queue dataModel config (Display/MQTT -> Logic) : Overwrite + Peek
extern QueueHandle_t qModeCtx;            // modeCtx Queue (Logic -> Display/MQTT)          : Overwrite + Peek


bool queuesCreateSensors();
bool queuesCreateEncoder();
bool queuesCreateConfig();
bool queuesCreateModeCtx();

bool queuesCreateAll();     // Creates all queues simultaneously
bool queuesReadyAll();      // Checks if all queues are created


// -------- Loading and Fetching data in Queue -------- //

// ---- Sensors DataModel (Queue DataModel) ----
bool sensorsModelWrite(const SensorData_t &d);                    // Producer: Sensors Task (Overwrite)
bool sensorsModelPeek(SensorData_t &out, TickType_t to = 0);      // Consumer: Display/Logic/MQTT (Peek)


// ---- Encoder events (Encoder Queue) ----
bool encoderSendEvent(EncodeEvent_t e, TickType_t to = 0);        // Producer: Encoder Task (Send)
bool encoderRecvEvent(EncodeEvent_t &out, TickType_t to = 0);     // Consumer: Display/Logic (Receive)


// ---- Config commands (Queue dataModel config) ----
bool configSend(const ConfigCmd_t &cmd, TickType_t to = 0);       // Producer: Display/MQTT (Overwrite)
bool configRecv(ConfigCmd_t &out, TickType_t to = 0);             // Consumer: Logic (Peek)


// ---- Mode context (modeCtx Queue) ----
bool modeCtxWrite(const ModeCtx_t &ctx);                          // Producer: Logic (Overwrite)
bool modeCtxPeek(ModeCtx_t &out, TickType_t to = 0);              // Consumer: Display/MQTT (Peek)
