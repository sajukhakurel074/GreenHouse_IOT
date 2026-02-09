#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radioMqttTask.h"
#include "RTOSQueues.h"   
#include "globals.h"

static TaskHandle_t hMqtt = nullptr;

// -------------------- Task function (template) --------------------
static void mqttTask(void *pv) {
  Serial.println("Rtos::Task MQTT > started.");

  while (1) {
    // ============================================================
    // 1) READ LATEST DATA MODEL (Peek)
    //    - used to publish telemetry/state to server
    // ============================================================
    SensorData_t sensors{};
    ModeCtx_t mode{};

    const bool haveSensors = sensorsModelPeek(sensors, 0);
    const bool haveMode    = modeCtxPeek(mode, 0);

    (void)haveSensors;
    (void)haveMode;

    // TODO(Student 5): Publish to MQTT server
    // - connect WiFi
    // - connect MQTT broker
    // - publish sensors + mode context as JSON

    // ============================================================
    // 2) RECEIVE CONFIG FROM SERVER AND OVERWRITE CONFIG QUEUE
    //    - server -> device config command
    //    - overwrite latest config snapshot
    // ============================================================
    // TODO(Student 5): When a config message arrives from MQTT:
    // ConfigCmd_t cfg{};
    // cfg.type = ...;
    // cfg.ts_ms = millis();
    // configWriteLatest(cfg);   // overwrite latest config

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// -------------------- Public API --------------------
void mqttInit() {
  // TODO(Student 5): WiFi + MQTT client setup (server address, topics, callback)
  Serial.println("Main::initMQTT ✓ configured (template).");
}

void mqttStartTask() {
  // Stack/priority placeholders; tune later
  xTaskCreate(mqttTask, "MQTT", 6144, nullptr, 2, &hMqtt);
}
