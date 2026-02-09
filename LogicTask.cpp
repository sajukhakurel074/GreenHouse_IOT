
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "LogicTask.h"
#include "RTOSQueues.h"     // sensorsModelPeek(), modeCtxWrite(), actuatorWrite(), configRecv()
#include "globals.h"

static TaskHandle_t hLogic = nullptr;

static void logicTask(void *pv) {
  Serial.println("Rtos::Task PLC Logic > started.");

  while (1) {
    // TODO: consume sensors + config, decide actuators, publish ModeCtx

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void logicInit() {
  // TODO: default state, thresholds, outputs off
  Serial.println("Main::initPLC ✓ completed.");
}

void logicStartTask() {
  xTaskCreate(logicTask, "Logic", 4096, nullptr, 3, &hLogic);
}
