
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "displayTask.h"
#include "RTOSQueues.h"   
#include "globals.h"

static TaskHandle_t hDisplay = nullptr;

static void displayTask(void *pv) {
  Serial.println("Rtos::Task Display > started.");

  while (1) {
    // TODO: refresh screen using latest data
    // SensorData_t s; if (sensorsModelPeek(s)) { ... }
    // EncoderData_t s; if(.....)
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void displayInit() {
  // TODO: display begin(), clear, set font, etc.
  Serial.println("Main::initDisplay ✓ completed.");
}

void displayStartTask() {
  xTaskCreate(displayTask, "Display", 4096, nullptr, 1, &hDisplay);
}
