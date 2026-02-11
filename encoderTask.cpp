#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoderTask.h"
#include "RTOSQueues.h"   
#include "globals.h"


static TaskHandle_t hEncoder = nullptr;

static void encoderTask(void *pv) {
  Serial.println("Rtos::Task Rotary encoder > started.");

  while (1) {
    // TODO: read real encoder and send events

    // encoderSendEvent(ENC_UP);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void encoderInit() {
  // TODO: pinMode / attach interrupts / encoder library init

  Serial.println("Main::initEncoder ✓ completed.");
}

void encoderStartTask() {
  xTaskCreate(encoderTask, "Encoder", 3072, nullptr, 1, &hEncoder);
}
