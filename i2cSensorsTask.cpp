
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2CSensorsTask.h"
#include "RTOSQueues.h"   // for sensorsModelWrite(...)
#include "globals.h"


static TaskHandle_t hSensors = nullptr;

static void sensorsTask(void *pv) {
  Serial.println("Rtos::Task Sensors > started.");

  uint32_t seq = 0;

  while (1) {
    // TODO: read real sensors (SHT31, BH1750)
    SensorData_t d{};
    // d.temperature = 22.0f;
    // d.humidity    = 55.0f;
    // d.lux         = 100.0f;
    // d.ts_ms       = millis();

    sensorsModelWrite(d);  // overwrite latest snapshot, or use ID for each data to load only updated data in the queue

    seq++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sensorsInit() {
  // IMPORTANT later: only one place should call Wire.begin()
  // TODO: Wire.begin(SDA,SCL), probe sensors, init libs
  Serial.println("Main::initSensors ✓ completed.");
}

void sensorsStartTask() {
  xTaskCreate(sensorsTask, "Sensors", 4096, nullptr, 2, &hSensors);
}
