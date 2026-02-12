#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cSensorsTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

// RobTillaart libs (TRY #2)
#include <SHT31.h>
#include <BH1750FVI.h>   // from BH1750FVI_RT library (IMPORTANT: not BH1750.h)

#ifndef SENSOR_READ_INTERVAL_MS
#define SENSOR_READ_INTERVAL_MS 1000
#endif

// Dedicated sensors bus: I2C controller #1
TwoWire I2CSensors(1);

static constexpr uint8_t ADDR_SHT31  = 0x44;
static constexpr uint8_t ADDR_BH1750 = 0x23;

// Sensors
static SHT31 sht31(ADDR_SHT31, &I2CSensors);
static BH1750FVI lux(ADDR_BH1750, &I2CSensors);

static TaskHandle_t hSensors = nullptr;

static uint32_t readCount  = 0;
static uint32_t errorCount = 0;

static bool g_sht_ok = false;
static bool g_lux_ok = false;

static bool i2cLookup(uint8_t addr) {
  I2CSensors.beginTransmission(addr);
  return (I2CSensors.endTransmission() == 0);
}

static void scanI2C() {
  Serial.println("Sensors::scanI2C Scanning I2C bus #1...");
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cLookup(a)) {
      Serial.printf("Sensors::scanI2C Found 0x%02X\n", a);
    }
  }
}

bool checkSensorHealth() {
  bool sht_ok = false;
  bool lux_ok = false;

  // SHT31
  float t = sht31.getTemperature();
  float h = sht31.getHumidity();
  sht_ok = (!isnan(t) && !isnan(h));

  if (!sht_ok) {
    Serial.println("Sensors::health SHT31 invalid -> reinit");
    g_sht_ok = sht31.begin();
    sht_ok = g_sht_ok;
  }

  // BH1750FVI_RT
  float lx = lux.getLux();
  lux_ok = (!isnan(lx) && lx >= 0.0f && lx < 200000.0f);

  if (!lux_ok) {
    Serial.println("Sensors::health BH1750 invalid -> reinit");
    g_lux_ok = lux.begin();   // reinit
    lux_ok = g_lux_ok;
  }

  return (sht_ok && lux_ok);
}

static void sensorsTask(void *pv) {
  Serial.println("Rtos::Task Sensors > started.");
  vTaskDelay(pdMS_TO_TICKS(300));
  (void)checkSensorHealth();

  while (1) {
    SensorData_t d{};
    d.ts_ms = millis();

    if (g_sht_ok) {
      d.temperature = sht31.getTemperature();
      d.humidity    = sht31.getHumidity();
    } else {
      d.temperature = -999.0f;
      d.humidity    = -999.0f;
    }

    if (g_lux_ok) {
      d.lux = lux.getLux();
    } else {
      d.lux = -1.0f;
    }

    bool valid = (d.temperature > -500.0f) && (d.humidity > -500.0f) && (d.lux >= 0.0f);

    if (valid) {
      sensorsModelWrite(d);
      readCount++;
      if ((readCount % 10) == 0) {
        Serial.printf("Sensors::task #%lu: ts=%lu T=%.2fC H=%.2f%% Lux=%.0f\n",
                      (unsigned long)readCount, (unsigned long)d.ts_ms,
                      d.temperature, d.humidity, d.lux);
      }
    } else {
      errorCount++;
      sensorsModelWrite(d);
      Serial.printf("Sensors::task invalid read #%lu\n", (unsigned long)errorCount);
      if ((errorCount % 10) == 0) (void)checkSensorHealth();
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
  }
}

void sensorsInit() {
  Serial.println("Sensors::init (TRY #2) Using TwoWire(1) for sensors...");

  // init I2C bus #1
  I2CSensors.begin(GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, 100000);

  scanI2C();

  Serial.print("Sensors::init SHT31... ");
  g_sht_ok = sht31.begin();
  Serial.println(g_sht_ok ? "OK" : "FAIL");

  Serial.print("Sensors::init BH1750FVI... ");
  g_lux_ok = lux.begin();
  Serial.println(g_lux_ok ? "OK" : "FAIL");

  Serial.println("Sensors::init Done (fail-operational).");
}

void sensorsStartTask() {
  xTaskCreate(sensorsTask, "Sensors", 4096, nullptr, 2, &hSensors);
  Serial.println(hSensors ? "Sensors task created" : "Failed to create sensors task");
}
