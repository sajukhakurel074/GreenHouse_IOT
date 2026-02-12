#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cSensorsTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

// --------------------
// Libraries (TRY #1)
// --------------------
// 1) BH1750 by Christopher Laws (claws)
// 2) SHT31 by Rob Tillaart
#include <BH1750.h>    // claws/BH1750
#include <SHT31.h>     // RobTillaart/SHT31

#ifndef SENSOR_READ_INTERVAL_MS
#define SENSOR_READ_INTERVAL_MS 1000
#endif

// --------------------
// Dedicated sensors I2C bus (bus #1)
// --------------------
TwoWire I2CSensors(1);

// Addresses
static constexpr uint8_t ADDR_BH1750 = 0x23;
static constexpr uint8_t ADDR_SHT31  = 0x44;

// Sensor objects
static BH1750 lightMeter(ADDR_BH1750);        // claws library supports begin(..., &wire)
static SHT31  sht31(ADDR_SHT31, &I2CSensors); // RobTillaart supports TwoWire*

static TaskHandle_t hSensors = nullptr;

static uint32_t readCount  = 0;
static uint32_t errorCount = 0;

static bool g_bh_ok  = false;
static bool g_sht_ok = false;


// -------------------- Helpers --------------------
static bool i2cLookup(uint8_t address) {
  I2CSensors.beginTransmission(address);
  return (I2CSensors.endTransmission() == 0);
}

static void scanI2C() {
  Serial.println("Sensors::scanI2C Scanning I2C bus #1...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cLookup(addr)) {
      Serial.printf("Sensors::scanI2C Found 0x%02X\n", addr);
    }
  }
}

// -------------------- Health Check --------------------
bool checkSensorHealth() {
  // SHT31
  float t = sht31.getTemperature();
  float h = sht31.getHumidity();
  bool sht_ok = (!isnan(t) && !isnan(h));

  // BH1750
  float lux = lightMeter.readLightLevel();
  bool bh_ok = (lux >= 0.0f && lux < 65535.0f);

  // Attempt reinit if needed (fail-operational)
  if (!sht_ok) {
    Serial.println("Sensors::health SHT31 invalid -> reinit");
    g_sht_ok = sht31.begin();  // uses I2CSensors already (constructor)
    sht_ok = g_sht_ok;
  }
  if (!bh_ok) {
    Serial.println("Sensors::health BH1750 invalid -> reinit");
    g_bh_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, ADDR_BH1750, &I2CSensors);
    bh_ok = g_bh_ok;
  }

  return (sht_ok && bh_ok);
}

// -------------------- Task --------------------
static void sensorsTask(void *pv) {
  Serial.println("Rtos::Task Sensors > started.");
  vTaskDelay(pdMS_TO_TICKS(300));

  (void)checkSensorHealth();

  while (1) {
    digitalWrite(GPIO_BOARD_LED, HIGH);

    SensorData_t d{};
    d.ts_ms = millis();

    // Read sensors (use sentinels if not ok)
    if (g_sht_ok) {
      d.temperature = sht31.getTemperature();
      d.humidity    = sht31.getHumidity();
    } else {
      d.temperature = -999.0f;
      d.humidity    = -999.0f;
    }

    if (g_bh_ok) {
      d.lux = lightMeter.readLightLevel();
    } else {
      d.lux = -1.0f;
    }

    bool valid = (d.temperature > -500.0f) && (d.humidity > -500.0f) && (d.lux >= 0.0f);

    if (valid) {
      sensorsModelWrite(d);
      readCount++;

      if ((readCount % 10) == 0) {
        Serial.printf("Sensors::task #%lu: ts=%lu T=%.2fC H=%.2f%% Lux=%.0f\n",
                      (unsigned long)readCount,
                      (unsigned long)d.ts_ms,
                      d.temperature, d.humidity, d.lux);
      }
    } else {
      errorCount++;
      sensorsModelWrite(d);
      Serial.printf("Sensors::task invalid read #%lu\n", (unsigned long)errorCount);

      if ((errorCount % 10) == 0) {
        (void)checkSensorHealth();
      }
    }

    digitalWrite(GPIO_BOARD_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
  }
}

// -------------------- Init / Start --------------------
void sensorsInit() {
  Serial.println("Sensors::init (TRY #1) Using I2C bus #1 (TwoWire) for sensors...");

  // Init bus #1 on your chosen pins
  I2CSensors.begin(GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, 100000);

  scanI2C();

  // Init SHT31 (RobTillaart)
  Serial.print("Sensors::init SHT31... ");
  g_sht_ok = sht31.begin();
  Serial.println(g_sht_ok ? "OK" : "FAILED (continuing)");

  // Init BH1750 (claws)
  Serial.print("Sensors::init BH1750... ");
  g_bh_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, ADDR_BH1750, &I2CSensors);
  Serial.println(g_bh_ok ? "OK" : "FAILED (continuing)");

  Serial.println("Sensors::init Done (fail-operational).");
}

void sensorsStartTask() {
  xTaskCreate(sensorsTask, "Sensors", 4096, nullptr, 2, &hSensors);
  Serial.println(hSensors ? "Sensors task created" : "Failed to create sensors task");
}
