#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cSensorsTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

// RobTillaart libs
#include <SHT31.h>
#include <BH1750FVI.h>   

#ifndef SENSOR_READ_INTERVAL_MS
#define SENSOR_READ_INTERVAL_MS 2000 
#endif

// Dedicated sensors bus: I2C controller #1
TwoWire I2CSensors(1);

static constexpr uint8_t ADDR_SHT31  = 0x44;
static constexpr uint8_t ADDR_BH1750 = 0x23;

// Sensor Objects
static SHT31 sht31(ADDR_SHT31, &I2CSensors);
static BH1750FVI lux(ADDR_BH1750, &I2CSensors);

static TaskHandle_t hSensors = nullptr;
static uint32_t readCount  = 0;
static uint32_t errorCount = 0;

static bool g_sht_ok = false;
static bool g_lux_ok = false;

/**
 * Validates sensor health and attempts re-initialization if communication is lost.
 */
bool checkSensorHealth() {
    // SHT31 Check: Call read() to verify the sensor is actually responding
    bool sht_ok = sht31.read(); 
    if (!sht_ok) {
        Serial.println("Sensors::health SHT31 re-init...");
        g_sht_ok = sht31.begin();
        if (g_sht_ok) {
            sht31.heatOff(); // Standardized check to ensure heater is off
        }
    } else {
        g_sht_ok = true;
    }

    // BH1750 Check: Attempt a dummy read
    float lx = lux.getLux();
    bool lux_ok = (!isnan(lx) && lx >= 0.0f);
    if (!lux_ok) {
        Serial.println("Sensors::health BH1750 re-init...");
        g_lux_ok = lux.begin();
    } else {
        g_lux_ok = true;
    }

    return (g_sht_ok && g_lux_ok);
}

/**
 * Main RTOS Task
 */
static void sensorsTask(void *pv) {
    Serial.println("Rtos::Task Sensors > started.");
    
    // Give I2C bus time to settle
    vTaskDelay(pdMS_TO_TICKS(500));
    checkSensorHealth();

    while (1) {
        SensorData_t d{};
        d.ts_ms = millis();
        bool sht_read_success = false;

        // 1. SHT31 Measurement
        if (g_sht_ok) {
            // Trigger the I2C transaction
            if (sht31.read()) {
                d.temperature = sht31.getTemperature();
                d.humidity    = sht31.getHumidity();
                sht_read_success = true;
            } else {
                d.temperature = -999.0f;
                d.humidity    = -999.0f;
            }
        }

        // 2. BH1750 Measurement
        if (g_lux_ok) {
            d.lux = lux.getLux();
        } else {
            d.lux = -1.0f;
        }

        // Validation Logic
        bool valid = (sht_read_success && d.lux >= 0.0f);

        if (valid) {
            sensorsModelWrite(d);
            readCount++;
            if ((readCount % 5) == 0) {
                Serial.printf("Sensors::Data [OK] T=%.2f C, H=%.2f %%, L=%.1f\n", 
                               d.temperature, d.humidity, d.lux);
            }
        } else {
            errorCount++;
            Serial.printf("Sensors::Error #%lu! Attempting recovery...\n", errorCount);
            checkSensorHealth();
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

void sensorsInit() {
    Serial.println("Sensors::init Starting I2C Bus 1...");

    // Initialize I2C Bus 1
    I2CSensors.begin(GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, 100000);

    // Initialize SHT31
    g_sht_ok = sht31.begin();
    if(g_sht_ok) {
        sht31.heatOff(); 
        Serial.println("Sensors::SHT31 [OK]");
    } else {
        Serial.println("Sensors::SHT31 [FAIL]");
    }

    // Initialize BH1750
    g_lux_ok = lux.begin();
    if(g_lux_ok) {
        lux.setContHighRes(); 
        Serial.println("Sensors::BH1750 [OK]");
    } else {
        Serial.println("Sensors::BH1750 [FAIL]");
    }
}

void sensorsStartTask() {
    xTaskCreate(sensorsTask, "Sensors_Task", 4096, nullptr, 2, &hSensors);
    if (hSensors) {
        Serial.println("Sensors Task Live.");
    }
}