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
 * Manually pulses the SCL line to un-stick any sensors holding SDA low.
 * This is a hardware-level "clear" for the I2C bus.
 */
void recoverI2CBus() {
    Serial.println("Sensors::bus Pulse recovery sequence...");
    pinMode(GPIO_I2C_1_SCL, OUTPUT);
    pinMode(GPIO_I2C_1_SDA, INPUT_PULLUP);
    
    for (int i = 0; i < 10; i++) {
        digitalWrite(GPIO_I2C_1_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(GPIO_I2C_1_SCL, HIGH);
        delayMicroseconds(5);
    }
}

/**
 * Scans the bus to verify hardware presence.
 */
void scanBus() {
    Serial.println("Sensors::scan Scanning Bus #1...");
    for (uint8_t i = 1; i < 127; i++) {
        I2CSensors.beginTransmission(i);
        if (I2CSensors.endTransmission() == 0) {
            Serial.printf("Sensors::scan Found device at 0x%02X\n", i);
        }
    }
}

bool checkSensorHealth() {
    // SHT31 Health Check
    if (!sht31.read()) {
        Serial.println("Sensors::health SHT31 fail - attempting soft reset");
        // Manual I2C Reset command (standard for SHT3x)
        I2CSensors.beginTransmission(ADDR_SHT31);
        I2CSensors.write(0x30); 
        I2CSensors.write(0xA2); 
        I2CSensors.endTransmission();
        vTaskDelay(pdMS_TO_TICKS(50));
        g_sht_ok = sht31.begin();
    } else {
        g_sht_ok = true;
    }

    // BH1750 Health Check
    float lx = lux.getLux();
    g_lux_ok = (!isnan(lx) && lx >= 0.0f);
    if (!g_lux_ok) {
        g_lux_ok = lux.begin();
    }

    return (g_sht_ok && g_lux_ok);
}

static void sensorsTask(void *pv) {
    Serial.println("Rtos::Task Sensors > started.");
    vTaskDelay(pdMS_TO_TICKS(500));
    checkSensorHealth();

    while (1) {
        SensorData_t d{};
        d.ts_ms = millis();
        bool sht_valid = false;

        // 1. Read SHT31
        if (g_sht_ok && sht31.read()) {
            d.temperature = sht31.getTemperature();
            d.humidity    = sht31.getHumidity();
            sht_valid = true;
        } else {
            d.temperature = -999.0f;
            d.humidity    = -999.0f;
        }

        // 2. Read BH1750
        if (g_lux_ok) {
            d.lux = lux.getLux();
        } else {
            d.lux = -1.0f;
        }

        // Send to model if data is valid
        if (sht_valid || d.lux >= 0.0f) {
            sensorsModelWrite(d);
            readCount++;
            if ((readCount % 5) == 0) {
                Serial.printf("Sensors::#%lu T=%.2fC H=%.1f%% L=%.1f\n", 
                               readCount, d.temperature, d.humidity, d.lux);
            }
        } else {
            errorCount++;
            Serial.printf("Sensors::Critical Error #%lu\n", errorCount);
            checkSensorHealth();
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

void sensorsInit() {
    Serial.println("Sensors::init Starting I2C Bus...");

    // 1. Hardware Bus Recovery
    recoverI2CBus();

    // 2. Start I2C Controller
    I2CSensors.begin(GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, 100000);
    I2CSensors.setTimeOut(50); 

    // 3. Scan for hardware presence
    scanBus();

    // 4. Init SHT31
    g_sht_ok = sht31.begin();
    if(g_sht_ok) {
        Serial.println("Sensors::SHT31 [OK]");
    } else {
        Serial.println("Sensors::SHT31 [FAIL]");
    }

    // 5. Init BH1750
    g_lux_ok = lux.begin();
    if (g_lux_ok) {
        lux.setContHighRes();
        Serial.println("Sensors::BH1750 [OK]");
    } else {
        Serial.println("Sensors::BH1750 [FAIL]");
    }
}

void sensorsStartTask() {
    xTaskCreate(sensorsTask, "Sensors_Task", 4096, nullptr, 2, &hSensors);
}