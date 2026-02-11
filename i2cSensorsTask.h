#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <BH1750.h>
#include "globals.h"
#include "gpio.h"

#define SHT31_I2C_ADDR  0x44
#define BH1750_I2C_ADDR 0x23
#define SENSOR_READ_INTERVAL_MS 1000    // Reduced 10000 to 1000, was too slow response time

void sensorsInit();
void sensorsStartTask();
bool checkSensorHealth();
//void scanI2C();               // Internal Function, Generally are static

extern Adafruit_SHT31 sht31;
extern BH1750 lightMeter;