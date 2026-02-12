#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// Sensor task API
void sensorsInit();
void sensorsStartTask();

// Optional: health check hook
bool checkSensorHealth();