#include <Arduino.h>
#include "LogicTask.h"
#include "RTOSQueues.h"
#include "gpio.h"

// Thresholds for Auto Mode
const float TEMP_HYSTERESIS = 0.5f; 
const float HUM_HYSTERESIS  = 2.0f;

static TaskHandle_t hLogic = nullptr;
static uint32_t TimerValue =0;
static const uint32_t LogTimer=5000;

static void applyOutputs(bool fanOn, bool ledOn, uint8_t heaterPwm) {
    digitalWrite(GPIO_FAN, fanOn ? HIGH : LOW);
    digitalWrite(GPIO_BOARD_LED, ledOn ? HIGH : LOW);
    analogWrite(GPIO_PWM_HEATER, heaterPwm);
}

static void logicTask(void *pv) {
    Serial.println("Rtos::Task PLC Logic > started.");
    
    ConfigCmd_t   config{};
    SensorData_t  sensors{};
    ModeCtx_t     ctx{}; 

    // Initialize system to safe OFF state
    applyOutputs(false, false, 0);

    while (1) {
        // Fetch latest instructions and sensor data
        configRecv(config, 0);  
        sensorsModelPeek(sensors, 0);
        // modeCtxPeek(ctx, 0);

        bool nextFan     = ctx.fanOn;
        bool nextLed     = ctx.ledOn;
        uint8_t nextHeater = ctx.heaterPwm;

        switch (config.mode) {
            case MODE_OFF:
                nextFan = false;
                nextHeater = 0;
                nextLed = false;
                break;

            case MODE_MANUAL:
                /**
                 * MANUAL CONTROL:
                 * We take the values directly from the Config command.
                 * The Logic task does not change these based on sensors.
                 */
                nextFan    = config.fanOn;
                nextHeater = config.heaterPwm; 
                // In manual, LED is usually turned on to indicate the system is active
                // or controlled by a specific manual light bit if available.
                nextLed    = config.ledOn; 
                break;

            case MODE_AUTO:
                // --- Temperature Hysteresis (Heater) ---
                if (sensors.temperature < (config.tempTarget_C - TEMP_HYSTERESIS)) {
                    nextHeater = 180; // Stable power-on value
                } else if (sensors.temperature > (config.tempTarget_C + TEMP_HYSTERESIS)) {
                    nextHeater = 0;
                }

                // --- Humidity Hysteresis (Fan) ---
                if (sensors.humidity > (config.humTarget_RH + HUM_HYSTERESIS)) {
                    nextFan = true;
                } else if (sensors.humidity < (config.humTarget_RH - HUM_HYSTERESIS)) {
                    nextFan = false;
                }

                // --- Light Control (LED) ---
                nextLed = (sensors.lux < config.luxTarget);
                break;
        }

        // Apply to Physical Pins
        applyOutputs(nextFan, nextLed, nextHeater);

        // Update the State Structure (ModeCtx) for the rest of the system to see
        ctx.mode          = config.mode;
        ctx.fanOn         = nextFan;
        ctx.heaterOn      = (nextHeater > 0);
        ctx.ledOn        = nextLed;
        ctx.heaterPwm     = nextHeater;
        ctx.lastUpdate_ms = millis();
        
        // Update Label
        if (ctx.mode == MODE_AUTO)      strcpy(ctx.modeLabel, "AUTO");
        else if (ctx.mode == MODE_MANUAL) strcpy(ctx.modeLabel, "MANUAL");
        else                            strcpy(ctx.modeLabel, "OFF");

        // Overwrite the system state snapshot
        modeCtxWrite(ctx);

        if((millis()-TimerValue)>LogTimer){
            TimerValue=millis();
            Serial.printf("LogicTask:: Fan: %s , Heater %s, Light %s\n", ctx.fanOn?"ON":"OFF",ctx.heaterOn?"ON":"OFF",  ctx.heaterOn?"ON":"OFF");
        }


        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void logicInit() {
    pinMode(GPIO_FAN, OUTPUT);
    pinMode(GPIO_BOARD_LED, OUTPUT);
    pinMode(GPIO_PWM_HEATER, OUTPUT);
    Serial.println("LogicTask::Init ✓ completed.");
}

void logicStartTask() {
    xTaskCreate(logicTask, "Logic", 4096, nullptr, 3, &hLogic);
}