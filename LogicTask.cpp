#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "LogicTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

static TaskHandle_t hLogic = nullptr;

static void applyOutputs(bool fanOn, bool ledOn, uint8_t heaterPwm)
{
    digitalWrite(GPIO_FAN, fanOn ? HIGH : LOW);              // MOSFET on GPIO_FAN (J3:17)
    digitalWrite(GPIO_BOARD_LED, ledOn ? HIGH : LOW);        // Board LED indicator
    analogWrite(GPIO_PWM_HEATER, heaterPwm);                 // PWM on GPIO_PWM_HEATER (J3:18)

    Serial.printf("[LOGIC] Outputs -> Fan:%s  Heater PWM:%d  LED:%s\n",
                  fanOn ? "ON" : "OFF", heaterPwm, ledOn ? "ON" : "OFF");
}

static void logicTask(void *pv)
{
    Serial.println("Rtos::Task PLC Logic > started.");
    // ConfigCmd_t   config{};
    // SensorData_t  sensors{};
    ModeCtx_t     ctx{};
    ModeCtx_t     prevCtx{};        // Track previous state to log only on change
    bool          firstRun = true;

    while (1)
    {
        // Peek latest mode context written by DisplayTask
        modeCtxPeek(ctx, 0);

        bool fanOn     = false;
        bool ledOn     = false;
        uint8_t heater = 0;

        switch (ctx.mode)
        {
            case MODE_OFF:
                fanOn  = false;
                ledOn  = false;
                heater = 0;
                break;

            case MODE_MANUAL:
                fanOn  = ctx.fanOn;
                heater = ctx.heaterPwm;
                ledOn  = (fanOn || heater > 0);  // LED on when any actuator is active
                break;

            case MODE_AUTO:
                // Future: automatic control based on sensor readings
                ledOn = true;
                break;

            default:
                break;
        }

        applyOutputs(fanOn, ledOn, heater);

        // Log only when state changes (avoid serial spam)
        if (firstRun || ctx.fanOn != prevCtx.fanOn || ctx.heaterPwm != prevCtx.heaterPwm || ctx.mode != prevCtx.mode) {
            Serial.printf("[LOGIC] Mode:%d Fan:%s Heater PWM:%d\n",
                          ctx.mode, fanOn ? "ON" : "OFF", heater);
            prevCtx  = ctx;
            firstRun = false;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void logicInit()
{
    Serial.println("Main::initPLC ✓ completed.");
}

void logicStartTask()
{
    xTaskCreate(logicTask, "Logic", 4096, nullptr, 3, &hLogic);
}
