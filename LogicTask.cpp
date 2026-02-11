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
    digitalWrite(GPIO_FAN, fanOn ? HIGH : LOW);        // Digital control
    digitalWrite(GPIO_BOARD_LED, ledOn ? HIGH : LOW);        // Digital control

    analogWrite(GPIO_PWM_HEATER, heaterPwm);               // PWM control (0–255 depending on config)
}

static void logicTask(void *pv)
{
    Serial.println("Rtos::Task PLC Logic > started.");
    // ConfigCmd_t   config{};
    // SensorData_t  sensors{};
    ModeCtx_t     ctx{};
    while (1)
    {
        //configRecv(config, 0);   // Peek latest config (non-blocking)
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
                ledOn  = true;                 
                break;

            case MODE_AUTO:
                // ledOn = true;
                
                // if (sensors.temperature < ctx.tempTarget_C)
                // {
                //     heater = 180;              
                //     fanOn  = false;
                // }
                // else
                // {
                //     heater = 0;
                //     fanOn  = true;
                // }
                break;

            default:
                break;
        }
        applyOutputs(fanOn, ledOn, heater);

        // ctx.mode           = config.mode;
        // ctx.alarm          = ALARM_NONE;       // Can expand later
        // ctx.fanOn          = fanOn;
        // ctx.heaterOn       = (heater > 0);
        // ctx.fanPwm         = 0;
        // ctx.heaterPwm      = heater;
        // ctx.lastUpdate_ms  = millis();

        // strncpy(ctx.modeLabel,
        //         (config.mode == MODE_OFF)   ? "Mode Off" :
        //         (config.mode == MODE_AUTO)  ? "Mode Auto" :
        //                                       "Mode Manual",
        //         sizeof(ctx.modeLabel)-1);

        // modeCtxWrite(ctx);                   // Overwrite latest system state

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
