#include <Arduino.h>
#include <math.h>

#include "LogicTask.h"
#include "RTOSQueues.h"
#include "gpio.h"

static TaskHandle_t hLogic = nullptr;

// Physical heater rating
static const float HEATER_MAX_W = 25.0f;
static const uint8_t PWM_MAX = 255;

static void applyOutputs(bool fanOn, uint8_t heaterPwm) {
  digitalWrite(GPIO_FAN, fanOn ? HIGH : LOW);
  analogWrite(GPIO_PWM_HEATER, heaterPwm);
}

static void logicTask(void *pv) {
  Serial.println("Rtos::Task PLC Logic > started (Solar regulation mode).");

  ConfigCmd_t   cfg{};
  bool          haveCfg = false;

  SensorData_t  sensors{};
  ModeCtx_t     ui{};
  ModeCtx_t     out{};

  applyOutputs(false, 0);

  while (1) {

    // Receive new configuration if available
    ConfigCmd_t tmpCfg;
    if (configRecv(tmpCfg, 0)) {
      cfg = tmpCfg;
      haveCfg = true;
    }

    sensorsModelPeek(sensors, 0);
    modeCtxPeek(ui, 0);

    bool serverControl = haveCfg && (cfg.accessCode != 0);

    Mode_t activeMode = serverControl ? cfg.mode : ui.mode;

    if (!serverControl && activeMode == MODE_AUTO) {
      activeMode = MODE_MANUAL;
    }

    ModeCtx_t current{};
    modeCtxPeek(current, 0);

    bool    nextFan    = current.fanOn;
    uint8_t nextHeater = current.heaterPwm;

    switch (activeMode) {

      case MODE_OFF:
        nextFan = false;
        nextHeater = 0;
        break;

      case MODE_MANUAL:
        if (serverControl) {
          nextFan    = cfg.fanOn;
          nextHeater = cfg.heaterPwm;
        } else {
          nextFan    = ui.fanOn;
          nextHeater = ui.heaterPwm;
        }
        break;

      case MODE_AUTO: {

        // ==============================
        // SOLAR IRRADIATION REGULATION
        // ==============================

        float lux = sensors.lux;
        if (isnan(lux) || lux < 0.0f) lux = 0.0f;

        // Server defines what is "full sun"
        float luxFull = (cfg.luxTarget > 0.0f) ? cfg.luxTarget : 1.0f;

        float norm = lux / luxFull;
        if (norm > 1.0f) norm = 1.0f;
        if (norm < 0.0f) norm = 0.0f;

        float P_target_W = norm * HEATER_MAX_W;

        // ==============================
        // HOT / COLD SAFETY
        // ==============================

        float T = sensors.temperature;

        float T_hot  = cfg.tempTarget_C + cfg.heaterRange_C;
        float T_cold = cfg.tempTarget_C - cfg.heaterRange_C;

        // Too hot → cut heater + ventilation
        if (T > T_hot) {
          P_target_W = 0.0f;
          nextFan = true;
        }

        // Too cold → force minimum heating (10% of max)
        if (T < T_cold) {
          float P_min = 0.1f * HEATER_MAX_W;
          if (P_target_W < P_min)
            P_target_W = P_min;
        }

        nextHeater = (uint8_t)((P_target_W / HEATER_MAX_W) * PWM_MAX);

        break;
      }

      default:
        nextFan = false;
        nextHeater = 0;
        break;
    }

    applyOutputs(nextFan, nextHeater);

    // Publish system state snapshot (NO extra fields added)
    memset(&out, 0, sizeof(out));
    out.mode          = activeMode;
    out.fanOn         = nextFan;
    out.heaterOn      = (nextHeater > 0);
    out.heaterPwm     = nextHeater;
    out.lastUpdate_ms = millis();

    if (out.mode == MODE_AUTO) strcpy(out.modeLabel, "AUTO");
    else if (out.mode == MODE_MANUAL) strcpy(out.modeLabel, "MANUAL");
    else strcpy(out.modeLabel, "OFF");

    modeCtxWrite(out);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void logicInit() {
  pinMode(GPIO_FAN, OUTPUT);
  pinMode(GPIO_PWM_HEATER, OUTPUT);
  Serial.println("LogicTask::Init ✓ completed.");
}

void logicStartTask() {
  xTaskCreate(logicTask, "Logic", 4096, nullptr, 3, &hLogic);
}

// #include <Arduino.h>
// #include "LogicTask.h"
// #include "RTOSQueues.h"
// #include "gpio.h"

// static TaskHandle_t hLogic = nullptr;

// static void applyOutputs(bool fanOn, bool ledOn, uint8_t heaterPwm) {
//   digitalWrite(GPIO_FAN, fanOn ? HIGH : LOW);
//   digitalWrite(GPIO_BOARD_LED, ledOn ? HIGH : LOW);
//   analogWrite(GPIO_PWM_HEATER, heaterPwm);
// }

// static void logicTask(void *pv) {
//   Serial.println("Rtos::Task PLC Logic > started.");

//   ConfigCmd_t   cfg{};
//   bool          haveCfg = false;

//   SensorData_t  sensors{};
//   ModeCtx_t     ui{};       // from encoder/display
//   ModeCtx_t     out{};      // published state snapshot

//   applyOutputs(false, false, 0);

//   uint32_t lastLog = 0;

//   while (1) {
//     // Update config only if a new message arrives
//     ConfigCmd_t tmpCfg;
//     if (configRecv(tmpCfg, 0)) {
//       cfg = tmpCfg;
//       haveCfg = true;
//     }

//     // Read latest sensors & UI commands
//     sensorsModelPeek(sensors, 0);
//     modeCtxPeek(ui, 0);

//     // Decide authority
//     bool serverControl = haveCfg && (cfg.accessCode != 0);

//     // Choose active mode based on authority
//     Mode_t activeMode = serverControl ? cfg.mode : ui.mode;

//     // In LOCAL authority, prevent AUTO from fighting encoder (treat AUTO as MANUAL)
//     if (!serverControl && activeMode == MODE_AUTO) {
//       activeMode = MODE_MANUAL;
//     }

//     // Start from last applied output state (prevents “reset to 0” bugs)
//     ModeCtx_t current{};
//     modeCtxPeek(current, 0);

//     bool    nextFan    = current.fanOn;
//     bool    nextLed    = current.ledOn;
//     uint8_t nextHeater = current.heaterPwm;

//     switch (activeMode) {
//       case MODE_OFF:
//         nextFan = false;
//         nextHeater = 0;
//         nextLed = false;
//         break;

//       case MODE_MANUAL:
//         if (serverControl) {
//           // SERVER manual (if you ever send cfg.fanOn/cfg.heaterPwm from server)
//           nextFan    = cfg.fanOn;
//           nextHeater = cfg.heaterPwm;
//           nextLed    = cfg.ledOn;
//         } else {
//           // LOCAL manual (encoder/display)
//           nextFan    = ui.fanOn;
//           nextHeater = ui.heaterPwm;
//           nextLed    = ui.ledOn;
//         }
//         break;

//       case MODE_AUTO: {
//         // ===== SIMPLE SERVER AUTO LOGIC (as you requested) =====
//         // Heater: ON if setpoint > actual temperature, else OFF
//         if (cfg.tempTarget_C > sensors.temperature) {
//           nextHeater = 255;  // use 180 if you want softer heating
//         } else {
//           nextHeater = 0;
//         }

//         // Fan: OFF if humidity setpoint > actual humidity, else ON
//         // (equivalently: ON when actual humidity > setpoint)
//         if (sensors.humidity > cfg.humTarget_RH) {
//           nextFan = true;
//         } else {
//           nextFan = false;
//         }

//         // LED rule (optional)
//         nextLed = (sensors.lux < cfg.luxTarget);

//         // Debug (uncomment if needed)
//         // Serial.printf("[AUTO_SIMPLE] T=%.2f SP_T=%.2f -> HeaterPWM=%u | RH=%.2f SP_RH=%.2f -> Fan=%s\n",
//         //   sensors.temperature, cfg.tempTarget_C, (unsigned)nextHeater,
//         //   sensors.humidity, cfg.humTarget_RH, nextFan ? "ON" : "OFF");
//       } break;

//       default:
//         // Safe fallback
//         nextFan = false;
//         nextHeater = 0;
//         nextLed = false;
//         break;
//     }

//     // Apply to pins
//     applyOutputs(nextFan, nextLed, nextHeater);

//     // Publish system state snapshot
//     memset(&out, 0, sizeof(out));
//     out.mode          = activeMode;
//     out.fanOn         = nextFan;
//     out.heaterOn      = (nextHeater > 0);
//     out.ledOn         = nextLed;
//     out.heaterPwm     = nextHeater;
//     out.lastUpdate_ms = millis();

//     if (out.mode == MODE_AUTO) strcpy(out.modeLabel, "AUTO");
//     else if (out.mode == MODE_MANUAL) strcpy(out.modeLabel, "MANUAL");
//     else strcpy(out.modeLabel, "OFF");

//     modeCtxWrite(out);

//     // Log occasionally
//     if (millis() - lastLog > 2000) {
//       lastLog = millis();
//       Serial.printf("[LOGIC] %s Mode:%s T=%.2f SP_T=%.2f RH=%.2f SP_RH=%.2f Fan:%s HeaterPWM:%d LED:%s\n",
//         serverControl ? "SERVER" : "LOCAL",
//         out.modeLabel,
//         sensors.temperature, cfg.tempTarget_C,
//         sensors.humidity, cfg.humTarget_RH,
//         out.fanOn ? "ON" : "OFF",
//         (int)out.heaterPwm,
//         out.ledOn ? "ON" : "OFF");
//     }

//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }

// void logicInit() {
//   pinMode(GPIO_FAN, OUTPUT);
//   pinMode(GPIO_BOARD_LED, OUTPUT);
//   pinMode(GPIO_PWM_HEATER, OUTPUT);
//   Serial.println("LogicTask::Init ✓ completed.");
// }

// void logicStartTask() {
//   xTaskCreate(logicTask, "Logic", 4096, nullptr, 3, &hLogic);
// }
