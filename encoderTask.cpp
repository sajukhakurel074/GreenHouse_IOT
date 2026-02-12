// ============================================================================
//  encoderTask.cpp — Student 1: Rotary Encoder + Button → FreeRTOS Queue
//  Board : Heltec WiFi LoRa 32 V3 (ESP32-S3)
//  Pins  : CLK = GPIO 20 (J2:17), DT = GPIO 19 (J2:18), BTN = GPIO 0 (J2:08)
// ============================================================================

#include <Arduino.h>
#include <RotaryEncoder.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF driver — needed to reclaim GPIO 19/20 from USB peripheral
#include "driver/gpio.h"

#include "encoderTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

// Parameters for Button Debouncing
#define DEBOUNCE_MS       50
#define LONG_PRESS_MS   1000

// ==========================================
// STATIC VARIABLES
// ==========================================

static RotaryEncoder *pEncoder = nullptr;
static TaskHandle_t   hEncoder = nullptr;

// ==========================================
// INTERRUPT SERVICE ROUTINE
// ==========================================

static volatile bool encoderMoved = false;

void IRAM_ATTR checkPosition() {
  encoderMoved = true;
}

// ==========================================
// MAIN TASK
// ==========================================

static void encoderTask(void *pv) {
  Serial.println("Rtos::Task Rotary encoder > started.");
  Serial.printf("  CLK=GPIO%d  DT=GPIO%d  BTN=GPIO%d\n",
                GPIO_ENC_CLK, GPIO_ENC_DT, GPIO_BOARD_BP);
  Serial.printf("  Initial: CLK=%d DT=%d BTN=%d\n",
                digitalRead(GPIO_ENC_CLK), digitalRead(GPIO_ENC_DT),
                digitalRead(GPIO_BOARD_BP));

  int lastPos = 0;

  // Button state
  bool     lastBtnState     = HIGH;
  uint32_t pressStartTime   = 0;
  bool     longPressHandled = false;

  // Event pulse mechanism
  bool     eventActive = false;
  uint32_t clearEventAt = 0;

  // Heartbeat timer
  uint32_t lastDiagMs = 0;

  pEncoder->tick();
  lastPos = pEncoder->getPosition();

  while (1) {
    bool          eventSent    = false;
    EncodeEvent_t currentEvent = ENC_NONE;

    // --- Heartbeat every 5 seconds ---
    if (millis() - lastDiagMs > 5000) {
      Serial.printf("[ENC] CLK=%d DT=%d BTN=%d pos=%d\n",
                    digitalRead(GPIO_ENC_CLK), digitalRead(GPIO_ENC_DT),
                    digitalRead(GPIO_BOARD_BP), pEncoder->getPosition());
      lastDiagMs = millis();
    }

    // --- 0. Event Auto-Clear ---
    if (eventActive && millis() > clearEventAt) {
      encoderSendEvent(ENC_NONE);
      eventActive = false;
    }

    // --- 1. Encoder Rotation ---
    if (encoderMoved) {
      encoderMoved = false;
    }
    pEncoder->tick();

    int newPos = pEncoder->getPosition();
    if (newPos != lastPos) {
      if (newPos > lastPos) {
        currentEvent = ENC_UP;
        Serial.println("Encoder > UP");
      } else {
        currentEvent = ENC_DOWN;
        Serial.println("Encoder > DOWN");
      }
      lastPos   = newPos;
      eventSent = true;
    }

    // --- 2. Button Handling ---
    int btnState = digitalRead(GPIO_BOARD_BP);

    if (lastBtnState == HIGH && btnState == LOW) {
      pressStartTime   = millis();
      longPressHandled = false;
    }
    else if (btnState == LOW && !longPressHandled) {
      if ((millis() - pressStartTime) > LONG_PRESS_MS) {
        Serial.println("Button > LONG SELECT");
        currentEvent     = ENC_SELECT_LONG;
        eventSent        = true;
        longPressHandled = true;
      }
    }
    else if (lastBtnState == LOW && btnState == HIGH && !longPressHandled) {
      if ((millis() - pressStartTime) > DEBOUNCE_MS) {
        Serial.println("Button > SELECT");
        currentEvent = ENC_SELECT;
        eventSent    = true;
      }
    }

    lastBtnState = btnState;

    // --- 3. Dispatch Event ---
    if (eventSent) {
      encoderSendEvent(currentEvent);
      clearEventAt = millis() + 150;
      eventActive  = true;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ==========================================
// INIT
// ==========================================

// void encoderInit() {
//   // =====================================================================
//   // GPIO 19/20 are shared with USB on ESP32-S3.
//   // Since Heltec V3 uses CP2102 (GPIO 43/44) for serial,
//   // we MUST reclaim GPIO 19/20 from the USB peripheral.
//   // =====================================================================

//   // Reset GPIO from any peripheral assignment back to plain GPIO
//   // (reclaims GPIO 19/20 from USB — safe because Heltec V3 uses CP2102 on GPIO 43/44)
//   gpio_reset_pin((gpio_num_t)GPIO_ENC_CLK);   // GPIO 20
//   gpio_reset_pin((gpio_num_t)GPIO_ENC_DT);    // GPIO 19

//   // Configure as digital input with pull-up
//   gpio_set_direction((gpio_num_t)GPIO_ENC_CLK, GPIO_MODE_INPUT);
//   gpio_set_pull_mode((gpio_num_t)GPIO_ENC_CLK, GPIO_PULLUP_ONLY);
//   gpio_set_direction((gpio_num_t)GPIO_ENC_DT,  GPIO_MODE_INPUT);
//   gpio_set_pull_mode((gpio_num_t)GPIO_ENC_DT,  GPIO_PULLUP_ONLY);

//   // Also set Arduino layer
//   pinMode(GPIO_ENC_CLK,  INPUT_PULLUP);
//   pinMode(GPIO_ENC_DT,   INPUT_PULLUP);
//   pinMode(GPIO_BOARD_BP, INPUT_PULLUP);
//   delay(10);

//   Serial.printf("Encoder pins: CLK=GPIO%d(%d) DT=GPIO%d(%d) BTN=GPIO%d(%d)\n",
//                 GPIO_ENC_CLK, digitalRead(GPIO_ENC_CLK),
//                 GPIO_ENC_DT,  digitalRead(GPIO_ENC_DT),
//                 GPIO_BOARD_BP, digitalRead(GPIO_BOARD_BP));

//   // Create encoder object
//   pEncoder = new RotaryEncoder(GPIO_ENC_DT, GPIO_ENC_CLK,
//                                RotaryEncoder::LatchMode::FOUR0);

//   // Attach edge-triggered interrupts on both pins
//   attachInterrupt(digitalPinToInterrupt(GPIO_ENC_CLK), checkPosition, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(GPIO_ENC_DT),  checkPosition, CHANGE);

//   Serial.println("Main::initEncoder completed.");
// }

void encoderInit() {
  pinMode(GPIO_ENC_CLK,  INPUT_PULLUP);
  pinMode(GPIO_ENC_DT,   INPUT_PULLUP);
  pinMode(GPIO_BOARD_BP, INPUT_PULLUP);
  delay(10);

  Serial.printf("Encoder pins: CLK=GPIO%d(%d) DT=GPIO%d(%d) BTN=GPIO%d(%d)\n",
                GPIO_ENC_CLK, digitalRead(GPIO_ENC_CLK),
                GPIO_ENC_DT,  digitalRead(GPIO_ENC_DT),
                GPIO_BOARD_BP, digitalRead(GPIO_BOARD_BP));

  pEncoder = new RotaryEncoder(GPIO_ENC_DT, GPIO_ENC_CLK,
                               RotaryEncoder::LatchMode::FOUR0);

  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_CLK), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_DT),  checkPosition, CHANGE);

  Serial.println("Main::initEncoder completed.");
}


void encoderStartTask() {
  xTaskCreate(encoderTask, "Encoder", 3072, nullptr, 1, &hEncoder);
}
