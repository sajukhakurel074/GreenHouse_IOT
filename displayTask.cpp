#include <Wire.h>
#include "HT_SSD1306Wire.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "displayTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

#define PRG_BUTTON 0


static TaskHandle_t hDisplay = nullptr;

#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);  // addr , freq , i2c group , resolution , rst
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);  // addr , freq , i2c group , resolution , rst
#endif



// ===== MENU =====
const char *menuItems[] = {
  "Temperature :",
  "Humidity :",
  "Light :",
  "Fan ON",
  "Fan OFF",
  "Heater ON",
  "Heater OFF"
};

const int menuSize = 7;
static int selected = 0;

// ===== ACTUATOR STATE (sent to LogicTask via qModeCtx) =====
static bool fanState    = false;
static bool heaterState = false;



// =======================
// ===== DRAW MENU =======
// =======================
// Scrolling window: 128x64 OLED fits 5 rows at 12px each
#define VISIBLE_ROWS  5
#define ROW_HEIGHT   12

void drawMenu(const SensorData_t &sensors) {
  display.clear();

  // Calculate scroll window so selected item is always visible
  int topIndex = 0;
  if (selected >= VISIBLE_ROWS) {
    topIndex = selected - VISIBLE_ROWS + 1;
  }

  for (int v = 0; v < VISIBLE_ROWS && (topIndex + v) < menuSize; v++) {
    int i = topIndex + v;
    int y = v * ROW_HEIGHT;

    if (i == selected) {
      display.setColor(WHITE);
      display.fillRect(0, y, 128, ROW_HEIGHT);
      display.setColor(BLACK);
    } else {
      display.setColor(WHITE);
    }

    display.drawString(2, y, menuItems[i]);

    // Sensor values for first 3 items, status for actuators
    String value = "";

    switch (i) {
      case 0: value = String(sensors.temperature, 1) + "C"; break;
      case 1: value = String(sensors.humidity, 1) + "%"; break;
      case 2: value = String((int)sensors.lux); break;
      case 3: value = fanState    ? "[*]" : ""; break;  // Fan ON
      case 4: value = !fanState   ? "[*]" : ""; break;  // Fan OFF
      case 5: value = heaterState ? "[*]" : ""; break;  // Heater ON
      case 6: value = !heaterState? "[*]" : ""; break;  // Heater OFF
    }

    if (i == selected) {
      display.setColor(BLACK);
    }

    display.drawString(90, y, value);
  }

  // Scroll indicators
  display.setColor(WHITE);
  if (topIndex > 0) display.drawString(120, 0, "^");
  if (topIndex + VISIBLE_ROWS < menuSize) display.drawString(120, 52, "v");

  display.display();
}

// =======================
// ===== DISPLAY TASK ====
// =======================

// Helper: send current actuator state to LogicTask via qModeCtx
static void sendActuatorCommand() {
  ModeCtx_t ctx = {};
  ctx.mode        = MODE_MANUAL;
  ctx.fanOn       = fanState;
  ctx.heaterOn    = heaterState;
  ctx.heaterPwm   = heaterState ? 200 : 0;   // PWM 0-255 (adjust as needed)
  ctx.fanPwm      = fanState    ? 255 : 0;
  ctx.lastUpdate_ms = millis();
  strncpy(ctx.modeLabel, "MANUAL", sizeof(ctx.modeLabel) - 1);

  modeCtxWrite(ctx);
  Serial.printf("[DISPLAY] Command sent -> Fan:%s  Heater:%s\n",
                fanState ? "ON" : "OFF", heaterState ? "ON" : "OFF");
}

static void displayTask(void *pv) {
  Serial.println("Rtos::Task Display > started.");

  SensorData_t sensorData = {};
  EncodeEvent_t encoderEvent;

  // Send initial state (everything OFF)
  sendActuatorCommand();

  while (1) {

    // Read sensors
    sensorsModelPeek(sensorData, 0);

    // Read encoder event
    if (encoderRecvEvent(encoderEvent, 0)) {

      switch (encoderEvent) {

        case ENC_UP:
          if (selected > 0) selected--;
          delay(100);
          break;

        case ENC_DOWN:
          if (selected < menuSize - 1) selected++;
          delay(100);
          break;

        case ENC_SELECT:
          Serial.printf("Selected: %s\n", menuItems[selected]);

          // Execute action based on selected menu item
          switch (selected) {
            case 3:  // Fan ON
              fanState = true;
              sendActuatorCommand();
              break;
            case 4:  // Fan OFF
              fanState = false;
              sendActuatorCommand();
              break;
            case 5:  // Heater ON
              heaterState = true;
              sendActuatorCommand();
              break;
            case 6:  // Heater OFF
              heaterState = false;
              sendActuatorCommand();
              break;
            default:
              // Sensor items (0-2): display only, no action
              break;
          }
          delay(100);
          break;

        case ENC_SELECT_LONG:
          // Long press = emergency OFF (all actuators off)
          fanState    = false;
          heaterState = false;
          sendActuatorCommand();
          selected = 0;
          Serial.println("[DISPLAY] LONG PRESS -> ALL OFF");
          delay(100);
          break;
      }
    }

    // Draw menu
    drawMenu(sensorData);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =======================
// ===== INIT =========
// =======================

void displayInit() {
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);

  Serial.println("Main::initDisplay ✓ completed.");
}

void displayStartTask() {
  xTaskCreate(displayTask, "Display", 4096, nullptr, 1, &hDisplay);
}