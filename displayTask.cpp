#include <Wire.h>
#include "HT_SSD1306Wire.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "displayTask.h"
#include "RTOSQueues.h"
#include "globals.h"

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
  "Fan :",
  "Heater :"
};

const int menuSize = 5;
static int selected = 0;



// =======================
// ===== DRAW MENU =======
// =======================

void drawMenu(const SensorData_t &sensors) {
  display.clear();
  for (int i = 0; i < menuSize; i++) {

    int y = i * 12;

    if (i == selected) {
      display.setColor(WHITE);
      display.fillRect(0, y, 128, 12);
      display.setColor(BLACK);
    } else {
      display.setColor(WHITE);
    }

    display.drawString(2, y, menuItems[i]);

    // Valeur capteur
    String value = "";

    switch (i) {
      case 0: value = String(sensors.temperature, 1) + "C"; break;
      case 1: value = String(sensors.humidity, 1) + "%"; break;
      case 2: value = String((int)sensors.lux); break;
    }

    if (i == selected) {
      display.setColor(BLACK);
    }

    display.drawString(90, y, value);
  }
  display.display();
}

// =======================
// ===== DISPLAY TASK ====
// =======================

static void displayTask(void *pv) {
  Serial.println("Rtos::Task Display > started.");

  Serial.println("Rtos::Task Display > started.");


  SensorData_t sensorData = {};
  EncodeEvent_t encoderEvent;



  while (1) {

    // Lire capteurs
    sensorsModelPeek(sensorData, 0);


    // Récupérer événement encoder
    if (encoderRecvEvent(encoderEvent, 0)) {

      switch (encoderEvent) {

        case ENC_UP:
          if (selected > 0) selected--;
          delay(100);
          break;

        case ENC_DOWN:
          if (selected < menuSize) selected++;
          delay(100);
          break;

        case ENC_SELECT:
          Serial.printf("Selected: %s\n", menuItems[selected]);
          delay(100);
          break;

        case ENC_SELECT_LONG:
          selected = 0;
          delay(100);
          break;
      }
    }

    // Dessiner menu
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