#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//--------------------------------------------- Include Personal Libraries ---------------------------------------------//
#include "gpio.h"
#include "globals.h"
#include "encoderTask.h"
#include "displayTask.h"
#include "i2cSensorsTask.h"
#include "radioMqttTask.h"
#include "RTOSQueues.h"
#include "LogicTask.h"


// Project version
#define APP_NAME        "ConnectedGreenhouse"
#define APP_VERSION     "v1.1.0"
#define APP_BUILD_DATE  __DATE__
#define APP_BUILD_TIME  __TIME__


void setup() {

  Serial.begin(115200);
  delay(200);

  //--------------------------------------------- Version Print ---------------------------------------------//
  Serial.println();
  Serial.println("=================================");
  Serial.print("App: ");
  Serial.println(APP_NAME);
  Serial.print("Version: ");
  Serial.println(APP_VERSION);
  Serial.print("Build: ");
  Serial.print(APP_BUILD_DATE);
  Serial.print(" ");
  Serial.println(APP_BUILD_TIME);
  Serial.println("=================================");


  //--------------------------------------------- Initialze the GPIO's ---------------------------------------------//
  initGPIO();
  Serial.println("Main::setup initGPIO() completed.");

  //--------------------------------------------- Modules Initialization ---------------------------------------------//
  encoderInit();
  sensorsInit();
  displayInit();
  logicInit();
  Serial.println("Main::setup Module Initialization completed.");

  //--------------------------------------------- Queues Creation ---------------------------------------------//
  queuesCreateAll();

  //--------------------------------------------- Task Creation ---------------------------------------------//
  encoderStartTask();
  sensorsStartTask();
  displayStartTask();
  logicStartTask();
  Serial.println("Main::setup Creation of Tasks completed.");

}

void loop() {
  delay(1000);
}