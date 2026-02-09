

// Project version
#define APP_NAME        "ConnectedGreenhouse"
#define APP_VERSION     "v1.0.0"
#define APP_BUILD_DATE  __DATE__
#define APP_BUILD_TIME  __TIME__


void setup() {

  Serial.begin(115200);
  delay(200);

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

}

void loop() {
  delay(1000);
}