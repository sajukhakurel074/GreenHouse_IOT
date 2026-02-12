#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radioMqttTask.h"
#include "RTOSQueues.h"
#include "globals.h"

// ===================== Configuration =====================
static const char* WIFI_SSID = "Cudy-5B0C";
static const char* WIFI_PASS = "0477060671";

static const char* MQTT_HOST = "192.168.10.102";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USER = "greenhouse-user";
static const char* MQTT_PASS = "greenhouse-password";

static const char* BIRTH_STATUS = "ONLINE";
static const char* DEATH_STATUS = "OFFLINE";

// Topics
static const char* PRESENCE_TOPIC = "greenhouse/physical/state/connection";
static const char* TOPIC_GREENHOUSE_STATE = "greenhouse/physical/state/environment";
static const char* TOPIC_SERVER_COMMANDS = "greenhouse/commands/";

// Timeouts / pacing
static const uint32_t WIFI_TIMEOUT_MS = 20000;
static const uint32_t MQTT_TIMEOUT_MS = 15000;
static const uint32_t TASK_DELAY_MS   = 50;

// Telemetry rate limit
static const uint32_t MIN_PUB_MS = 1000;  // max 1 publish per second

// MQTT reconnect backoff
static const uint32_t MQTT_RECONNECT_BACKOFF_MS = 2000;

// ===================== State =====================
static String deviceId;        // base id (stable)
static String clientId;        // actual MQTT client id (unique per boot)
static String presenceTopic;
static String pubTopic;
static String subTopic;

static WiFiClient netClient;
static PubSubClient mqtt(netClient);

static TaskHandle_t hMqttTask = nullptr;

// Presence once per boot
static bool birthSent = false;

// Track connection state changes (to avoid log spam)
static bool wasMqttConnected = false;

// ===================== MQTT Callback =====================
static void onMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);
}

// ===================== Connectivity =====================
static bool ensureWiFi(uint32_t timeoutMs = WIFI_TIMEOUT_MS) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(300));
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP=");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI=");
    Serial.println(WiFi.RSSI());
    return true;
  }

  Serial.print("WiFi FAILED. Status=");
  Serial.println((int)WiFi.status());
  return false;
}

static bool ensureMQTT(uint32_t timeoutMs = MQTT_TIMEOUT_MS) {
  if (mqtt.connected()) return true;

  // reconnect backoff
  static uint32_t lastAttemptMs = 0;
  uint32_t now = millis();
  if (now - lastAttemptMs < MQTT_RECONNECT_BACKOFF_MS) return false;
  lastAttemptMs = now;

  // Only print "connecting" once per disconnect period
  if (wasMqttConnected) {
    Serial.print("MQTT disconnected (state=");
    Serial.print(mqtt.state());
    Serial.println("). Reconnecting...");
    wasMqttConnected = false;
  }

  // LWT OFFLINE payload
  char willPayload[96];
  snprintf(willPayload, sizeof(willPayload),
           "{\"device_id\":\"%s\",\"status\":\"%s\"}",
           deviceId.c_str(), DEATH_STATUS);

  uint32_t start = millis();
  while (!mqtt.connected() && (millis() - start) < timeoutMs) {

    // IMPORTANT: use unique-per-boot clientId to avoid broker kicking us
    bool ok = mqtt.connect(
      clientId.c_str(),
      MQTT_USER, MQTT_PASS,
      presenceTopic.c_str(), 1, true,
      willPayload
    );

    if (ok) {
      wasMqttConnected = true;
      Serial.print("MQTT connected as ");
      Serial.println(clientId);

      mqtt.subscribe(subTopic.c_str(), 1);
      Serial.print("Subscribed: ");
      Serial.println(subTopic);

      // Birth message ONCE PER BOOT
      if (!birthSent) {
        String ts = String(millis()); // placeholder
        String onlinePayload =
          "{\"device_id\":\"" + deviceId +
          "\",\"timestamp_utc\":\"" + ts +
          "\",\"status\":\"" + String(BIRTH_STATUS) + "\"}";

        mqtt.publish(presenceTopic.c_str(), onlinePayload.c_str(), true);
        Serial.print("Presence published (boot): ");
        Serial.println(onlinePayload);

        birthSent = true;
      }

      return true;
    }

    Serial.print("MQTT connect failed, state=");
    Serial.println(mqtt.state());
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  return mqtt.connected();
}

// ===================== RTOS Task =====================
static void mqttTask(void* pv) {
  Serial.println("RTOS MQTT task started.");

  static uint32_t seq = 1;
  static uint32_t lastSensorTs = 0;
  static uint32_t lastPubMs = 0;

  for (;;) {
    mqtt.loop();

    if (!ensureWiFi()) {
      if (mqtt.connected()) mqtt.disconnect();
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    if (!ensureMQTT()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ---- Telemetry: new sensor snapshot, max 1/sec ----
    SensorData_t s;
    ModeCtx_t m;

    if (sensorsModelPeek(s, 0)) {
      if (s.ts_ms != lastSensorTs) {
        lastSensorTs = s.ts_ms;

        uint32_t now = millis();
        if (now - lastPubMs >= MIN_PUB_MS) {
          lastPubMs = now;

          bool fanEnabled = false;
          uint8_t heaterPwm = 0;

          if (modeCtxPeek(m, 0)) {
            fanEnabled = m.fanOn;      // or (m.fanPwm > 0)
            heaterPwm  = m.heaterPwm;  // 0..100
          }

          char payload[512];
          snprintf(payload, sizeof(payload),
            "{"
              "\"device_id\":\"%s\","
              "\"timestamp_utc\":\"%lu\","
              "\"seq\":%lu,"
              "\"sensors\":{"
                "\"ambient_light_level_lux\":%.2f,"
                "\"air_temperature_celsius\":%.2f,"
                "\"relative_humidity_percent\":%.2f"
              "},"
              "\"actuators\":{"
                "\"ventilation_fan_enabled\":%s,"
                "\"heater_pwm_percent\":%u"
              "}"
            "}",
            deviceId.c_str(),
            (unsigned long)s.ts_ms,
            (unsigned long)seq++,
            s.lux, s.temperature, s.humidity,
            fanEnabled ? "true" : "false",
            (unsigned)heaterPwm
          );

          bool ok = mqtt.publish(pubTopic.c_str(), payload, false);

          Serial.print("MQTT TX [");
          Serial.print(pubTopic);
          Serial.print("] ");
          Serial.println(ok ? "✓" : "✗");
          Serial.println(payload);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
  }
}

// ===================== Public API =====================
void mqttInit() {
  const uint64_t mac = ESP.getEfuseMac();
  deviceId = "heltec-v3-" + String((uint32_t)(mac & 0xFFFFFFFF), HEX);

  // ✅ Make clientId unique per boot to avoid broker kicking us
  clientId = deviceId + "-" + String((uint32_t)esp_random(), HEX);

  presenceTopic = String(PRESENCE_TOPIC);
  pubTopic      = String(TOPIC_GREENHOUSE_STATE);
  subTopic      = String(TOPIC_SERVER_COMMANDS) + deviceId + "/#";

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(1024);

  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(5);

  birthSent = false;
  wasMqttConnected = false;

  Serial.println("===== MQTT CONFIG =====");
  Serial.print("DeviceId: ");       Serial.println(deviceId);
  Serial.print("ClientId: ");       Serial.println(clientId);
  Serial.print("PresenceTopic: ");  Serial.println(presenceTopic);
  Serial.print("PublishTopic: ");   Serial.println(pubTopic);
  Serial.print("SubscribeTopic: "); Serial.println(subTopic);
  Serial.println("=======================");
}

void mqttStartTask() {
  if (hMqttTask != nullptr) {
    Serial.println("MQTT task already running.");
    return;
  }
  xTaskCreate(mqttTask, "mqttTask", 6144, nullptr, 2, &hMqttTask);
}