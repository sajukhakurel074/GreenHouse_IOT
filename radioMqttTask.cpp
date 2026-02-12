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

// Presence topic
static const char* PRESENCE_TOPIC = "greenhouse/physical/state/connection";

// Telemetry topic
static const char* TOPIC_GREENHOUSE_STATE = "greenhouse/physical/state/environment";

// Commands base
static const char* TOPIC_SERVER_COMMANDS = "greenhouse/commands/";

// Timeouts
static const uint32_t WIFI_TIMEOUT_MS = 20000;
static const uint32_t MQTT_TIMEOUT_MS = 15000;
static const uint32_t TASK_DELAY_MS   = 50;

// Rate limit (prevents extreme publishes)
static const uint32_t MIN_PUB_MS = 2000; // max 1 publish every 2 seconds

// ===================== State =====================
static String deviceId;
static String presenceTopic;
static String pubTopic;
static String subTopic;

static WiFiClient netClient;
static PubSubClient mqtt(netClient);

static TaskHandle_t hMqttTask = nullptr;

// ===================== MQTT Callback =====================
static void onMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);

  // (Later) parse commands + push to configSend() etc.
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

  if (presenceTopic.isEmpty() || pubTopic.isEmpty() || subTopic.isEmpty()) {
    Serial.println("ERROR: MQTT topics are empty. Did mqttInit() run?");
    return false;
  }

  // Stable clientId (prevents fake OFFLINE)
  const String clientId = deviceId;

  Serial.print("MQTT connecting as ");
  Serial.println(clientId);

  // LWT OFFLINE payload (device_id + status only)
  char willPayload[96];
  snprintf(willPayload, sizeof(willPayload),
           "{\"device_id\":\"%s\",\"status\":\"%s\"}",
           deviceId.c_str(), DEATH_STATUS);

  uint32_t start = millis();
  while (!mqtt.connected() && (millis() - start) < timeoutMs) {
    const bool ok = mqtt.connect(
      clientId.c_str(),
      MQTT_USER, MQTT_PASS,
      presenceTopic.c_str(), 1, true,
      willPayload
    );

    if (ok) {
      Serial.println("MQTT connected!");

      // Subscribe to this device's commands
      mqtt.subscribe(subTopic.c_str(), 1);
      Serial.print("Subscribed: ");
      Serial.println(subTopic);

      // Birth ONLINE payload
      String timestamp = String(millis()); // placeholder for now
      String onlinePayload =
        "{\"device_id\":\"" + deviceId +
        "\",\"timestamp_utc\":\"" + timestamp +
        "\",\"status\":\"" + String(BIRTH_STATUS) + "\"}";

      mqtt.publish(presenceTopic.c_str(), onlinePayload.c_str(), true);

      Serial.print("Presence published: ");
      Serial.println(onlinePayload);

      return true;
    }

    Serial.print("MQTT failed, state=");
    Serial.println(mqtt.state());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  return mqtt.connected();
}

// ===================== RTOS Task =====================
static void mqttTask(void* pv) {
  Serial.println("RTOS MQTT task started.");

  static uint32_t seq = 1;
  static uint32_t lastTs = 0;
  static uint32_t lastPubMs = 0;

  for (;;) {
    // If WiFi drops, disconnect MQTT cleanly
    if (WiFi.status() != WL_CONNECTED && mqtt.connected()) {
      mqtt.disconnect();
    }

    if (!ensureWiFi()) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    if (!ensureMQTT()) {
      vTaskDelay(pdMS_TO_TICKS(1500));
      continue;
    }

    mqtt.loop();

    // ---- Peek latest sensor snapshot and publish when new (rate-limited) ----
    SensorData_t s;
    ModeCtx_t m;

    if (sensorsModelPeek(s, 0)) {

      // only react when Sensors task wrote a new snapshot
      if (s.ts_ms != lastTs) {
        lastTs = s.ts_ms;

        // rate limit
        uint32_t now = millis();
        if (now - lastPubMs >= MIN_PUB_MS) {
          lastPubMs = now;

          // actuators (default if mode ctx not available)
          bool fanEnabled = false;
          uint8_t heaterPwm = 0;

          if (modeCtxPeek(m, 0)) {
            fanEnabled = m.fanOn;
            heaterPwm  = m.heaterPwm;
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
            s.lux,
            s.temperature,
            s.humidity,
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

  presenceTopic = String(PRESENCE_TOPIC);
  pubTopic      = String(TOPIC_GREENHOUSE_STATE);
  subTopic      = String(TOPIC_SERVER_COMMANDS) + deviceId + "/#";

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(1024);

  Serial.println("===== MQTT CONFIG =====");
  Serial.print("DeviceId: ");       Serial.println(deviceId);
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