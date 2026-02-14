// radio.cpp

#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>   // NAN / isnan()

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
static const char* PRESENCE_TOPIC          = "greenhouse/physical/state/connection";
static const char* TOPIC_GREENHOUSE_STATE  = "greenhouse/physical/state/environment";

// Subscribe topic for commands
static const char* SUB_COMMANDS_TOPIC      = "greenhouse/physical/setpoint/cmd/#";

// Timeouts / pacing
static const uint32_t WIFI_TIMEOUT_MS = 20000;
static const uint32_t TASK_DELAY_MS   = 50;

// Telemetry rate limit
static const uint32_t MIN_PUB_MS = 1000;

// Reconnect backoff
static const uint32_t MQTT_RECONNECT_BACKOFF_MS = 2000;

// ===================== State =====================
static String deviceId;
static String clientId;        // stable per boot
static String presenceTopic;
static String pubTopic;
static String subTopic;

static WiFiClient netClient;
static PubSubClient mqtt(netClient);

static TaskHandle_t hMqttTask = nullptr;

// Presence only once per boot
static bool birthSent = false;

// Track connection transitions
static bool wasMqttConnected = false;

// ===================== MQTT Callback =====================
static void onMessage(char* topic, byte* payload, unsigned int length) {
  // Convert payload to String
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);

  // Parse JSON
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  // Check measurement type
  const char* meas = doc["measurement"] | "";
  if (strcmp(meas, "greenhouse_setpoint_data_cmd") != 0) {
    Serial.println("Ignored: wrong measurement type");
    return;
  }

  // ---- TAGS (optional) ----
  JsonObject tags = doc["tags"];
  const char* rxDeviceId = "";
  const char* rxSourceId = "";
  const char* accessFlag = "";

  if (!tags.isNull()) {
    rxDeviceId = tags["device_id"] | "";
    rxSourceId = tags["source_id"] | "";
    accessFlag = tags["hierarchy_access_flag"] | "";
  }

  // ---- FIELDS (required) ----
  JsonObject fields = doc["fields"];
  if (fields.isNull()) {
    Serial.println("Ignored: missing fields object");
    return;
  }

  // accessCode priority:
  // 1) fields["accessCode"] if provided
  // 2) else fallback to hierarchy_access_flag mapping
  int accessCode;
  if (fields.containsKey("accessCode")) {
    accessCode = fields["accessCode"] | 0;   // 0=LOCAL, non-zero=SERVER
  } else {
    // Keep your older behavior if server doesn't send accessCode explicitly
    accessCode = (strcmp(accessFlag, "operator") == 0) ? 0 : 1;
  }

  // -------- Extract heater values --------
  float heaterSetpoint   = fields["heater_setpoint_temp_value_celsius"] | NAN;
  float heaterRange      = fields["heater_plusminus_range_from_temp_setpoint_celsius"] | NAN;
  float heaterHysteresis = fields["heater_hysteresis_safeband_temp_value_celsius"] | NAN;
  int   heaterPersist    = fields["heater_persistence_time_s"] | -1;

  // -------- Extract fan values --------
  float fanSetpoint   = fields["fan_setpoint_rh_value_percent"] | NAN;
  float fanRange      = fields["fan_plusminus_range_from_rh_setpoint_percent"] | NAN;
  float fanHysteresis = fields["fan_hysteresis_safeband_rh_value_percent"] | NAN;
  int   fanPersist    = fields["fan_persistence_time_s"] | -1;

  // Validate required setpoints
  if (isnan(heaterSetpoint) || isnan(fanSetpoint)) {
    Serial.println("Ignored: missing heater or fan setpoints");
    return;
  }

  // -------- Print all received parameters --------
  Serial.println("---- Received Setpoints ----");
  Serial.print("device_id: "); Serial.println(rxDeviceId);
  Serial.print("source_id: "); Serial.println(rxSourceId);
  Serial.print("hierarchy_access_flag: "); Serial.print(accessFlag);
  Serial.print("  => accessCode="); Serial.println(accessCode);

  Serial.print("Heater setpoint: ");    Serial.print(heaterSetpoint);   Serial.println(" °C");
  Serial.print("Heater ± range: ");     Serial.print(heaterRange);      Serial.println(" °C");
  Serial.print("Heater hysteresis: ");  Serial.print(heaterHysteresis); Serial.println(" °C");
  Serial.print("setpoint_light_level_lux "); Serial.print(heaterPersist);    Serial.println(" s");

  Serial.print("Fan setpoint: ");       Serial.print(fanSetpoint);      Serial.println(" %RH");
  Serial.print("Fan ± range: ");        Serial.print(fanRange);         Serial.println(" %RH");
  Serial.print("Fan hysteresis: ");     Serial.print(fanHysteresis);    Serial.println(" %RH");
  Serial.print("Fan persistence: ");    Serial.print(fanPersist);       Serial.println(" s");
  Serial.println("----------------------------");

  // -------- Send snapshot to config queue --------
  ConfigCmd_t cmd = {};
  cmd.type = CFG_NONE;

  cmd.accessCode = (uint8_t)accessCode;

  // If server controls, default to AUTO.
  // If local controls, mode here doesn't matter much (LogicTask will ignore server in LOCAL)
  cmd.mode = (cmd.accessCode == 0) ? MODE_MANUAL : MODE_AUTO;

  // Setpoints
  cmd.tempTarget_C = heaterSetpoint;
  cmd.humTarget_RH = fanSetpoint;

  // Tuning
  cmd.heaterRange_C   = isnan(heaterRange)      ? 0.0f : heaterRange;
  cmd.heaterHyst_C    = isnan(heaterHysteresis) ? 0.0f : heaterHysteresis;
  cmd.luxTarget = (heaterPersist < 0)     ? 0    : (uint16_t)heaterPersist;

  cmd.fanRange_RH   = isnan(fanRange)      ? 0.0f : fanRange;
  cmd.fanHyst_RH    = isnan(fanHysteresis) ? 0.0f : fanHysteresis;
  cmd.fanPersist_s  = (fanPersist < 0)     ? 0    : (uint16_t)fanPersist;

  cmd.ts_ms = millis();
  static uint32_t seq = 1;
  cmd.seq = seq++;

  bool ok = configSend(cmd, 0);
  Serial.print("configSend -> ");
  Serial.println(ok ? "OK" : "FAIL");
}

// ===================== Connectivity =====================
static bool ensureWiFi(uint32_t timeoutMs = WIFI_TIMEOUT_MS) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

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

static void publishBirthOnce() {
  if (birthSent) return;

  String ts = String(millis()); // placeholder (not real UTC yet)
  String onlinePayload =
    "{\"device_id\":\"" + deviceId +
    "\",\"timestamp_utc\":\"" + ts +
    "\",\"status\":\"" + String(BIRTH_STATUS) + "\"}";

  mqtt.publish(presenceTopic.c_str(), onlinePayload.c_str(), true);

  Serial.print("Presence published (boot): ");
  Serial.println(onlinePayload);

  birthSent = true;
}

static bool ensureMQTT() {
  if (mqtt.connected()) return true;

  // Backoff reconnect attempts
  static uint32_t lastAttemptMs = 0;
  uint32_t now = millis();
  if (now - lastAttemptMs < MQTT_RECONNECT_BACKOFF_MS) return false;
  lastAttemptMs = now;

  // Log disconnect reason once
  if (wasMqttConnected) {
    Serial.print("MQTT LOST (state=");
    Serial.print(mqtt.state());
    Serial.print(") WiFi=");
    Serial.print((int)WiFi.status());
    Serial.print(" RSSI=");
    Serial.println(WiFi.RSSI());
    wasMqttConnected = false;
  }

  // LWT OFFLINE payload
  char willPayload[96];
  snprintf(willPayload, sizeof(willPayload),
           "{\"device_id\":\"%s\",\"status\":\"%s\"}",
           deviceId.c_str(), DEATH_STATUS);

  Serial.print("MQTT connecting as ");
  Serial.println(clientId);

  // One connect attempt
  bool ok = mqtt.connect(
    clientId.c_str(),
    MQTT_USER, MQTT_PASS,
    presenceTopic.c_str(), 1, true,
    willPayload
  );

  if (!ok) {
    Serial.print("MQTT connect failed, state=");
    Serial.println(mqtt.state());
    return false;
  }

  wasMqttConnected = true;
  Serial.println("MQTT connected!");

  // Subscribe to commands
  bool subOk = mqtt.subscribe(subTopic.c_str(), 1);
  Serial.print("Subscribed: ");
  Serial.print(subTopic);
  Serial.println(subOk ? " ✓" : " ✗");

  publishBirthOnce();
  return true;
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

  // Stable per boot
  clientId = deviceId + "-" + String((uint32_t)esp_random(), HEX);

  presenceTopic = String(PRESENCE_TOPIC);
  pubTopic      = String(TOPIC_GREENHOUSE_STATE);
  subTopic      = String(SUB_COMMANDS_TOPIC);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(1024);

  mqtt.setKeepAlive(120);
  mqtt.setSocketTimeout(10);

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
