/*
  ESP32 + ThingsBoard Single-Channel Relay Control (via Dashboard Button / RPC)
  --------------------------------------------------------------------------
  - ThingsBoard dashboard "Switch control" or "Button" widget sends an RPC
    call (method: "setRelayState", params: true/false) to the device.
  - ESP32 subscribes to v1/devices/me/rpc/request/+, toggles the relay GPIO,
    and publishes the current state back as telemetry so the widget stays
    in sync even after a reboot or page refresh.

  Library required: PubSubClient (install via Arduino Library Manager)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>   // install via Library Manager if not present

// ---------- WiFi credentials ----------
const char* WIFI_SSID     = "OOMNI-EYE-2.4GHz";
const char* WIFI_PASSWORD = "Admin@2026";

// ---------- ThingsBoard settings ----------
const char* TB_SERVER    = "allcad-chennai.selfip.com";   // or your ThingsBoard host/IP
const int   TB_PORT      = 1883;
const char* TB_TOKEN     = "feQhNs3O5m16WyQE1zMB";

// ---------- Relay pin ----------
const int RELAY_PIN = 15;          // change to whichever GPIO drives your relay
const bool RELAY_ACTIVE_LOW = true; // most relay modules trigger LOW = ON

WiFiClient   espClient;
PubSubClient client(espClient);

bool relayState = false;

void setRelay(bool state) {
  relayState = state;
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? !state : state);
  Serial.printf("Relay set to: %s\n", state ? "ON" : "OFF");

  // Publish current state as telemetry so the dashboard widget syncs
  StaticJsonDocument<64> doc;
  doc["relayState"] = relayState;
  char buffer[64];
  serializeJson(doc, buffer);
  client.publish("v1/devices/me/telemetry", buffer);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message on topic: ");
  Serial.println(topic);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  String methodName = doc["method"] | "";

  if (methodName == "setRelayState") {
    bool desiredState = doc["params"];
    setRelay(desiredState);

    // Reply back to the RPC request so the widget shows success
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    StaticJsonDocument<64> respDoc;
    respDoc["relayState"] = relayState;
    char respBuffer[64];
    serializeJson(respDoc, respBuffer);
    client.publish(responseTopic.c_str(), respBuffer);
  }
  else if (methodName == "getRelayState") {
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    StaticJsonDocument<64> respDoc;
    respDoc["relayState"] = relayState;
    char respBuffer[64];
    serializeJson(respDoc, respBuffer);
    client.publish(responseTopic.c_str(), respBuffer);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    // ThingsBoard MQTT auth: username = device access token, password = blank
    if (client.connect("ESP32Client", TB_TOKEN, NULL)) {
      Serial.println("connected");
      client.subscribe("v1/devices/me/rpc/request/+");
      // Publish initial state on connect
      setRelay(relayState);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 3s");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false); // start with relay OFF

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  client.setServer(TB_SERVER, TB_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
