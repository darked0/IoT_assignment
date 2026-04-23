#include "NetworkModule.h"
#include "Globals.h"

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Handling commands from the GUI
  if (message.startsWith("P:")) {
    String valueStr = message.substring(2);
    anomaly_probability = valueStr.toFloat();

    Serial.print(">>> COMMAND RECEIVED FROM EDGE DASHBOARD = ");
    Serial.println(anomaly_probability);
  }
}

void reconnectNetwork() {
  // Wifi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connection to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(500));
      Serial.print(".");
    }
    Serial.println(" Connected!");
  }

  // MQTT
  if (!mqttClient.connected()) {
    Serial.print("Connection to MQTT Broker...");

    // Callback before connecting
    mqttClient.setCallback(mqttCallback);

    // Client ID
    String clientId = "ESP32-" + String(random(1000), DEC);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" Connected!");

      // Commands topic
      mqttClient.subscribe("iot/assignment/edge/commands");
      Serial.println("Subscribed to topic: iot/assignment/edge/commands");

    } else {
      Serial.print(" Failed, rc=");
      Serial.println(mqttClient.state());
      switch (mqttClient.state()) {
      case -4: Serial.println(" (MQTT_CONNECTION_TIMEOUT)"); break;
      case -3: Serial.println(" (MQTT_CONNECTION_LOST)"); break;
      case -2: Serial.println(" (MQTT_CONNECT_FAILED)"); break;
      case -1: Serial.println(" (MQTT_DISCONNECTED)"); break;
      case 1: Serial.println(" (MQTT_CONNECT_BAD_PROTOCOL)"); break;
      case 2: Serial.println(" (MQTT_CONNECT_BAD_CLIENT_ID)"); break;
      case 3: Serial.println(" (MQTT_CONNECT_UNAVAILABLE)"); break;
      case 4: Serial.println(" (MQTT_CONNECT_UNAUTHORIZED)"); break;
      }
    }
  }
}

uint32_t sendMQTT(float avg) {
  uint32_t start_net_time = micros();
  if (mqttClient.connected()) {
    char payload[10];
    dtostrf(avg, 6, 3, payload);
    bool published = mqttClient.publish(mqtt_topic, payload);
    Serial.print("MQTT Publish result: ");
    Serial.println(published ? "OK" : "FAILED");
    Serial.print("Topic: ");
    Serial.println(mqtt_topic);
    Serial.print("Payload: ");
    Serial.println(payload);
  } else {
    Serial.println("MQTT non connected, send failed!");
  }
  return micros() - start_net_time;
}

void sendLoRaWAN(float avg) {
  if (!is_lora_joined) {
    Serial.println("[LoRaWAN] TTN not joined, cannot send data");
    return;
  }

  Serial.print("\n[LoRaWAN] Transmission data to TTN: ");
  Serial.println(avg);

  // Converting float to byte array (Payload)
  int16_t payload_val = (int16_t)(avg * 100);
  uint8_t payload[2];
  payload[0] = highByte(payload_val);
  payload[1] = lowByte(payload_val);

  // Send packet to port 1 (uplink)
  int state = node.sendReceive(payload, 2, 1);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRaWAN] Packet sent successfully!");
  } else {
    Serial.print("[LoRaWAN] Send error. Code: ");
    Serial.println(state);
  }
}

void sendLogToGUI(String logMessage) {
  Serial.println(logMessage);
  if (mqttClient.connected()) {
    mqttClient.publish("iot/assignment/edge/logs", logMessage.c_str());
  }
}
