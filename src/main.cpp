#include <Arduino.h>
#include "Config.h"
#include "Globals.h"
#include "DisplayUtils.h"
#include "Tasks.h"

void setup() {
  Serial.begin(115200);

  // Start display
  initDisplay();

  // MQTT buffer and queue
  mqttClient.setBufferSize(1024);
  sampleQueue = xQueueCreate(200, sizeof(float));

  // --- SETUP LORA E TTN (OTAA) ---
  Serial.println("\n[LoRaWAN] Starting Radio Module SX1262...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Init LoRa OTAA...");
  display.display();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.setRxBoostedGainMode(true); // Increases RX sensitivity
    // RadioLib automatically sets TCXO to 1.6V by default during begin(), which
    // is usually enough for Heltec V3.

    // LoRaWAN 1.0.x uses NwkKey as AppKey. RadioLib can accept the same array
    // for both.
    node.beginOTAA(appEui, devEui, appKey, appKey);
    Serial.println(
        "[LoRaWAN] Joining TTN via OTAA. This might take a few seconds...");

    // Sometimes the first Join Accept is missed due to window timing
    // or gateway latency. A retry loop is standard practice.
    int retry = 0;
    state = RADIOLIB_ERR_NETWORK_NOT_JOINED; // Reset state to force the loop
    while (retry < 5 && state != RADIOLIB_ERR_NONE) {
      Serial.print("Join attempt ");
      Serial.print(retry + 1);
      Serial.println("/5...");

      state = node.activateOTAA();

      if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed (code ");
        Serial.print(state);
        Serial.println("). Retrying in 10s...");
        delay(10000);
        retry++;
      }
    }

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("[LoRaWAN] OTAA JOINED! Ready to transmit.");
      display.println("LoRa Ready!");
      display.println("Mode: OTAA");
      display.display();
      is_lora_joined = true;
    } else {
      Serial.print("[LoRaWAN] Error activating OTAA: ");
      Serial.println(state);
      display.println("OTAA Error!");
      display.display();
    }
  } else {
    Serial.print("[LoRaWAN] Hardware error: ");
    Serial.println(state);
    display.println("Radio Error!");
    display.display();
  }

  delay(2000);

  // --- AUTO-CALIBRATION FREQUENCY ---
  int max_hardware_freq = autoCalibrateADCFrequency();

  Serial.print(">> Max Frequency (Hardware ADC): ");
  Serial.print(max_hardware_freq);
  Serial.println(" Hz");

  Serial.println(
      ">> Setting Initial Operating Frequency to RTOS Limit: 1000 Hz");

  current_sampling_freq = 1000.0; // 1 millisecond
  sampling_period_ticks = pdMS_TO_TICKS(1000.0 / current_sampling_freq);

  delay(2000);

  // Task creation (ESP32 has 2 Cores: Core 0 for Aggregation, Core 1 for Signal
  // Gen + Display)
  xTaskCreatePinnedToCore(SignalGeneratorTask, "SignalGenTask", 4096, NULL, 2,
                          &SignalGeneratorTaskHandle, 1);
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 1,
                          &DisplayTaskHandle, 1);
  xTaskCreatePinnedToCore(AggregationTask, "AggregationTask", 8192, NULL, 1,
                          &AggregationTaskHandle, 0);
}

// Arduino defualt loop() is not used since we are running FreeRTOS tasks, but
// it must be defined
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }