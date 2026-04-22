#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>

// --- PIN LORA (SX1262 su Heltec V3) ---
#define LORA_CS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10

// LoRaWAN node (Frequency EU868)
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);

// --- TTN CREDENTIALS (OTAA) ---
uint64_t appEui = 0x0000000000000000; // aka JoinEUI
uint64_t devEui = 0x70B3D57ED0077150;
uint8_t appKey[] = {0x16, 0xDD, 0xAB, 0x26, 0xB9, 0xAC, 0x7E, 0x0E,
                    0x83, 0xC9, 0x9F, 0x8F, 0x31, 0x22, 0xC7, 0xAD};

// --- DISPLAY SETUP AND PIN HELTEC V3 ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT 36

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// --- MQTT AND NETWORK SETUP ---
const char *ssid = "*********";
const char *password = "b7dva23vdmeuh78";
const char *mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_topic = "iot/assignment/edge/logs";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- SAMPLING PARAMS AND FFT ---
float current_sampling_freq;
TickType_t sampling_period_ticks;
const uint16_t samples = 64;
double vReal[samples];
double vImag[samples];
uint16_t sampleCounter = 0;
ArduinoFFT<double> FFT =
    ArduinoFFT<double>(vReal, vImag, samples,
                       1000.0); // placeholder freq, will be updated dynamically

// --- GLOBAL VARIABLES ---
volatile float current_signal_value = 0.0;
volatile float detected_peak_freq = 0.0;
volatile float last_aggregated_avg = 0.0;
volatile bool is_lora_joined = false;
QueueHandle_t sampleQueue;
TaskHandle_t SignalGeneratorTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t AggregationTaskHandle;

// --- FILTER PARAMETERS ---
#define FILTER_WINDOW_SIZE                                                     \
  16 // Dimension of the moving window for both Z-Score and Hampel filters
float filter_window[FILTER_WINDOW_SIZE];
int filter_idx = 0;
bool buffer_filled = false;

// TPR e FPR
volatile int true_positives = 0;
volatile int false_positives = 0;
volatile int total_anomalies = 0;
volatile int total_clean = 0;

volatile float anomaly_probability = 0.02; // Probability editable from GUI

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
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

// --- SUPPORT FUNCTION: NETWORK RECONNECTION ---
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
      // Debug: stampa il codice di errore
      switch (mqttClient.state()) {
      case -4:
        Serial.println(" (MQTT_CONNECTION_TIMEOUT)");
        break;
      case -3:
        Serial.println(" (MQTT_CONNECTION_LOST)");
        break;
      case -2:
        Serial.println(" (MQTT_CONNECT_FAILED)");
        break;
      case -1:
        Serial.println(" (MQTT_DISCONNECTED)");
        break;
      case 1:
        Serial.println(" (MQTT_CONNECT_BAD_PROTOCOL)");
        break;
      case 2:
        Serial.println(" (MQTT_CONNECT_BAD_CLIENT_ID)");
        break;
      case 3:
        Serial.println(" (MQTT_CONNECT_UNAVAILABLE)");
        break;
      case 4:
        Serial.println(" (MQTT_CONNECT_UNAUTHORIZED)");
        break;
      }
    }
  }
}

// --- TRANSMISSION FUNCTIONS WITH LATENCY ---
// Modify the payload format to include the timestamp of the sample generation
// for better latency tracking
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

// --- MATH FUNCTIONS ---
float generateGaussianNoise(float mu, float sigma) {
  float u1 = max(0.00001f, (float)random(10000) / 10000.0f);
  float u2 = (float)random(10000) / 10000.0f;
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
  return z0 * sigma + mu;
}

float injectAnomaly(float probability) {
  float rand_val = (float)random(10000) / 10000.0f;
  if (rand_val < probability) {
    float magnitude = 5.0 + ((float)random(10000) / 10000.0f) * 10.0;
    return (random(2) == 0) ? magnitude : -magnitude;
  }
  return 0.0;
}

// --- Z-SCORE FILTER ---
float applyZScoreFilter(float new_sample, bool is_actual_anomaly) {
  // Update counters for "Ground Truth"
  if (is_actual_anomaly)
    total_anomalies++;
  else
    total_clean++;

  // Fill initial buffer
  if (!buffer_filled) {
    filter_window[filter_idx++] = new_sample;
    if (filter_idx >= FILTER_WINDOW_SIZE) {
      buffer_filled = true;
      filter_idx = 0;
    }
    return new_sample;
  }

  // Compute mean (Mu)
  float sum = 0;
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++)
    sum += filter_window[i];
  float mean = sum / FILTER_WINDOW_SIZE;

  // Compute standard deviation (Sigma)
  float variance = 0;
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    variance += pow(filter_window[i] - mean, 2);
  }
  float std_dev = sqrt(variance / FILTER_WINDOW_SIZE);

  // Compute Z-Score and flag anomalies
  // Z = |x - mu| / sigma
  float z_score = (std_dev > 0.001f) ? abs(new_sample - mean) / std_dev : 0;
  float filtered_sample = new_sample;
  bool flagged_as_anomaly = false;

  if (z_score > 3.0f) { // Threshold: 3 standard deviations
    flagged_as_anomaly = true;
    filtered_sample =
        mean; // Replace the anomaly with the mean to clean the signal
  }

  // Update the TPR and FPR metrics
  if (flagged_as_anomaly && is_actual_anomaly)
    true_positives++;
  if (flagged_as_anomaly && !is_actual_anomaly)
    false_positives++;

  // Update the sliding window with the filtered sample (prevents an anomaly
  // from contaminating future statistics)
  filter_window[filter_idx] = filtered_sample;
  filter_idx = (filter_idx + 1) % FILTER_WINDOW_SIZE;

  return filtered_sample;
}

// --- SUPPORT FUNCTION: MEDIAN ---
// Requires sorting of the array
void sortArray(float *arr, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

float getMedian(float *arr, int n) {
  float tempArr[FILTER_WINDOW_SIZE];
  for (int i = 0; i < n; i++)
    tempArr[i] = arr[i];
  sortArray(tempArr, n);

  if (n % 2 == 0) {
    return (tempArr[n / 2 - 1] + tempArr[n / 2]) / 2.0;
  } else {
    return tempArr[n / 2];
  }
}

// --- HAMPEL FILTER ---
float applyHampelFilter(float new_sample, bool is_actual_anomaly) {
  if (is_actual_anomaly)
    total_anomalies++;
  else
    total_clean++;

  if (!buffer_filled) {
    filter_window[filter_idx++] = new_sample;
    if (filter_idx >= FILTER_WINDOW_SIZE) {
      buffer_filled = true;
      filter_idx = 0;
    }
    return new_sample;
  }

  // Compute the Median of the window
  float median = getMedian(filter_window, FILTER_WINDOW_SIZE);

  // Compute the absolute deviations from the median
  float deviations[FILTER_WINDOW_SIZE];
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    deviations[i] = abs(filter_window[i] - median);
  }

  // Compute the MAD (Median Absolute Deviation)
  float mad = getMedian(deviations, FILTER_WINDOW_SIZE);

  // Compute the Hampel test (1.4826 is a scale factor to make it consistent
  // with the standard deviation)
  float threshold = 3.0 * 1.4826 * mad;
  float filtered_sample = new_sample;
  bool flagged_as_anomaly = false;

  // If the new sample deviates from the median more than the threshold, flag it
  // as an anomaly and replace it with the median
  if (mad > 0.001f && abs(new_sample - median) > threshold) {
    flagged_as_anomaly = true;
    filtered_sample = median;
  }

  // Update TPR and FPR
  if (flagged_as_anomaly && is_actual_anomaly)
    true_positives++;
  if (flagged_as_anomaly && !is_actual_anomaly)
    false_positives++;

  // Update the sliding window with the filtered sample
  filter_window[filter_idx] = filtered_sample;
  filter_idx = (filter_idx + 1) % FILTER_WINDOW_SIZE;

  return filtered_sample;
}

void sendLogToGUI(String logMessage) {
  Serial.println(logMessage);
  if (mqttClient.connected()) {
    mqttClient.publish("iot/assignment/edge/logs", logMessage.c_str());
  }
}

// --- TASK 1: SIGNAL GENERATOR AND FFT (Core 1) ---
void SignalGeneratorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  randomSeed(esp_random());

  while (true) {
    float t = (float)xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0;

    float base_signal =
        2.0 * sin(2.0 * PI * 3.0 * t) + 4.0 * sin(2.0 * PI * 5.0 * t);
    float noise = generateGaussianNoise(0.0, 0.2);
    float anomaly = injectAnomaly(anomaly_probability);
    float dirty_signal = base_signal + noise + anomaly;

    // Appy Z-Score filter (o Hampel filter)
    bool is_actual_anomaly = (anomaly != 0.0);
    float clean_signal = applyZScoreFilter(dirty_signal, is_actual_anomaly);

    current_signal_value = clean_signal; // Clean value on display

    // Send the clean signal to the Aggregation Task via queue for further
    // processing and FFT
    xQueueSend(sampleQueue, &clean_signal, 0);
    vReal[sampleCounter] = clean_signal;
    vImag[sampleCounter] = 0.0;
    sampleCounter++;

    if (sampleCounter == samples) {
      FFT = ArduinoFFT<double>(vReal, vImag, samples, current_sampling_freq);
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();
      detected_peak_freq = FFT.majorPeak();

      if (detected_peak_freq > 1.0) {
        float new_freq = ceil(detected_peak_freq) * 2.0;
        if (new_freq < 10.0)
          new_freq = 10.0;
        if (new_freq > 100.0)
          new_freq = 100.0;
        if (current_sampling_freq != new_freq) {
          current_sampling_freq = new_freq;
          sampling_period_ticks = pdMS_TO_TICKS(1000.0 / current_sampling_freq);
        }
      }
      sampleCounter = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, sampling_period_ticks);
  }
}

// --- TASK 3: AGGREGATION AND NETWORKING (Core 0) ---
void AggregationTask(void *pvParameters) {
  mqttClient.setServer(mqtt_server, mqtt_port);

  Serial.println("[AggregationTask] Initialization completed");

  float received_signal;
  float sum = 0.0;
  int count = 0;
  TickType_t window_start_time = xTaskGetTickCount();
  const TickType_t WINDOW_DURATION_TICKS = pdMS_TO_TICKS(5000);

  while (true) {
    reconnectNetwork();

    // Debug: verifiy connection status
    static int debug_counter = 0;
    if (debug_counter++ % 50 == 0) {
      Serial.print("[Aggregation] WiFi: ");
      Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
      Serial.print(" | MQTT: ");
      Serial.println(mqttClient.connected() ? "CONNECTED" : "NOT CONNECTED");
    }

    mqttClient.loop();

    if (xQueueReceive(sampleQueue, &received_signal, pdMS_TO_TICKS(100)) ==
        pdPASS) {
      sum += received_signal;
      count++;

      if ((xTaskGetTickCount() - window_start_time) >= WINDOW_DURATION_TICKS) {
        // Take timestamp
        uint32_t start_window_exec = micros();

        if (count > 0) {
          float average = sum / count;
          last_aggregated_avg = average;

          uint32_t end_calc_time = micros();
          uint32_t local_exec_time = end_calc_time - start_window_exec;

          // Transmission with latency measurement
          uint32_t network_latency = sendMQTT(average);
          sendLoRaWAN(average);
          uint32_t end_to_end_latency = local_exec_time + network_latency;

          // --- METRICS CALCULATION ---
          // Data Volume
          // Original (no edge): 100Hz * 5 sec = 500 samplings * 4 bytes (float)
          // = 2000 bytes
          int data_volume_original = 500 * sizeof(float);
          // System (edge computing): Send only our data (es. 6 bytes)
          int data_volume_transmitted = String(average).length();

          // Energy Saving Estimation
          // Savings in terms of CPU wake-ups: if we had sent all 500 samples to
          // the cloud, the CPU would have woken up 500 times to transmit each
          // sample. With edge computing, we wake up only once every 5 seconds
          // to send the aggregated result.
          float cpu_wakeups_saved_pct = ((500.0 - count) / 500.0) * 100.0;

          float tpr = (total_anomalies > 0)
                          ? ((float)true_positives / total_anomalies) * 100.0f
                          : 0.0f;
          float fpr = (total_clean > 0)
                          ? ((float)false_positives / total_clean) * 100.0f
                          : 0.0f;

          // --- REPORTING ---
          Serial.println(
              "\n========= REPORT PERFORMANCE WINDOW (5s) =========");
          Serial.print("Elaborated Samplings:\t\t");
          Serial.println(count);
          Serial.print("Execution Time (Local):\t");
          Serial.print(local_exec_time);
          Serial.println(" us");
          Serial.print("Network Latency (MQTT TX):\t");
          Serial.print(network_latency);
          Serial.println(" us");
          Serial.print("End-to-End Latency:\t");
          Serial.print(end_to_end_latency);
          Serial.println(" us");

          Serial.print("\n--- DATA VOLUME SAVINGS ---");
          Serial.print("\nData Volume (No Edge):\t\t");
          Serial.print(data_volume_original);
          Serial.println(" Bytes");
          Serial.print("Data Volume (Our TX):\t");
          Serial.print(data_volume_transmitted);
          Serial.println(" Bytes");

          Serial.print("\n--- ENERGY SAVING ---");
          Serial.print("\nCPU/Sensor Wake-ups Saved:\t");
          Serial.print(cpu_wakeups_saved_pct);
          Serial.println(" %");

          Serial.print("\n--- PERFORMANCE Z-SCORE FILTER ---");
          Serial.print("\nTPR (Anomalies Caught):\t");
          Serial.print(tpr);
          Serial.println(" %");
          Serial.print("FPR (False Alarms):\t\t");
          Serial.print(fpr);
          Serial.println(" %");
          Serial.println(
              "==================================================\n");

          // Log to GUI (One big packet)
          String report =
              "\n========= REPORT PERFORMANCE WINDOW (5s) =========\n";
          report += "Elaborated Samplings:\t\t" + String(count) + "\n";
          report +=
              "Execution Time (Local):\t" + String(local_exec_time) + " us\n";
          report += "Network Latency (MQTT TX):\t" + String(network_latency) +
                    " us\n";
          report +=
              "End-to-End Latency:\t" + String(end_to_end_latency) + " us\n";

          report += "\n--- DATA VOLUME SAVINGS ---\n";
          report += "Data Volume (No Edge):\t\t" +
                    String(data_volume_original) + " Bytes\n";
          report += "Data Volume (Our TX):\t" +
                    String(data_volume_transmitted) + " Bytes\n";

          report += "\n--- ENERGY SAVING ---\n";
          report += "CPU/Sensor Wake-ups Saved:\t" +
                    String(cpu_wakeups_saved_pct, 2) + " %\n";

          report += "\n--- PERFORMANCE Z-SCORE FILTER ---\n";
          report += "TPR (Anomalies Caught):\t" + String(tpr, 2) + " %\n";
          report += "FPR (False Alarms):\t\t" + String(fpr, 2) + " %\n";
          report += "==================================================\n";

          sendLogToGUI(report);
        }

        sum = 0.0;
        count = 0;
        window_start_time = xTaskGetTickCount();
      }
    }
  }
}

// --- TASK 2: DISPLAY HANDLER (Core 1) ---
// (Identico a prima)
void DisplayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t DISPLAY_PERIOD_TICKS = pdMS_TO_TICKS(500);

  while (true) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Signal : ");
    display.println(current_signal_value);
    display.setCursor(0, 16);
    display.print("FFT Peak: ");
    display.print(detected_peak_freq);
    display.println("Hz");
    display.setCursor(0, 32);
    display.print("Samp Freq: ");
    display.print(current_sampling_freq);
    display.println("Hz");
    display.setCursor(0, 48);
    display.print("Mean (5s): ");
    display.println(last_aggregated_avg);
    display.display();
    vTaskDelayUntil(&xLastWakeTime, DISPLAY_PERIOD_TICKS);
  }
}

int autoCalibrateADCFrequency() {
  Serial.println("--- BOOT: Calibrating Hardware ---");
  Serial.println("Measuring max ADC frequency (1 sec)...");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- BOOT SEQUENCE ---");
  display.println("");
  display.println("Calibrating ADC...");
  display.println("Please wait 1 sec");
  display.display();

  uint32_t start_time = micros();
  uint32_t count = 0;

  // Execute reads for exactly 1.000.000 microseconds (1 second)
  while (micros() - start_time < 1000000) {
    volatile int val =
        analogRead(4); // 'volatile' force CPU to read phisically the pin
    count++;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("--- BOOT SEQUENCE ---");
  display.println("");
  display.print("Max ADC: ");
  display.print(count);
  display.println(" Hz");
  display.println("");
  display.println("Set RTOS to: 1000 Hz");
  display.println("Starting OS...");
  display.display();

  return count;
}

void setup() {
  Serial.begin(115200);

  // Start display power (Vext LOW)
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);
  delay(50); // stabilization

  // Button reset for OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  // Initialize I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // Start display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Display Initialization Failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.display();

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
    // RadioLib automatically sets TCXO to 1.6V by default during begin(), which is usually enough for Heltec V3. 

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