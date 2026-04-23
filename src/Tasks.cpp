#include "Tasks.h"
#include "Globals.h"
#include "Filters.h"
#include "NetworkModule.h"

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

    if (sampleCounter == SAMPLES) {
      FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, current_sampling_freq);
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
          int data_volume_original = count * sizeof(float);
          // System (edge computing): Send only our data (es. 6 bytes)
          int data_volume_transmitted = String(average).length();

          // Energy Saving Estimation
          // Savings in terms of CPU wake-ups: if we had sent all 'count'
          // samples to the cloud, the CPU would have woken up 'count' times to
          // transmit each sample. With edge computing, we wake up only once
          float cpu_wakeups_saved_pct = ((count - 1.0) / (float)count) * 100.0;

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
