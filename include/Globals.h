#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include "Config.h"

// Hardware instances
extern SX1262 radio;
extern LoRaWANNode node;
extern Adafruit_SSD1306 display;
extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern ArduinoFFT<double> FFT;

// Global State Variables
extern float current_sampling_freq;
extern TickType_t sampling_period_ticks;
extern double vReal[SAMPLES];
extern double vImag[SAMPLES];
extern uint16_t sampleCounter;

extern volatile float current_signal_value;
extern volatile float detected_peak_freq;
extern volatile float last_aggregated_avg;
extern volatile bool is_lora_joined;
extern volatile float anomaly_probability;

// Filter State Variables
extern float filter_window[FILTER_WINDOW_SIZE];
extern int filter_idx;
extern bool buffer_filled;

// Metrics
extern volatile int true_positives;
extern volatile int false_positives;
extern volatile int total_anomalies;
extern volatile int total_clean;

// FreeRTOS Handles
extern QueueHandle_t sampleQueue;
extern TaskHandle_t SignalGeneratorTaskHandle;
extern TaskHandle_t DisplayTaskHandle;
extern TaskHandle_t AggregationTaskHandle;

#endif // GLOBALS_H
