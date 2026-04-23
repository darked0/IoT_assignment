#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- PIN LORA (SX1262 su Heltec V3) ---
#define LORA_CS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10

// --- TTN CREDENTIALS (OTAA) ---
extern uint64_t appEui;
extern uint64_t devEui;
extern uint8_t appKey[];

// --- DISPLAY SETUP AND PIN HELTEC V3 ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT 36

// --- MQTT AND NETWORK SETUP ---
extern const char *ssid;
extern const char *password;
extern const char *mqtt_server;
extern const int mqtt_port;
extern const char *mqtt_topic;

// --- FILTER PARAMETERS ---
#define FILTER_WINDOW_SIZE 16 // Dimension of the moving window for both Z-Score and Hampel filters

// --- SAMPLING CONSTANTS ---
#define SAMPLES 64 // FFT window size

#endif // CONFIG_H
