#include "Globals.h"

// --- TTN CREDENTIALS (OTAA) ---
uint64_t appEui = 0x0000000000000000;
uint64_t devEui = 0x70B3D57ED0077150;
uint8_t appKey[] = {0x16, 0xDD, 0xAB, 0x26, 0xB9, 0xAC, 0x7E, 0x0E,
                    0x83, 0xC9, 0x9F, 0x8F, 0x31, 0x22, 0xC7, 0xAD};

// --- MQTT AND NETWORK SETUP ---
const char *ssid = "";
const char *password = "";
const char *mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_topic = "iot/assignment/edge/logs";

// Hardware instances
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, 1000.0);

// Global State Variables
float current_sampling_freq = 1000.0;
TickType_t sampling_period_ticks = 0;
double vReal[SAMPLES];
double vImag[SAMPLES];
uint16_t sampleCounter = 0;

volatile float current_signal_value = 0.0;
volatile float detected_peak_freq = 0.0;
volatile float last_aggregated_avg = 0.0;
volatile bool is_lora_joined = false;
volatile float anomaly_probability = 0.02;

// Filter State Variables
float filter_window[FILTER_WINDOW_SIZE];
int filter_idx = 0;
bool buffer_filled = false;

// Metrics
volatile int true_positives = 0;
volatile int false_positives = 0;
volatile int total_anomalies = 0;
volatile int total_clean = 0;

// FreeRTOS Handles
QueueHandle_t sampleQueue;
TaskHandle_t SignalGeneratorTaskHandle;
TaskHandle_t DisplayTaskHandle;
TaskHandle_t AggregationTaskHandle;
