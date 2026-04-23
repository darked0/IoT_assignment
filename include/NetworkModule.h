#ifndef NETWORK_MODULE_H
#define NETWORK_MODULE_H

#include <Arduino.h>

void mqttCallback(char *topic, byte *payload, unsigned int length);
void reconnectNetwork();
uint32_t sendMQTT(float avg);
void sendLoRaWAN(float avg);
void sendLogToGUI(String logMessage);

#endif // NETWORK_MODULE_H
