#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>

void SignalGeneratorTask(void *pvParameters);
void AggregationTask(void *pvParameters);
void DisplayTask(void *pvParameters);

#endif // TASKS_H
