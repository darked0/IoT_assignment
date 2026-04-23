#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>

float generateGaussianNoise(float mu, float sigma);
float injectAnomaly(float probability);

float applyZScoreFilter(float new_sample, bool is_actual_anomaly);
float applyHampelFilter(float new_sample, bool is_actual_anomaly);

void sortArray(float *arr, int n);
float getMedian(float *arr, int n);

#endif // FILTERS_H
