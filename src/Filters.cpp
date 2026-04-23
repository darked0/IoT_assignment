#include "Filters.h"
#include "Globals.h"

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

float applyZScoreFilter(float new_sample, bool is_actual_anomaly) {
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

  float sum = 0;
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++)
    sum += filter_window[i];
  float mean = sum / FILTER_WINDOW_SIZE;

  float variance = 0;
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    variance += pow(filter_window[i] - mean, 2);
  }
  float std_dev = sqrt(variance / FILTER_WINDOW_SIZE);

  float z_score = (std_dev > 0.001f) ? abs(new_sample - mean) / std_dev : 0;
  float filtered_sample = new_sample;
  bool flagged_as_anomaly = false;

  if (z_score > 3.0f) { 
    flagged_as_anomaly = true;
    filtered_sample = mean; 
  }

  if (flagged_as_anomaly && is_actual_anomaly)
    true_positives++;
  if (flagged_as_anomaly && !is_actual_anomaly)
    false_positives++;

  filter_window[filter_idx] = filtered_sample;
  filter_idx = (filter_idx + 1) % FILTER_WINDOW_SIZE;

  return filtered_sample;
}

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

  float median = getMedian(filter_window, FILTER_WINDOW_SIZE);

  float deviations[FILTER_WINDOW_SIZE];
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    deviations[i] = abs(filter_window[i] - median);
  }

  float mad = getMedian(deviations, FILTER_WINDOW_SIZE);
  float threshold = 3.0 * 1.4826 * mad;
  float filtered_sample = new_sample;
  bool flagged_as_anomaly = false;

  if (mad > 0.001f && abs(new_sample - median) > threshold) {
    flagged_as_anomaly = true;
    filtered_sample = median;
  }

  if (flagged_as_anomaly && is_actual_anomaly)
    true_positives++;
  if (flagged_as_anomaly && !is_actual_anomaly)
    false_positives++;

  filter_window[filter_idx] = filtered_sample;
  filter_idx = (filter_idx + 1) % FILTER_WINDOW_SIZE;

  return filtered_sample;
}
