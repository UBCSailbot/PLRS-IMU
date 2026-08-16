/**
 * FreeRTOS task for fusing sensor measurements.
 */

#pragma once

#ifdef ARDUINO
#include "ekf_filter.h"
#include "telemetry.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>

namespace fusion_task {

struct TaskParams {
  QueueHandle_t imu_queue;
  QueueHandle_t gnss_queue;
  QueueHandle_t heading_mailbox;
  // Latest mag offset + variance for the persist task (1-slot overwrite).
  QueueHandle_t offset_mailbox;
  fusion::TinyEkfFilter::Config filter_config;
  plrs::TelemetrySink &telemetry;
};

void task(void *params);

} // namespace fusion_task
#endif
