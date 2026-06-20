/**
 * FreeRTOS task for fusing sensor measurements.
 */

#pragma once

#ifdef ARDUINO
#include "ekf_filter.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace fusion_task {

struct TaskParams {
  QueueHandle_t imu_queue;
  QueueHandle_t gnss_queue;
  QueueHandle_t heading_mailbox;
  fusion::TinyEkfFilter::Config filter_config;
};

void task(void *params);

} // namespace fusion_task
#endif
