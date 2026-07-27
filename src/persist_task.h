/**
 * FreeRTOS task that persists the GNSS-learned mag offset to flash.
 *
 * The fusion task publishes the latest offset to a 1-slot mailbox; this
 * low-priority task applies the write policy and, when due, commits it to
 * flash. Keeping the (few-ms, core-idling) flash write on its own task keeps
 * the hitch off the 100 Hz fusion loop.
 */

#pragma once

#ifdef ARDUINO
#include "fusion.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>

namespace persist_task {

/**
 * One offset snapshot from the fusion task: the EKF mag offset, its variance
 * (the GNSS-validated gate), and the sample time the policy clocks off.
 */
struct OffsetSample {
  float offset_deg;
  float variance_deg2;
  fusion::Ms timestamp;
};

struct TaskParams {
  QueueHandle_t offset_mailbox;
  Print &telemetry;
};

void task(void *params);

} // namespace persist_task
#endif
