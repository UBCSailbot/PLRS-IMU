/**
 * FreeRTOS task that emits F/I/M/G telemetry lines from the latest fusion
 * snapshot. Keeps serial I/O off the fusion/estimation path.
 */

#pragma once

#ifdef ARDUINO
#include "ekf_filter.h"
#include "fusion.h"
#include "telemetry.h"
#include <FreeRTOS.h>
#include <cstdint>
#include <queue.h>

namespace telemetry_task {

/**
 * Latest values published by fusion_task via xQueueOverwrite. One slot is
 * enough: telemetry always wants the freshest estimate, not a backlog.
 */
struct Snapshot {
  fusion::FusionOutput out {};
  fusion::TinyEkfFilter::Debug dbg {};
  fusion::ImuSample imu {};
  fusion::GnssSample gnss {};
  bool has_imu = false;
  bool has_gnss = false;
  // Monotonic publish counter so telemetry can detect a fresh overwrite when
  // TELEMETRY_INTERVAL_MS is 0 (full-rate / PLRS_RAW_LOG).
  uint32_t seq = 0;
};

struct TaskParams {
  // Length-1 latest-value mailbox of Snapshot (xQueueOverwrite).
  QueueHandle_t snapshot_mailbox;
  plrs::TelemetrySink &telemetry;
};

void task(void *params);

} // namespace telemetry_task
#endif
