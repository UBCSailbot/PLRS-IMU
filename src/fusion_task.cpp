#ifdef ARDUINO
#include "fusion_task.h"
#include "ekf_filter.h"
#include "fusion.h"
#include "hardware_config.h"
#include "persist_task.h"
#include "stack_check.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

namespace fusion_task {

static_assert(
    plrs::fits_on_task_stack<fusion::TinyEkfFilter>(FUSION_TASK_STACK_SIZE),
    "FUSION_TASK_STACK_SIZE too small for fusion::TinyEkfFilter");

static constexpr uint32_t IMU_QUEUE_TIMEOUT_MS = 20;

// Publish cadence for the telemetry mailbox. The default 10 Hz keeps the live
// monitor readable; a PLRS_RAW_LOG build drops it so every predict tick is
// published for full-rate capture (see docs/magnetometer.md).
#ifdef PLRS_RAW_LOG
static constexpr uint32_t TELEMETRY_PUBLISH_INTERVAL_MS = 0;
#else
static constexpr uint32_t TELEMETRY_PUBLISH_INTERVAL_MS = 100;
#endif

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  fusion::TinyEkfFilter filter {p.filter_config};

  TickType_t next_publish = xTaskGetTickCount();
  uint32_t telemetry_seq = 0;
  fusion::GnssSample last_gnss {};
  bool have_gnss = false;

  while (true) {
    fusion::ImuSample imu;
    if (xQueueReceive(p.imu_queue, &imu, pdMS_TO_TICKS(IMU_QUEUE_TIMEOUT_MS)) ==
        pdTRUE) {
      filter.predict(imu);

      fusion::GnssSample gnss;
      while (xQueueReceive(p.gnss_queue, &gnss, 0) == pdTRUE) {
        filter.update(gnss);
        last_gnss = gnss;
        have_gnss = true;
      }

      const fusion::FusionOutput out = filter.output();
      xQueueOverwrite(p.heading_mailbox, &out);

      const bool due =
          (TELEMETRY_PUBLISH_INTERVAL_MS == 0) ||
          (xTaskGetTickCount() >= next_publish);
      if (due) {
        const fusion::TinyEkfFilter::Debug dbg = filter.debug();
        // Hand the latest offset to the persist task; it decides whether to
        // write flash (GNSS-validated, changed, min interval). See
        // persist_task.
        const persist_task::OffsetSample offset_sample {
            .offset_deg = dbg.mag_offset_deg,
            .variance_deg2 = dbg.mag_offset_variance_deg2,
            .timestamp = out.timestamp,
        };
        xQueueOverwrite(p.offset_mailbox, &offset_sample);

        telemetry_task::Snapshot snap {
            .out = out,
            .dbg = dbg,
            .imu = imu,
            .gnss = last_gnss,
            .has_imu = true,
            .has_gnss = have_gnss,
            .seq = ++telemetry_seq,
        };
        xQueueOverwrite(p.telemetry_mailbox, &snap);

        if (TELEMETRY_PUBLISH_INTERVAL_MS != 0) {
          next_publish += pdMS_TO_TICKS(TELEMETRY_PUBLISH_INTERVAL_MS);
        }
      }
    }
  }
}

} // namespace fusion_task
#endif
