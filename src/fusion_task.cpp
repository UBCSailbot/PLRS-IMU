#ifdef ARDUINO
#include "fusion_task.h"
#include "ekf_filter.h"
#include "fusion.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

namespace fusion_task {

static constexpr uint32_t IMU_QUEUE_TIMEOUT_MS = 20;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

/**
 * @brief Print a FusionOutput as a single CSV line over USB serial.
 *
 * @param out  Fused estimate to print.
 */
static void print_output(const fusion::FusionOutput &out) {
  Serial.print(out.timestamp.count());
  Serial.print(',');
  Serial.print(out.heading_deg, 3);
  Serial.print(',');
  Serial.print(out.roll_deg, 3);
  Serial.print(',');
  Serial.println(out.pitch_deg, 3);
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  fusion::TinyEkfFilter filter {p.filter_config};

  TickType_t next_print = xTaskGetTickCount();

  while (true) {
    fusion::ImuSample imu;
    if (xQueueReceive(p.imu_queue,
                      &imu,
                      pdMS_TO_TICKS(IMU_QUEUE_TIMEOUT_MS)) == pdTRUE) {
      filter.predict(imu);

      fusion::GnssSample gnss;
      while (xQueueReceive(p.gnss_queue, &gnss, 0) == pdTRUE) {
        filter.update(gnss);
      }

      if (xTaskGetTickCount() >= next_print) {
        print_output(filter.output());
        next_print += pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
      }
    }
  }
}

} // namespace fusion_task
#endif
