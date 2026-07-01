#ifdef ARDUINO
#include "fusion_task.h"
#include "ekf_filter.h"
#include "fusion.h"
#include "hardware_config.h"
#include "stack_check.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

namespace fusion_task {

static_assert(
    plrs::fits_on_task_stack<fusion::TinyEkfFilter>(FUSION_TASK_STACK_SIZE),
    "FUSION_TASK_STACK_SIZE too small for fusion::TinyEkfFilter");

static constexpr uint32_t IMU_QUEUE_TIMEOUT_MS = 20;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

/**
 * @brief Emit the fused estimate as an `F` telemetry line.
 *
 * `F,ts_ms,heading,roll,pitch,hdg_sigma,roll_sigma,pitch_sigma` (deg).
 *
 * @param out  Fused estimate to print.
 */
static void print_fusion(const fusion::FusionOutput &out) {
  if (!Serial) {
    return;
  }
  Serial.print("F,");
  Serial.print(out.timestamp.count());
  Serial.print(',');
  Serial.print(out.heading_deg, 3);
  Serial.print(',');
  Serial.print(out.roll_deg, 3);
  Serial.print(',');
  Serial.print(out.pitch_deg, 3);
  Serial.print(',');
  Serial.print(std::sqrt(out.heading_variance_deg2), 3);
  Serial.print(',');
  Serial.print(std::sqrt(out.roll_variance_deg2), 3);
  Serial.print(',');
  Serial.println(std::sqrt(out.pitch_variance_deg2), 3);
}

/**
 * @brief Emit a raw IMU sample as an `I` telemetry line.
 *
 * `I,ts_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az` (quaternion, gyro rad/s, accel
 * m/s^2).
 *
 * @param imu  Raw IMU sample as received from the IMU task.
 */
static void print_imu(const fusion::ImuSample &imu) {
  if (!Serial) {
    return;
  }
  const plrs::Quaternion q = imu.orientation.components();
  Serial.print("I,");
  Serial.print(imu.timestamp.count());
  Serial.print(',');
  Serial.print(q.w, 5);
  Serial.print(',');
  Serial.print(q.x, 5);
  Serial.print(',');
  Serial.print(q.y, 5);
  Serial.print(',');
  Serial.print(q.z, 5);
  Serial.print(',');
  Serial.print(imu.angular_velocity_rad_s.x, 5);
  Serial.print(',');
  Serial.print(imu.angular_velocity_rad_s.y, 5);
  Serial.print(',');
  Serial.print(imu.angular_velocity_rad_s.z, 5);
  Serial.print(',');
  Serial.print(imu.accel_ms2.x, 4);
  Serial.print(',');
  Serial.print(imu.accel_ms2.y, 4);
  Serial.print(',');
  Serial.println(imu.accel_ms2.z, 4);
}

/**
 * @brief Emit a raw GNSS attitude sample as a `G` telemetry line.
 *
 * `G,ts_ms,heading,hdg_sigma,valid` (deg).
 *
 * @param gnss  Raw GNSS sample as received from the GNSS task.
 */
static void print_gnss(const fusion::GnssSample &gnss) {
  if (!Serial) {
    return;
  }
  Serial.print("G,");
  Serial.print(gnss.timestamp.count());
  Serial.print(',');
  Serial.print(gnss.heading_deg, 3);
  Serial.print(',');
  Serial.print(std::sqrt(gnss.heading_variance_deg2), 3);
  Serial.print(',');
  Serial.println(gnss.valid ? 1 : 0);
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  fusion::TinyEkfFilter filter {p.filter_config};

  TickType_t next_print = xTaskGetTickCount();

  while (true) {
    fusion::ImuSample imu;
    if (xQueueReceive(p.imu_queue, &imu, pdMS_TO_TICKS(IMU_QUEUE_TIMEOUT_MS)) ==
        pdTRUE) {
      filter.predict(imu);

      fusion::GnssSample gnss;
      while (xQueueReceive(p.gnss_queue, &gnss, 0) == pdTRUE) {
        filter.update(gnss);
        print_gnss(gnss);
      }

      const fusion::FusionOutput out = filter.output();
      xQueueOverwrite(p.heading_mailbox, &out);

      if (xTaskGetTickCount() >= next_print) {
        print_fusion(out);
        print_imu(imu);
        next_print += pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
      }
    }
  }
}

} // namespace fusion_task
#endif
