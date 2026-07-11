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
 * A float telemetry field with its wire precision (decimal digits). The
 * precisions here mirror the Annotated types in sim/plrs_sim/live.py.
 */
struct Real {
  float v;
  uint8_t prec;
};

static void print_field(Real f) { Serial.print(f.v, f.prec); }
template <typename T> static void print_field(T v) { Serial.print(v); }

/**
 * @brief Print one comma-separated telemetry line: tag, then each field.
 */
template <typename... Fields>
static void print_line(char tag, Fields... fields) {
  if (!Serial) {
    return;
  }
  Serial.print(tag);
  ((Serial.print(','), print_field(fields)), ...);
  Serial.println();
}

/**
 * @brief Emit the fused estimate as an `F` telemetry line.
 *
 * `F,ts_ms,heading,roll,pitch,hdg_sigma,roll_sigma,pitch_sigma,bias,
 * bias_sigma,mag_offset,offset_sigma,gate_rejects,mag_gate_rejects`
 * (deg, deg/s). The trailing debug fields expose the internal states behind
 * the heading drift signature (docs/internal/heading_drift.md); the parser
 * treats them as one optional format-version tail.
 *
 * @param out  Fused estimate to print.
 * @param dbg  Internal state snapshot from the same filter tick.
 */
static void print_fusion(const fusion::FusionOutput &out,
                         const fusion::TinyEkfFilter::Debug &dbg) {
  print_line('F',
             out.timestamp.count(),
             Real {out.heading_deg, 3},
             Real {out.roll_deg, 3},
             Real {out.pitch_deg, 3},
             Real {std::sqrt(out.heading_variance_deg2), 3},
             Real {std::sqrt(out.roll_variance_deg2), 3},
             Real {std::sqrt(out.pitch_variance_deg2), 3},
             Real {dbg.gyro_bias_dps, 4},
             Real {std::sqrt(dbg.gyro_bias_variance_deg2_s2), 4},
             Real {dbg.mag_offset_deg, 3},
             Real {std::sqrt(dbg.mag_offset_variance_deg2), 3},
             dbg.gate_rejects,
             dbg.mag_gate_rejects);
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
  const plrs::Quaternion q = imu.orientation.components();
  const plrs::Vec3 &g = imu.angular_velocity_rad_s;
  const plrs::Vec3 &a = imu.accel_ms2;
  print_line('I',
             imu.timestamp.count(),
             Real {q.w, 5},
             Real {q.x, 5},
             Real {q.y, 5},
             Real {q.z, 5},
             Real {g.x, 5},
             Real {g.y, 5},
             Real {g.z, 5},
             Real {a.x, 4},
             Real {a.y, 4},
             Real {a.z, 4});
}

/**
 * @brief Emit the raw MEMS sensor triad as an `M` telemetry line.
 *
 * `M,ts_ms,ax,ay,az,gx,gy,gz,mx,my,mz` (accel m/s^2, gyro rad/s, magnetic
 * field a.u.). Accel and gyro duplicate the `I` frame so this line stands
 * alone as the bare MEMS output; the magnetometer appears only here.
 *
 * @param imu  Raw IMU sample as received from the IMU task.
 */
static void print_mems(const fusion::ImuSample &imu) {
  const plrs::Vec3 &a = imu.accel_ms2;
  const plrs::Vec3 &g = imu.angular_velocity_rad_s;
  const plrs::Vec3 &m = imu.magnetic_field_au;
  print_line('M',
             imu.timestamp.count(),
             Real {a.x, 4},
             Real {a.y, 4},
             Real {a.z, 4},
             Real {g.x, 5},
             Real {g.y, 5},
             Real {g.z, 5},
             Real {m.x, 5},
             Real {m.y, 5},
             Real {m.z, 5});
}

/**
 * @brief Emit a raw GNSS attitude sample as a `G` telemetry line.
 *
 * `G,ts_ms,heading,hdg_sigma,valid,mode,error` (deg; mode/error are the raw
 * AttEuler codes).
 *
 * @param gnss  Raw GNSS sample as received from the GNSS task.
 */
static void print_gnss(const fusion::GnssSample &gnss) {
  print_line('G',
             gnss.timestamp.count(),
             Real {gnss.heading_deg, 3},
             Real {std::sqrt(gnss.heading_variance_deg2), 3},
             gnss.valid ? 1 : 0,
             gnss.mode,
             gnss.error);
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
        print_fusion(out, filter.debug());
        print_imu(imu);
        print_mems(imu);
        next_print += pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
      }
    }
  }
}

} // namespace fusion_task
#endif
