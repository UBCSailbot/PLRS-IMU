#ifdef ARDUINO
#include "telemetry_task.h"
#include "hardware_config.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <cmath>
#include <task.h>

namespace telemetry_task {

// Match the former fusion_task throttle so the sim monitor still sees ~10 Hz
// lines. PLRS_RAW_LOG drops the delay for full-rate capture.
#ifdef PLRS_RAW_LOG
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 0;
#else
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;
#endif

/**
 * A float telemetry field with its wire precision (decimal digits). The
 * precisions here mirror the Annotated types in sim/plrs_sim/live.py.
 */
struct Real {
  float v;
  uint8_t prec;
};

static void print_field(Print &out, Real f) { out.print(f.v, f.prec); }
template <typename T> static void print_field(Print &out, T v) { out.print(v); }

/**
 * @brief Print one comma-separated telemetry line: tag, then each field.
 */
template <typename... Fields>
static void print_line(Print &out, char tag, Fields... fields) {
  out.print(tag);
  ((out.print(','), print_field(out, fields)), ...);
  out.println();
}

static void print_fusion(plrs::TelemetrySink &sink,
                         const fusion::FusionOutput &out,
                         const fusion::TinyEkfFilter::Debug &dbg) {
  auto line = sink.line();
  print_line(line,
             'F',
             out.timestamp.count(),
             Real {out.heading_deg, 3},
             Real {out.roll_deg, 3},
             Real {out.pitch_deg, 3},
             Real {std::sqrt(out.heading_variance_deg2), 3},
             Real {std::sqrt(out.roll_variance_deg2), 3},
             Real {std::sqrt(out.pitch_variance_deg2), 3},
             Real {dbg.gyro_bias_dps, 4},
             Real {std::sqrt(dbg.gyro_bias_variance_deg2_s2), 4},
             Real {dbg.gyro_bias_x_dps, 4},
             Real {std::sqrt(dbg.gyro_bias_x_variance_deg2_s2), 4},
             Real {dbg.gyro_bias_y_dps, 4},
             Real {std::sqrt(dbg.gyro_bias_y_variance_deg2_s2), 4},
             Real {dbg.mag_offset_deg, 3},
             Real {std::sqrt(dbg.mag_offset_variance_deg2), 3},
             dbg.gate_rejects,
             dbg.mag_gate_rejects,
             out.mag_accuracy);
}

static void print_imu(plrs::TelemetrySink &sink, const fusion::ImuSample &imu) {
  const plrs::Quaternion q = imu.orientation.components();
  const plrs::Vec3 &g = imu.angular_velocity_rad_s;
  const plrs::Vec3 &a = imu.accel_ms2;
  auto line = sink.line();
  print_line(line,
             'I',
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

static void print_mems(plrs::TelemetrySink &sink, const fusion::ImuSample &imu) {
  const plrs::Vec3 &a = imu.accel_ms2;
  const plrs::Vec3 &g = imu.angular_velocity_rad_s;
  const plrs::Vec3 &m = imu.magnetic_field_au;
  auto line = sink.line();
  print_line(line,
             'M',
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

static void print_gnss(plrs::TelemetrySink &sink,
                       const fusion::GnssSample &gnss) {
  auto line = sink.line();
  print_line(line,
             'G',
             gnss.timestamp.count(),
             Real {gnss.heading_deg, 3},
             Real {std::sqrt(gnss.heading_variance_deg2), 3},
             gnss.valid ? 1 : 0,
             gnss.mode,
             gnss.error);
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  TickType_t next = xTaskGetTickCount();
  uint32_t last_seq = 0;
  bool have_last_gnss = false;
  fusion::Ms last_gnss_ts {};

  while (true) {
    if constexpr (TELEMETRY_INTERVAL_MS == 0) {
      // Full-rate: wait briefly for a newer snapshot rather than spinning.
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      vTaskDelayUntil(&next, pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS));
    }

    Snapshot snap;
    if (xQueuePeek(p.snapshot_mailbox, &snap, 0) != pdTRUE) {
      continue;
    }
    if (!snap.has_imu || snap.seq == last_seq) {
      continue;
    }
    last_seq = snap.seq;

    print_fusion(p.telemetry, snap.out, snap.dbg);
    print_imu(p.telemetry, snap.imu);
    print_mems(p.telemetry, snap.imu);
    if (snap.has_gnss &&
        (!have_last_gnss || snap.gnss.timestamp != last_gnss_ts)) {
      print_gnss(p.telemetry, snap.gnss);
      last_gnss_ts = snap.gnss.timestamp;
      have_last_gnss = true;
    }
  }
}
} // namespace telemetry_task
#endif
