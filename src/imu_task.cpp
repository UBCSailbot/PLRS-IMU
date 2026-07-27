#ifdef ARDUINO
#include "imu_task.h"
#include "fusion.h"
#include "hardware_config.h"
#include "sh2_reports.h"
#include "shtp_protocol.h"
#include "stack_check.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <array>
#include <optional>
#include <pico/time.h>
#include <task.h>

namespace imu_task {

static_assert(
    plrs::fits_on_task_stack<std::array<uint8_t, bno08x::MAX_CARGO>>(
        IMU_TASK_STACK_SIZE),
    "IMU_TASK_STACK_SIZE too small for the SHTP cargo buffer");

static constexpr uint16_t IMU_RATE_HZ = 100;
static constexpr uint32_t REPORT_INTERVAL_US = 1'000'000 / IMU_RATE_HZ;
static constexpr uint32_t READY_TIMEOUT_MS = 1000;
static constexpr uint32_t RETRY_DELAY_MS = 500;
static constexpr uint32_t RESET_DRAIN_MS = 300;
static constexpr uint8_t SOFT_RESET_CMD = 0x01;

// Sensors mirrored into every ImuSample. Rotation Vector is the mag-referenced
// orientation (the MTi quaternion's analog); the triads back it up.
static constexpr std::array<sh2::Report, 4> ENABLED_SENSORS {{
    sh2::Report::RotationVector,
    sh2::Report::GyroCalibrated,
    sh2::Report::Accelerometer,
    sh2::Report::MagCalibrated,
}};

static std::chrono::milliseconds now() {
  return std::chrono::milliseconds(time_us_64() / 1000);
}

/**
 * @brief Frame a payload onto an SHTP channel and send it.
 *
 * @param transport  I2C transport to the BNO085.
 * @param seq  Outgoing sequence counter, incremented in place.
 * @param channel  Destination SHTP channel.
 * @param payload  Cargo payload bytes.
 */
static void send(bno08x::I2cTransport &transport,
                 uint8_t &seq,
                 shtp::Channel channel,
                 shtp::ByteSpan payload) {
  auto encoded = shtp::encode_packet(channel, seq++, payload);
  if (encoded) {
    transport.write_packet(encoded->view());
  }
}

/**
 * @brief Read and discard every pending cargo for @p duration_ms.
 *
 * Used after a reset to swallow the advertisement and reset-complete cargos so
 * the first sensor batch is not mistaken for them.
 */
static void drain(bno08x::I2cTransport &transport,
                  std::span<uint8_t> scratch,
                  uint32_t duration_ms) {
  auto deadline = now() + std::chrono::milliseconds(duration_ms);
  while (now() < deadline) {
    if (!transport.read_cargo(scratch)) {
      vTaskDelay(1);
    }
  }
}

/**
 * @brief Reset, enable the sensor set, and wait for the first sensor batch.
 *
 * Retries indefinitely: on the RP2040 a silent IMU means no fused heading, so
 * there is nothing to fall back to but trying again.
 *
 * @param transport  I2C transport to the BNO085.
 * @param scratch  Cargo buffer (MAX_CARGO).
 */
static void bring_up(bno08x::I2cTransport &transport,
                     std::span<uint8_t> scratch,
                     Print &out) {
  uint8_t seq = 0;
  while (true) {
    const std::array<uint8_t, 1> reset {SOFT_RESET_CMD};
    send(transport, seq, shtp::Channel::Executable, {reset.data(), 1});
    drain(transport, scratch, RESET_DRAIN_MS);

    for (const sh2::Report sensor : ENABLED_SENSORS) {
      const auto feature = sh2::build_set_feature(sensor, REPORT_INTERVAL_US);
      send(transport,
           seq,
           shtp::Channel::Control,
           {feature.data(), feature.size()});
    }

    auto deadline = now() + std::chrono::milliseconds(READY_TIMEOUT_MS);
    while (now() < deadline) {
      auto cargo = transport.read_cargo(scratch);
      if (!cargo) {
        vTaskDelay(1);
        continue;
      }
      auto packet = shtp::parse_packet(*cargo);
      if (packet && packet->channel ==
                        static_cast<uint8_t>(shtp::Channel::InputReports)) {
        out.println("# IMU: ready");
        return;
      }
    }
    out.println("# IMU: no reports, retrying");
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  std::array<uint8_t, bno08x::MAX_CARGO> scratch;

  p.transport.scan(p.telemetry); // bring-up diagnostic: what ACKs on the bus
  bring_up(p.transport, scratch, p.telemetry);

  // Each report may arrive in its own cargo, so hold the latest triads and emit
  // an ImuSample when a fresh Rotation Vector lands, paired with them.
  std::optional<plrs::Vec3> last_gyro;
  std::optional<plrs::Vec3> last_accel;
  std::optional<plrs::Vec3> last_mag;

  while (true) {
    auto cargo = p.transport.read_cargo(scratch);
    if (!cargo) {
      vTaskDelay(1);
      continue;
    }

    auto packet = shtp::parse_packet(*cargo);
    if (!packet ||
        packet->channel != static_cast<uint8_t>(shtp::Channel::InputReports)) {
      continue;
    }

    if (auto g = sh2::read_gyro(packet->payload)) {
      last_gyro = g;
    }
    if (auto a = sh2::read_accel(packet->payload)) {
      last_accel = a;
    }
    if (auto m = sh2::read_mag(packet->payload)) {
      last_mag = m;
    }

    auto quat = sh2::read_rotation_vector(packet->payload);
    if (!quat || !last_gyro || !last_accel) {
      continue;
    }

    auto orientation = fusion::UnitQuaternion::from_raw(*quat);
    if (!orientation) {
      continue;
    }

    fusion::ImuSample sample {
        .angular_velocity_rad_s = *last_gyro,
        .accel_ms2 = *last_accel,
        .magnetic_field_au = last_mag ? *last_mag : plrs::Vec3 {},
        .orientation = *orientation,
        .timestamp = now(),
    };

    xQueueSend(p.queue, &sample, 0);
  }
}

} // namespace imu_task
#endif
