/**
 * SH-2 sensor reports carried on the BNO085's SHTP input-report channel.
 *
 * A channel-3 batch is a Base-Timestamp report (0xFB) followed by one or more
 * sensor reports. Each sensor report is a 4-byte header (id, seq, status,
 * delay) then a run of little-endian int16 values in a sensor-specific
 * Q-format. All fields little endian.
 */

#pragma once

#include "common.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace sh2 {

using plrs::ByteSpan;

/**
 * Report / sensor IDs (SH-2 Reference Manual). The same value names the sensor
 * in a Set-Feature request and tags its report in a batch.
 */
enum class Report : uint8_t {
  Accelerometer = 0x01,
  GyroCalibrated = 0x02,
  MagCalibrated = 0x03,
  RotationVector = 0x05,
  BaseTimestamp = 0xfb,
  TimestampRebase = 0xfa,
};

// Fixed-point exponents: value = raw / 2^Q.
constexpr int ACCEL_Q = 8;  // m/s^2
constexpr int GYRO_Q = 9;   // rad/s
constexpr int MAG_Q = 4;    // uT
constexpr int QUAT_Q = 14;  // unit quaternion component

constexpr std::size_t REPORT_HEADER = 4; // id, seq, status, delay
constexpr std::size_t TIMESTAMP_LEN = 5; // id + int32 delta
constexpr std::size_t VEC3_DATA = 3 * sizeof(int16_t);
constexpr std::size_t QUAT_DATA = 4 * sizeof(int16_t); // i, j, k, real

// Set-Feature command (host -> control channel).
constexpr uint8_t SET_FEATURE_REQUEST = 0xfd;
constexpr std::size_t SET_FEATURE_LEN = 17;

constexpr int16_t read_s16_little_endian(ByteSpan b) {
  return static_cast<int16_t>(plrs::read_u16_little_endian(b));
}

/**
 * @brief Convert a fixed-point sample to float: raw / 2^qpoint.
 */
constexpr float q_to_float(int16_t raw, int qpoint) {
  return static_cast<float>(raw) / static_cast<float>(1 << qpoint);
}

/**
 * @brief Byte length of a report given its leading id, including its header.
 *
 * @return 0 for an unknown id, which the batch walker treats as "cannot
 *   advance" and stops (report lengths are fixed, so an unknown id means the
 *   rest of the batch can no longer be located).
 */
constexpr std::size_t report_len(uint8_t id) {
  switch (static_cast<Report>(id)) {
  case Report::BaseTimestamp:
  case Report::TimestampRebase:
    return TIMESTAMP_LEN;
  case Report::Accelerometer:
  case Report::GyroCalibrated:
  case Report::MagCalibrated:
    return REPORT_HEADER + VEC3_DATA;
  case Report::RotationVector:
    return REPORT_HEADER + QUAT_DATA + sizeof(int16_t); // + accuracy
  }
  return 0;
}

/**
 * @brief Find one sensor report's data bytes (past its 4-byte header).
 *
 * Walks the batch, skipping the leading timestamp report and any other reports,
 * until `wanted` is found.
 *
 * @param payload  A channel-3 batch payload.
 * @param wanted   Sensor report id to locate.
 *
 * @return The report's data span (borrowing `payload`), or nullopt if absent
 *   or the batch is truncated / holds an unknown id before it.
 */
constexpr std::optional<ByteSpan> find_report(ByteSpan payload, Report wanted) {
  std::size_t i = 0;
  while (i < payload.size()) {
    const uint8_t id = payload[i];
    const std::size_t len = report_len(id);
    if (len == 0 || i + len > payload.size()) {
      break;
    }
    if (id == static_cast<uint8_t>(wanted) && len > REPORT_HEADER) {
      return payload.subspan(i + REPORT_HEADER, len - REPORT_HEADER);
    }
    i += len;
  }
  return std::nullopt;
}

/**
 * @brief Read a 3-axis report (accel / gyro / mag) as a Vec3 in SI units.
 *
 * @param payload  Channel-3 batch payload.
 * @param id       One of Accelerometer, GyroCalibrated, MagCalibrated.
 * @param qpoint   The report's Q exponent (ACCEL_Q / GYRO_Q / MAG_Q).
 */
constexpr std::optional<plrs::Vec3>
read_vec3(ByteSpan payload, Report id, int qpoint) {
  auto data = find_report(payload, id);
  if (!data || data->size() < VEC3_DATA) {
    return std::nullopt;
  }
  auto axis = [&](std::size_t idx) {
    return q_to_float(read_s16_little_endian(data->subspan(idx * 2, 2)), qpoint);
  };
  return plrs::Vec3 {axis(0), axis(1), axis(2)};
}

constexpr std::optional<plrs::Vec3> read_accel(ByteSpan payload) {
  return read_vec3(payload, Report::Accelerometer, ACCEL_Q);
}

constexpr std::optional<plrs::Vec3> read_gyro(ByteSpan payload) {
  return read_vec3(payload, Report::GyroCalibrated, GYRO_Q);
}

constexpr std::optional<plrs::Vec3> read_mag(ByteSpan payload) {
  return read_vec3(payload, Report::MagCalibrated, MAG_Q);
}

/**
 * @brief Read the Rotation Vector quaternion in {w, x, y, z} order.
 *
 * The BNO reports i, j, k, real (x, y, z, w); the trailing accuracy field is
 * ignored. Not renormalized here (the fusion layer validates norm).
 */
constexpr std::optional<plrs::Quaternion>
read_rotation_vector(ByteSpan payload) {
  auto data = find_report(payload, Report::RotationVector);
  if (!data || data->size() < QUAT_DATA) {
    return std::nullopt;
  }
  auto comp = [&](std::size_t idx) {
    return q_to_float(read_s16_little_endian(data->subspan(idx * 2, 2)), QUAT_Q);
  };
  return plrs::Quaternion {
      .w = comp(3),
      .x = comp(0),
      .y = comp(1),
      .z = comp(2),
  };
}

/**
 * @brief Build a Set-Feature request selecting one sensor at a report interval.
 *
 * The 17-byte SH-2 payload; frame it onto the control channel with
 * shtp::encode_packet. All optional fields (flags, sensitivity, batch interval,
 * sensor config) are left zero.
 *
 * @param sensor       Sensor report id to enable.
 * @param interval_us  Report interval in microseconds (e.g. 10000 for 100 Hz).
 */
constexpr std::array<uint8_t, SET_FEATURE_LEN>
build_set_feature(Report sensor, uint32_t interval_us) {
  std::array<uint8_t, SET_FEATURE_LEN> p {};
  p[0] = SET_FEATURE_REQUEST;
  p[1] = static_cast<uint8_t>(sensor);
  // p[2] feature flags, p[3..4] change sensitivity: left zero.
  p[5] = static_cast<uint8_t>(interval_us & 0xff);
  p[6] = static_cast<uint8_t>((interval_us >> 8) & 0xff);
  p[7] = static_cast<uint8_t>((interval_us >> 16) & 0xff);
  p[8] = static_cast<uint8_t>((interval_us >> 24) & 0xff);
  // p[9..12] batch interval, p[13..16] sensor-specific config: left zero.
  return p;
}

} // namespace sh2
