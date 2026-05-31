/**
 * Sensor fusion types and filter concept.
 *
 * No Arduino or FreeRTOS dependency. Host-compilable.
 */

#pragma once

#include <chrono>
#include <concepts>

namespace fusion {

using Ms = std::chrono::milliseconds;

/**
 * One IMU sample from the MTi-3, in SI units.
 */
struct ImuSample {
  float rate_of_turn_x_rad_s;
  float rate_of_turn_y_rad_s;
  float rate_of_turn_z_rad_s;
  float accel_x_ms2;
  float accel_y_ms2;
  float accel_z_ms2;
  Ms timestamp;
};

/**
 * One GNSS attitude measurement from the mosaic-go-H.
 *
 * Check valid before using heading fields; invalid samples are passed to
 * update() and silently skipped by the filter.
 */
struct GnssSample {
  float heading_deg;
  float heading_variance_deg2;
  // TODO: add heading_dot_deg_s + heading_dot_variance_deg2_s2 for m=2 update
  Ms timestamp;
  bool valid;
};

/**
 * Fused heading estimate published after each predict step.
 */
struct FusionOutput {
  float heading_deg;
  float heading_variance_deg2; // P[0][0] from the filter covariance
  Ms timestamp;
};

/**
 * @brief Compile-time interface every filter implementation must satisfy.
 *
 * @tparam F Filter type. predict() is called at IMU rate (~100 Hz) and
 *   propagates state using gyro. update() is called when a GnssSample arrives
 *   and skips if !gnss.valid. output() returns the current heading estimate,
 *   valid after the first predict().
 */
template <typename F>
concept FusionFilter = requires(F &f, ImuSample imu, GnssSample gnss) {
  { f.predict(imu) } -> std::same_as<void>;
  { f.update(gnss) } -> std::same_as<void>;
  { f.output() } -> std::same_as<FusionOutput>;
};

} // namespace fusion
