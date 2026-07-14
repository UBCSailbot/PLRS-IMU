/**
 * Sensor fusion types and filter concept.
 *
 * No Arduino or FreeRTOS dependency. Host-compilable.
 */

#pragma once

#include "common.h"
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <expected>
#include <numbers>

namespace fusion {

using Ms = std::chrono::milliseconds;

constexpr float GRAVITY_MS2 = 9.81f;
// Arduino.h defines these as macros; only define our constexpr versions on
// non-Arduino builds (e.g. native host tests).
#ifndef RAD_TO_DEG
constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>;
#endif
#ifndef DEG_TO_RAD
constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
#endif

/**
 * Quaternion known to be unit-norm by construction.
 *
 * Built only via `identity()`, `from_raw()`, `multiply()`, or
 * `conjugate()`. Attitude-consuming functions take this type so
 * passing an un-validated quaternion is a compile error.
 */
class UnitQuaternion {
public:
  /**
   * Tolerance on `|q|` for `from_raw` to accept an input as
   * approximately unit before normalizing.
   */
  static constexpr float NORM_TOLERANCE = 0.01f;

  static constexpr UnitQuaternion identity() {
    return UnitQuaternion {plrs::Quaternion {1.0f, 0.0f, 0.0f, 0.0f}};
  }

  /**
   * @brief Wrap already-unit components without validating or normalizing.
   *
   * The constexpr counterpart to `from_raw`, for components computed unit at
   * build time (e.g. the generated mount in tuning.h). The caller guarantees
   * `|q| == 1`; use `from_raw` for untrusted input.
   *
   * @param q  Unit quaternion components.
   */
  static constexpr UnitQuaternion from_unit_unchecked(plrs::Quaternion q) {
    return UnitQuaternion {q};
  }

  /**
   * @brief Validate that `q` is approximately unit and normalize it.
   *
   * @param q  Raw quaternion components.
   *
   * @return A unit quaternion, or an error if `|q|` is outside
   *   `[1 - NORM_TOLERANCE, 1 + NORM_TOLERANCE]`. NaN inputs are
   *   rejected.
   */
  static std::expected<UnitQuaternion, const char *>
  from_raw(plrs::Quaternion q) {
    const float norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    const float norm = std::sqrt(norm_sq);
    if (!(norm >= 1.0f - NORM_TOLERANCE && norm <= 1.0f + NORM_TOLERANCE)) {
      return std::unexpected("Quaternion norm not close to 1");
    }
    return UnitQuaternion {
        plrs::Quaternion {q.w / norm, q.x / norm, q.y / norm, q.z / norm}};
  }

  /**
   * @brief Hamilton product of two unit quaternions. Does not
   * renormalize the result.
   */
  static constexpr UnitQuaternion multiply(UnitQuaternion a, UnitQuaternion b) {
    const plrs::Quaternion p = a._q;
    const plrs::Quaternion q = b._q;
    return UnitQuaternion {plrs::Quaternion {
        p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
        p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
        p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
        p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w,
    }};
  }

  /**
   * @brief Conjugate of a unit quaternion.
   */
  constexpr UnitQuaternion conjugate() const {
    return UnitQuaternion {plrs::Quaternion {_q.w, -_q.x, -_q.y, -_q.z}};
  }

  constexpr plrs::Quaternion components() const { return _q; }

private:
  explicit constexpr UnitQuaternion(plrs::Quaternion q) : _q(q) {}

  plrs::Quaternion _q;
};

/**
 * Static rotation from the boat body frame into the IMU body frame.
 */
struct MountRotation {
  UnitQuaternion boat_to_imu = UnitQuaternion::identity();
};

/**
 * One IMU sample from the MTi-3, in SI units.
 */
struct ImuSample {
  plrs::Vec3 angular_velocity_rad_s;
  plrs::Vec3 accel_ms2;
  // Raw MTi magnetometer, arbitrary units (normalized ~1 after calibration).
  // Carried for telemetry only; the filter aids heading off the MTi quaternion,
  // not the raw mag. Zero when the packet carries no MagneticField field.
  plrs::Vec3 magnetic_field_au = {};
  UnitQuaternion orientation = UnitQuaternion::identity();
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
  // Raw AttEuler mode/error carried through for telemetry only; the filter
  // uses valid (derived from these). Lets a monitor tell float from
  // no-attitude from a flagged baseline without re-deriving the gate.
  uint16_t mode = 0;
  uint8_t error = 0;
};

/**
 * Fused estimate published after each predict step.
 */
struct FusionOutput {
  float heading_deg;
  float heading_variance_deg2; // P[0][0] from the filter covariance
  float roll_deg;
  float roll_variance_deg2;
  float pitch_deg;
  float pitch_variance_deg2;
  Ms timestamp;
  float yaw_rate_dps;
  // Pre-filter values straight off the MTi-3 onboard orientation, carried so
  // the rudder can steer on the sensor's own AHRS while our EKF is worked on.
  // raw_roll_deg is the measured boat-frame roll; raw_yaw_rate_dps is the
  // heel-aware heading rate from the same measured attitude, without gyro-bias
  // subtraction.
  float raw_roll_deg;
  float raw_yaw_rate_dps;
};

/**
 * @brief Whether a fused heading is trustworthy enough to steer on.
 *
 * Three ways it is not: a non-finite value (should never happen, but a hard
 * backstop against a NaN reaching the rudder), a variance past max_variance
 * (GNSS-unanchored and free-drifting), or a pitch past max_pitch of level,
 * where the ZYX heading kinematics are singular and "heading" is not a
 * well-defined compass bearing (see ekf_filter.h PITCH_KINEMATICS_LIMIT_DEG).
 * Thresholds are the caller's policy; the rudder link carries the result as
 * FusionOutput's heading_valid flag.
 */
inline bool heading_trustworthy(const FusionOutput &out,
                                float max_variance_deg2,
                                float max_pitch_deg) {
  return std::isfinite(out.heading_deg) &&
         out.heading_variance_deg2 <= max_variance_deg2 &&
         std::fabs(out.pitch_deg) <= max_pitch_deg;
}

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
