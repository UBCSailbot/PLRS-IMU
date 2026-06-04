/**
 * Sensor fusion types and filter concept.
 *
 * No Arduino or FreeRTOS dependency. Host-compilable.
 */

#pragma once

#include <chrono>
#include <cmath>
#include <concepts>
#include <expected>
#include <numbers>

namespace fusion {

using Ms = std::chrono::milliseconds;

constexpr float GRAVITY_MS2 = 9.81f;
constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>;
constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

/**
 * Three-component vector.
 */
struct Vec3 {
  float x;
  float y;
  float z;
};

/**
 * Raw quaternion in {w, x, y, z} order (Xsens / Eigen convention).
 *
 * Plain four floats with no invariant; used at wire-level parsing and
 * as a math primitive. Rotation-bearing storage should use
 * `UnitQuaternion` instead, which defends the unit-norm invariant.
 */
struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

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
    return UnitQuaternion{Quaternion{1.0f, 0.0f, 0.0f, 0.0f}};
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
  static std::expected<UnitQuaternion, const char *> from_raw(Quaternion q) {
    const float norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    const float norm = std::sqrt(norm_sq);
    if (!(norm >= 1.0f - NORM_TOLERANCE && norm <= 1.0f + NORM_TOLERANCE)) {
      return std::unexpected("Quaternion norm not close to 1");
    }
    return UnitQuaternion{
        Quaternion{q.w / norm, q.x / norm, q.y / norm, q.z / norm}};
  }

  /**
   * @brief Hamilton product of two unit quaternions. Does not
   * renormalize the result.
   */
  static constexpr UnitQuaternion multiply(UnitQuaternion a, UnitQuaternion b) {
    const Quaternion p = a._q;
    const Quaternion q = b._q;
    return UnitQuaternion{Quaternion{
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
    return UnitQuaternion{Quaternion{_q.w, -_q.x, -_q.y, -_q.z}};
  }

  constexpr Quaternion components() const { return _q; }

private:
  explicit constexpr UnitQuaternion(Quaternion q) : _q(q) {}

  Quaternion _q;
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
  Vec3 angular_velocity_rad_s;
  Vec3 accel_ms2;
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
