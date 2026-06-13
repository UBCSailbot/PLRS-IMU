/**
 * Attitude math: quaternion / Euler / world-frame projection.
 */

#pragma once

#include "fusion.h"
#include <algorithm>
#include <cmath>

namespace fusion {

/**
 * ZYX intrinsic Euler angles in degrees.
 */
struct EulerZyx {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
};

/**
 * @brief Rotate a vector by a unit quaternion.
 *
 * @param q  Unit quaternion encoding the rotation.
 * @param v  Vector to rotate.
 *
 * @return The rotated vector.
 */
inline plrs::Vec3 rotate(UnitQuaternion q, plrs::Vec3 v) {
  const plrs::Quaternion p = q.components();
  const float tx = 2.0f * (p.y * v.z - p.z * v.y);
  const float ty = 2.0f * (p.z * v.x - p.x * v.z);
  const float tz = 2.0f * (p.x * v.y - p.y * v.x);
  return plrs::Vec3 {
      .x = v.x + p.w * tx + (p.y * tz - p.z * ty),
      .y = v.y + p.w * ty + (p.z * tx - p.x * tz),
      .z = v.z + p.w * tz + (p.x * ty - p.y * tx),
  };
}

/**
 * @brief Express a body-frame angular velocity in the world frame.
 *
 * @param orientation  Unit quaternion rotating body-frame vectors into
 *   the world frame.
 * @param omega_body   Angular velocity in the body frame.
 *
 * @return Angular velocity in the world frame.
 */
inline plrs::Vec3 world_angular_velocity(UnitQuaternion orientation,
                                         plrs::Vec3 omega_body) {
  return rotate(orientation, omega_body);
}

/**
 * @brief World-frame Z component of `world_angular_velocity`.
 *
 * @param orientation  Unit quaternion rotating body-frame vectors into
 *   the world frame.
 * @param omega_body   Angular velocity in the body frame.
 *
 * @return Yaw rate in the world frame.
 */
inline float world_yaw_rate(UnitQuaternion orientation, plrs::Vec3 omega_body) {
  return world_angular_velocity(orientation, omega_body).z;
}

/**
 * ZYX intrinsic Euler-angle rates, radians per second.
 */
struct EulerRates {
  float roll_dot;
  float pitch_dot;
  float yaw_dot;
};

/**
 * @brief ZYX Euler-angle rates from a body-frame angular velocity.
 *
 * Inverts the Euler kinematic matrix E(roll, pitch). The yaw row is the
 * heel-aware heading rate; at zero pitch it equals world_yaw_rate. Singular
 * at pitch = +-90 deg (cos pitch -> 0), which trim never reaches.
 *
 * @param roll_rad    Roll angle (radians).
 * @param pitch_rad   Pitch angle (radians).
 * @param omega_body  Body-frame angular velocity (rad/s).
 *
 * @return Euler-angle rates (rad/s).
 */
inline EulerRates
euler_rates_zyx(float roll_rad, float pitch_rad, plrs::Vec3 omega_body) {
  const float sr = std::sin(roll_rad);
  const float cr = std::cos(roll_rad);
  const float sec_pitch = 1.0f / std::cos(pitch_rad);
  const float tan_pitch = std::tan(pitch_rad);
  const float a = omega_body.y * sr + omega_body.z * cr;
  return EulerRates {
      .roll_dot = omega_body.x + a * tan_pitch,
      .pitch_dot = omega_body.y * cr - omega_body.z * sr,
      .yaw_dot = a * sec_pitch,
  };
}

/**
 * Partials of the ZYX Euler rates with respect to roll and pitch, per
 * radian. Yaw drops out of the kinematic matrix, so only these two columns
 * are non-trivial; they populate the off-diagonal terms of the EKF Jacobian.
 */
struct EulerRatesJacobian {
  float droll_droll;
  float droll_dpitch;
  float dpitch_droll;
  float dpitch_dpitch;
  float dyaw_droll;
  float dyaw_dpitch;
};

/**
 * @brief Analytic Jacobian of euler_rates_zyx in roll and pitch.
 *
 * @param roll_rad    Roll angle (radians).
 * @param pitch_rad   Pitch angle (radians).
 * @param omega_body  Body-frame angular velocity (rad/s).
 *
 * @return Partial derivatives of the Euler rates (per radian).
 */
inline EulerRatesJacobian
euler_rates_jacobian(float roll_rad, float pitch_rad, plrs::Vec3 omega_body) {
  const float sr = std::sin(roll_rad);
  const float cr = std::cos(roll_rad);
  const float sec_pitch = 1.0f / std::cos(pitch_rad);
  const float tan_pitch = std::tan(pitch_rad);
  const float a = omega_body.y * sr + omega_body.z * cr;
  const float a_droll = omega_body.y * cr - omega_body.z * sr;
  return EulerRatesJacobian {
      .droll_droll = a_droll * tan_pitch,
      .droll_dpitch = a * sec_pitch * sec_pitch,
      .dpitch_droll = -a,
      .dpitch_dpitch = 0.0f,
      .dyaw_droll = a_droll * sec_pitch,
      .dyaw_dpitch = a * sec_pitch * tan_pitch,
  };
}

/**
 * @brief Wrap an angle in degrees to the range (-180, 180].
 */
inline float wrap180(float deg) { return std::remainder(deg, 360.0f); }

/**
 * @brief Decompose a unit quaternion into ZYX intrinsic Euler angles.
 *
 * @param q  Unit quaternion to decompose.
 *
 * @return ZYX Euler angles in degrees.
 */
inline EulerZyx quaternion_to_euler_zyx(UnitQuaternion q) {
  const plrs::Quaternion p = q.components();
  const float sin_pitch =
      std::clamp(2.0f * (p.w * p.y - p.z * p.x), -1.0f, 1.0f);
  return EulerZyx {
      .roll_deg = std::atan2(2.0f * (p.w * p.x + p.y * p.z),
                             1.0f - 2.0f * (p.x * p.x + p.y * p.y)) *
                  RAD_TO_DEG,
      .pitch_deg = std::asin(sin_pitch) * RAD_TO_DEG,
      .yaw_deg = std::atan2(2.0f * (p.w * p.z + p.x * p.y),
                            1.0f - 2.0f * (p.y * p.y + p.z * p.z)) *
                 RAD_TO_DEG,
  };
}

} // namespace fusion
