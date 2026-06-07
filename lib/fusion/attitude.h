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
  return plrs::Vec3{
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
  return EulerZyx{
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
