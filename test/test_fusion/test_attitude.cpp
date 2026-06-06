/**
 * Host tests for the attitude math.
 */

#include "attitude.h"
#include "fusion.h"
#include <cmath>
#include <unity.h>

using namespace fusion;

namespace {

constexpr float TOLERANCE = 1e-5f;

UnitQuaternion axis_angle(float ax, float ay, float az, float angle_rad) {
  const float half = angle_rad * 0.5f;
  const float s = std::sin(half);
  return UnitQuaternion::from_raw(
             plrs::Quaternion{std::cos(half), ax * s, ay * s, az * s})
      .value();
}

} // namespace

void test_unit_quaternion_identity_components() {
  plrs::Quaternion c = UnitQuaternion::identity().components();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, c.w);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.z);
}

void test_unit_quaternion_from_raw_accepts_unit() {
  auto q = UnitQuaternion::from_raw(plrs::Quaternion{1.0f, 0.0f, 0.0f, 0.0f});
  TEST_ASSERT_TRUE(q.has_value());
}

void test_unit_quaternion_from_raw_normalizes_near_unit() {
  auto q = UnitQuaternion::from_raw(plrs::Quaternion{1.005f, 0.0f, 0.0f, 0.0f});
  TEST_ASSERT_TRUE(q.has_value());
  plrs::Quaternion c = q->components();
  const float norm = std::sqrt(c.w * c.w + c.x * c.x + c.y * c.y + c.z * c.z);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, norm);
}

void test_unit_quaternion_from_raw_rejects_zero() {
  auto q = UnitQuaternion::from_raw(plrs::Quaternion{0.0f, 0.0f, 0.0f, 0.0f});
  TEST_ASSERT_FALSE(q.has_value());
}

void test_unit_quaternion_from_raw_rejects_far_from_unit() {
  auto q = UnitQuaternion::from_raw(plrs::Quaternion{2.0f, 0.0f, 0.0f, 0.0f});
  TEST_ASSERT_FALSE(q.has_value());
}

void test_unit_quaternion_from_raw_rejects_nan() {
  const float nan = std::nan("");
  auto q = UnitQuaternion::from_raw(plrs::Quaternion{nan, 0.0f, 0.0f, 0.0f});
  TEST_ASSERT_FALSE(q.has_value());
}

void test_unit_quaternion_multiply_with_identity_is_identity_element() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 1.234f);
  UnitQuaternion id = UnitQuaternion::identity();
  plrs::Quaternion a = UnitQuaternion::multiply(q, id).components();
  plrs::Quaternion b = UnitQuaternion::multiply(id, q).components();
  plrs::Quaternion c = q.components();
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.w, a.w);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.x, a.x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.y, a.y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.z, a.z);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.w, b.w);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.x, b.x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.y, b.y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, c.z, b.z);
}

void test_unit_quaternion_multiply_by_conjugate_is_identity() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 1.234f);
  plrs::Quaternion c = UnitQuaternion::multiply(q, q.conjugate()).components();
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, c.w);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, c.x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, c.y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, c.z);
}

void test_unit_quaternion_double_conjugate_is_self() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 1.234f);
  plrs::Quaternion c = q.conjugate().conjugate().components();
  plrs::Quaternion o = q.components();
  TEST_ASSERT_EQUAL_FLOAT(o.w, c.w);
  TEST_ASSERT_EQUAL_FLOAT(o.x, c.x);
  TEST_ASSERT_EQUAL_FLOAT(o.y, c.y);
  TEST_ASSERT_EQUAL_FLOAT(o.z, c.z);
}

void test_rotate_identity_passes_through() {
  plrs::Vec3 v{1.0f, 2.0f, 3.0f};
  plrs::Vec3 r = rotate(UnitQuaternion::identity(), v);
  TEST_ASSERT_EQUAL_FLOAT(v.x, r.x);
  TEST_ASSERT_EQUAL_FLOAT(v.y, r.y);
  TEST_ASSERT_EQUAL_FLOAT(v.z, r.z);
}

void test_rotate_yaw_90_maps_east_to_north() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 90.0f * DEG_TO_RAD);
  plrs::Vec3 r = rotate(q, plrs::Vec3{1.0f, 0.0f, 0.0f});
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, r.z);
}

void test_rotate_yaw_180_negates_xy() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 180.0f * DEG_TO_RAD);
  plrs::Vec3 r = rotate(q, plrs::Vec3{1.0f, 1.0f, 5.0f});
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -1.0f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -1.0f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 5.0f, r.z);
}

void test_euler_identity_is_zero() {
  EulerZyx e = quaternion_to_euler_zyx(UnitQuaternion::identity());
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.pitch_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.yaw_deg);
}

void test_euler_pure_yaw_recovers_yaw_angle() {
  UnitQuaternion q = axis_angle(0.0f, 0.0f, 1.0f, 30.0f * DEG_TO_RAD);
  EulerZyx e = quaternion_to_euler_zyx(q);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.pitch_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 30.0f, e.yaw_deg);
}

void test_euler_pure_pitch_recovers_pitch_angle() {
  UnitQuaternion q = axis_angle(0.0f, 1.0f, 0.0f, 30.0f * DEG_TO_RAD);
  EulerZyx e = quaternion_to_euler_zyx(q);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 30.0f, e.pitch_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.yaw_deg);
}

void test_euler_pure_roll_recovers_roll_angle() {
  UnitQuaternion q = axis_angle(1.0f, 0.0f, 0.0f, 30.0f * DEG_TO_RAD);
  EulerZyx e = quaternion_to_euler_zyx(q);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 30.0f, e.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.pitch_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, e.yaw_deg);
}

void test_euler_near_singular_does_not_nan() {
  UnitQuaternion q = axis_angle(0.0f, 1.0f, 0.0f, 89.99f * DEG_TO_RAD);
  EulerZyx e = quaternion_to_euler_zyx(q);
  TEST_ASSERT_FALSE(std::isnan(e.roll_deg));
  TEST_ASSERT_FALSE(std::isnan(e.pitch_deg));
  TEST_ASSERT_FALSE(std::isnan(e.yaw_deg));
}

void test_world_angular_velocity_identity_passes_through() {
  plrs::Vec3 omega_body{0.1f, -0.2f, 0.5f};
  plrs::Vec3 omega_world =
      world_angular_velocity(UnitQuaternion::identity(), omega_body);
  TEST_ASSERT_EQUAL_FLOAT(omega_body.x, omega_world.x);
  TEST_ASSERT_EQUAL_FLOAT(omega_body.y, omega_world.y);
  TEST_ASSERT_EQUAL_FLOAT(omega_body.z, omega_world.z);
}

void test_world_yaw_rate_heeled_boat_projects_by_cos_heel() {
  const float heel_rad = 30.0f * DEG_TO_RAD;
  UnitQuaternion orientation = axis_angle(1.0f, 0.0f, 0.0f, heel_rad);
  plrs::Vec3 omega_body{0.0f, 0.0f, 1.0f};
  float yaw_rate = world_yaw_rate(orientation, omega_body);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, std::cos(heel_rad), yaw_rate);
}

void test_world_yaw_rate_matches_world_angular_velocity_z() {
  UnitQuaternion orientation = axis_angle(0.0f, 1.0f, 0.0f, 0.3f);
  plrs::Vec3 omega_body{0.1f, 0.2f, 0.3f};
  plrs::Vec3 omega_world = world_angular_velocity(orientation, omega_body);
  float yaw_rate = world_yaw_rate(orientation, omega_body);
  TEST_ASSERT_EQUAL_FLOAT(omega_world.z, yaw_rate);
}
