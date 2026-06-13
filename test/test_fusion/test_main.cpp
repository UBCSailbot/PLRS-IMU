/**
 * Host (native) unit tests for the fusion EKF filter.
 */

#include "ekf_filter.h"
#include "fusion.h"
#include <cfloat>
#include <cmath>
#include <unity.h>

using namespace fusion;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
//
// Concept check: TinyEkfFilter must satisfy FusionFilter. Catches signature
// drift (return types, parameter types) at compile time rather than runtime.
// ---------------------------------------------------------------------------
namespace {

static_assert(FusionFilter<TinyEkfFilter>);

constexpr TinyEkfFilter::Config kTestConfig {
    .q_heading_deg2 = 0.01f,
    .q_roll_deg2 = 0.01f,
    .q_pitch_deg2 = 0.01f,
    .q_bias_deg2_s2 = 0.0001f,
    .p0_heading_deg2 = 1000.0f,
    .p0_roll_deg2 = 1000.0f,
    .p0_pitch_deg2 = 1000.0f,
    .p0_bias_deg2_s2 = 1.0f,
    .mti_roll_variance_deg2 = 1.0f,
    .mti_pitch_variance_deg2 = 1.0f,
};

ImuSample make_imu(float rate_z_rad_s, Ms t) {
  return ImuSample {
      .angular_velocity_rad_s = plrs::Vec3 {0.0f, 0.0f, rate_z_rad_s},
      .accel_ms2 = plrs::Vec3 {0.0f, 0.0f, GRAVITY_MS2},
      .timestamp = t,
  };
}

ImuSample make_imu_with(plrs::Vec3 omega, UnitQuaternion orientation, Ms t) {
  return ImuSample {
      .angular_velocity_rad_s = omega,
      .accel_ms2 = plrs::Vec3 {0.0f, 0.0f, GRAVITY_MS2},
      .orientation = orientation,
      .timestamp = t,
  };
}

UnitQuaternion axis_angle(float ax, float ay, float az, float angle_rad) {
  const float half = angle_rad * 0.5f;
  const float s = std::sin(half);
  return UnitQuaternion::from_raw(
             plrs::Quaternion {std::cos(half), ax * s, ay * s, az * s})
      .value();
}

ImuSample make_heeled_imu(float rate_z_rad_s, float heel_rad, Ms t) {
  return ImuSample {
      .angular_velocity_rad_s = plrs::Vec3 {0.0f, 0.0f, rate_z_rad_s},
      .accel_ms2 = plrs::Vec3 {0.0f, 0.0f, GRAVITY_MS2},
      .orientation = axis_angle(1.0f, 0.0f, 0.0f, heel_rad),
      .timestamp = t,
  };
}

GnssSample
make_gnss(float heading_deg, float variance_deg2, Ms t, bool valid = true) {
  return GnssSample {
      .heading_deg = heading_deg,
      .heading_variance_deg2 = variance_deg2,
      .timestamp = t,
      .valid = valid,
  };
}

} // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Initial state
// ---------------------------------------------------------------------------

/** @brief Fresh filter: output reports FLT_MAX variance (uninitialized). */
void test_initial_state_uninitialized() {
  TinyEkfFilter f(kTestConfig);
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX, out.heading_variance_deg2);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.heading_deg);
}

// ---------------------------------------------------------------------------
// update()
// ---------------------------------------------------------------------------

/** @brief Invalid GnssSample is a no-op; filter stays uninitialized. */
void test_invalid_gnss_is_noop() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(45.0f, 1.0f, Ms {1000}, false));
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX, out.heading_variance_deg2);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.heading_deg);
}

/** @brief First valid GnssSample seeds heading and marks filter initialized.
 *
 * Variance equals p0_heading_deg2: seeding bypasses the EKF update math, so
 * P[IDX_HEADING][IDX_HEADING] is unchanged from its constructor-set value. */
void test_first_valid_gnss_seeds_heading() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(45.0f, 1.0f, Ms {1000}));
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(45.0f, out.heading_deg);
  TEST_ASSERT_EQUAL_FLOAT(1000.0f, out.heading_variance_deg2);
}

/** @brief A tight GNSS update pulls heading strongly toward the measurement. */
void test_update_pulls_toward_measurement() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));    // seed at 0
  f.predict(make_imu(0.0f, Ms {1000}));          // baseline
  f.predict(make_imu(0.0f, Ms {5000}));          // grow P over 4 s
  f.update(make_gnss(45.0f, 0.001f, Ms {5000})); // very tight R
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 45.0f, out.heading_deg);
}

/** @brief A measurement update shrinks the heading variance. */
void test_update_shrinks_variance() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000})); // baseline
  f.predict(make_imu(0.0f, Ms {2000})); // grow P by Q
  auto before = f.output().heading_variance_deg2;
  f.update(make_gnss(0.0f, 0.1f, Ms {2000}));
  auto after = f.output().heading_variance_deg2;
  TEST_ASSERT_TRUE(after < before);
}

// ---------------------------------------------------------------------------
// predict()
// ---------------------------------------------------------------------------

/** @brief First predict() sets the timestamp baseline; no state advance. */
void test_first_predict_baseline_only() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(45.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(1.0f, Ms {2000})); // 1 rad/s but first predict call
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(45.0f, out.heading_deg);
}

/** @brief Gyro_z integrates into heading: 0.5 rad/s for 1 s ~= 28.6479 deg. */
void test_predict_integrates_gyro() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000})); // baseline
  f.predict(make_imu(0.5f, Ms {2000})); // 0.5 rad/s * 1 s
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.6479f, out.heading_deg);
}

/** @brief A predict step grows the heading variance (process noise added). */
void test_predict_grows_variance() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000})); // baseline
  auto before = f.output().heading_variance_deg2;
  f.predict(make_imu(0.0f, Ms {2000}));
  auto after = f.output().heading_variance_deg2;
  TEST_ASSERT_TRUE(after > before);
}

// ---------------------------------------------------------------------------
// Attitude wiring
// ---------------------------------------------------------------------------

/**
 * @brief A heeled flat turn integrates heading at the world yaw rate.
 *
 * At 30 deg heel a pure world-frame yaw rate of 1 rad/s shows up in the body
 * gyro as (0, sin(heel), cos(heel)). The Euler yaw row recovers the full
 * 1 rad/s, decoupled from heel, while roll and pitch hold steady.
 */
void test_predict_heeled_flat_turn_tracks_world_yaw() {
  TinyEkfFilter f(kTestConfig);
  const float heel_rad = 30.0f * DEG_TO_RAD;
  const float yaw_rate = 1.0f; // rad/s, world frame
  const plrs::Vec3 omega {
      0.0f, yaw_rate * std::sin(heel_rad), yaw_rate * std::cos(heel_rad)};
  const UnitQuaternion heel = axis_angle(1.0f, 0.0f, 0.0f, heel_rad);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu_with(omega, heel, Ms {1000})); // baseline
  f.predict(make_imu_with(omega, heel, Ms {2000})); // 1 s of turn
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, yaw_rate * RAD_TO_DEG, out.heading_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, out.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, out.pitch_deg);
}

/** @brief First predict seeds output().roll_deg from the MTi quaternion. */
void test_output_exposes_roll_from_orientation() {
  TinyEkfFilter f(kTestConfig);
  const float heel_rad = 20.0f * DEG_TO_RAD;
  f.predict(make_heeled_imu(0.0f, heel_rad, Ms {1000}));
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out.roll_deg);
}

/** @brief boat_to_imu mount rotation flows into output().roll_deg. */
void test_output_applies_mount_rotation() {
  TinyEkfFilter::Config cfg = kTestConfig;
  cfg.mount.boat_to_imu = axis_angle(1.0f, 0.0f, 0.0f, 10.0f * DEG_TO_RAD);
  TinyEkfFilter f(cfg);
  f.predict(make_imu(0.0f, Ms {1000}));
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, out.roll_deg);
}

// ---------------------------------------------------------------------------
// Attitude states
// ---------------------------------------------------------------------------

/** @brief Fresh filter reports FLT_MAX roll/pitch variance (no measurement). */
void test_attitude_variance_uninitialized() {
  TinyEkfFilter f(kTestConfig);
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX, out.roll_variance_deg2);
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX, out.pitch_variance_deg2);
}

/** @brief After the seeding predict, roll/pitch variance is finite. */
void test_attitude_variance_finite_after_predict() {
  TinyEkfFilter f(kTestConfig);
  f.predict(make_heeled_imu(0.0f, 15.0f * DEG_TO_RAD, Ms {1000}));
  auto out = f.output();
  TEST_ASSERT_TRUE(out.roll_variance_deg2 < FLT_MAX);
  TEST_ASSERT_TRUE(out.pitch_variance_deg2 < FLT_MAX);
}

/** @brief Repeated MTi samples drive roll to the heel and shrink its variance.
 */
void test_mti_corrects_roll_and_shrinks_variance() {
  TinyEkfFilter f(kTestConfig);
  const float heel_rad = 25.0f * DEG_TO_RAD;
  f.predict(make_heeled_imu(0.0f, heel_rad, Ms {1000})); // seed roll = 25
  const float seeded_var = f.output().roll_variance_deg2;
  for (int i = 2; i <= 20; i++) {
    f.predict(make_heeled_imu(0.0f, heel_rad, Ms {1000 * i}));
  }
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 25.0f, out.roll_deg);
  TEST_ASSERT_TRUE(out.roll_variance_deg2 < seeded_var);
}

/** @brief A GNSS fix across the +-180 seam wraps the innovation, not the long
 * way around. Seeded at 179 deg, a tight fix at -179 lands heading near -179.
 */
void test_gnss_update_wraps_across_seam() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(179.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {2000}));
  f.update(make_gnss(-179.0f, 0.001f, Ms {2000}));
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, wrap180(out.heading_deg - (-179.0f)));
}

/** @brief Heading stays wrapped to (-180, 180] after integrating past 180. */
void test_predict_wraps_heading_past_180() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(170.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000}));
  f.predict(make_imu(0.5f, Ms {2000})); // +28.6 deg -> 198.6 -> wraps to -161.4
  auto out = f.output();
  TEST_ASSERT_TRUE(out.heading_deg <= 180.0f && out.heading_deg > -180.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -161.35f, out.heading_deg);
}

// ---------------------------------------------------------------------------
// Forward declarations for tests defined in test_attitude.cpp.
// ---------------------------------------------------------------------------

void test_unit_quaternion_identity_components();
void test_unit_quaternion_from_raw_accepts_unit();
void test_unit_quaternion_from_raw_normalizes_near_unit();
void test_unit_quaternion_from_raw_rejects_zero();
void test_unit_quaternion_from_raw_rejects_far_from_unit();
void test_unit_quaternion_from_raw_rejects_nan();
void test_unit_quaternion_multiply_with_identity_is_identity_element();
void test_unit_quaternion_multiply_by_conjugate_is_identity();
void test_unit_quaternion_double_conjugate_is_self();
void test_rotate_identity_passes_through();
void test_rotate_yaw_90_maps_east_to_north();
void test_rotate_yaw_180_negates_xy();
void test_euler_identity_is_zero();
void test_euler_pure_yaw_recovers_yaw_angle();
void test_euler_pure_pitch_recovers_pitch_angle();
void test_euler_pure_roll_recovers_roll_angle();
void test_euler_near_singular_does_not_nan();
void test_world_angular_velocity_identity_passes_through();
void test_world_yaw_rate_heeled_boat_projects_by_cos_heel();
void test_world_yaw_rate_matches_world_angular_velocity_z();
void test_euler_rates_level_pass_through_gyro();
void test_euler_rates_heeled_body_z_splits_into_yaw_and_pitch();
void test_euler_rates_yaw_matches_world_yaw_rate_at_zero_pitch();
void test_euler_rates_jacobian_matches_finite_difference();
void test_wrap180_wraps_into_range();

void test_bridge_passes_heading_and_variance_through();
void test_bridge_subtracts_baseline_offset();
void test_bridge_wraps_heading_into_plus_minus_180();
void test_bridge_dnu_heading_is_invalid();
void test_bridge_no_attitude_mode_is_invalid();
void test_bridge_dnu_covariance_uses_fallback();

// ---------------------------------------------------------------------------

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_initial_state_uninitialized);
  RUN_TEST(test_invalid_gnss_is_noop);
  RUN_TEST(test_first_valid_gnss_seeds_heading);
  RUN_TEST(test_update_pulls_toward_measurement);
  RUN_TEST(test_update_shrinks_variance);
  RUN_TEST(test_first_predict_baseline_only);
  RUN_TEST(test_predict_integrates_gyro);
  RUN_TEST(test_predict_grows_variance);
  RUN_TEST(test_predict_heeled_flat_turn_tracks_world_yaw);
  RUN_TEST(test_output_exposes_roll_from_orientation);
  RUN_TEST(test_output_applies_mount_rotation);
  RUN_TEST(test_attitude_variance_uninitialized);
  RUN_TEST(test_attitude_variance_finite_after_predict);
  RUN_TEST(test_mti_corrects_roll_and_shrinks_variance);
  RUN_TEST(test_gnss_update_wraps_across_seam);
  RUN_TEST(test_predict_wraps_heading_past_180);

  RUN_TEST(test_unit_quaternion_identity_components);
  RUN_TEST(test_unit_quaternion_from_raw_accepts_unit);
  RUN_TEST(test_unit_quaternion_from_raw_normalizes_near_unit);
  RUN_TEST(test_unit_quaternion_from_raw_rejects_zero);
  RUN_TEST(test_unit_quaternion_from_raw_rejects_far_from_unit);
  RUN_TEST(test_unit_quaternion_from_raw_rejects_nan);
  RUN_TEST(test_unit_quaternion_multiply_with_identity_is_identity_element);
  RUN_TEST(test_unit_quaternion_multiply_by_conjugate_is_identity);
  RUN_TEST(test_unit_quaternion_double_conjugate_is_self);
  RUN_TEST(test_rotate_identity_passes_through);
  RUN_TEST(test_rotate_yaw_90_maps_east_to_north);
  RUN_TEST(test_rotate_yaw_180_negates_xy);
  RUN_TEST(test_euler_identity_is_zero);
  RUN_TEST(test_euler_pure_yaw_recovers_yaw_angle);
  RUN_TEST(test_euler_pure_pitch_recovers_pitch_angle);
  RUN_TEST(test_euler_pure_roll_recovers_roll_angle);
  RUN_TEST(test_euler_near_singular_does_not_nan);
  RUN_TEST(test_world_angular_velocity_identity_passes_through);
  RUN_TEST(test_world_yaw_rate_heeled_boat_projects_by_cos_heel);
  RUN_TEST(test_world_yaw_rate_matches_world_angular_velocity_z);
  RUN_TEST(test_euler_rates_level_pass_through_gyro);
  RUN_TEST(test_euler_rates_heeled_body_z_splits_into_yaw_and_pitch);
  RUN_TEST(test_euler_rates_yaw_matches_world_yaw_rate_at_zero_pitch);
  RUN_TEST(test_euler_rates_jacobian_matches_finite_difference);
  RUN_TEST(test_wrap180_wraps_into_range);

  RUN_TEST(test_bridge_passes_heading_and_variance_through);
  RUN_TEST(test_bridge_subtracts_baseline_offset);
  RUN_TEST(test_bridge_wraps_heading_into_plus_minus_180);
  RUN_TEST(test_bridge_dnu_heading_is_invalid);
  RUN_TEST(test_bridge_no_attitude_mode_is_invalid);
  RUN_TEST(test_bridge_dnu_covariance_uses_fallback);
  return UNITY_END();
}
