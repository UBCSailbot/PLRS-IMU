/**
 * Host (native) unit tests for the fusion EKF filter.
 */

#include "ekf_filter.h"
#include "fusion.h"
#include <cfloat>
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

constexpr TinyEkfFilter::Config kTestConfig{
    .q_heading_deg2 = 0.01f,
    .q_bias_deg2_s2 = 0.0001f,
    .p0_heading_deg2 = 1000.0f,
    .p0_bias_deg2_s2 = 1.0f,
};

ImuSample make_imu(float rate_z_rad_s, Ms t) {
  return ImuSample{
      .angular_velocity_rad_s = Vec3{0.0f, 0.0f, rate_z_rad_s},
      .accel_ms2 = Vec3{0.0f, 0.0f, GRAVITY_MS2},
      .timestamp = t,
  };
}

GnssSample make_gnss(float heading_deg, float variance_deg2, Ms t,
                     bool valid = true) {
  return GnssSample{
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
  f.update(make_gnss(45.0f, 1.0f, Ms{1000}, false));
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
  f.update(make_gnss(45.0f, 1.0f, Ms{1000}));
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(45.0f, out.heading_deg);
  TEST_ASSERT_EQUAL_FLOAT(1000.0f, out.heading_variance_deg2);
}

/** @brief A tight GNSS update pulls heading strongly toward the measurement. */
void test_update_pulls_toward_measurement() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms{1000}));    // seed at 0
  f.predict(make_imu(0.0f, Ms{1000}));          // baseline
  f.predict(make_imu(0.0f, Ms{5000}));          // grow P over 4 s
  f.update(make_gnss(45.0f, 0.001f, Ms{5000})); // very tight R
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 45.0f, out.heading_deg);
}

/** @brief A measurement update shrinks the heading variance. */
void test_update_shrinks_variance() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms{1000}));
  f.predict(make_imu(0.0f, Ms{1000})); // baseline
  f.predict(make_imu(0.0f, Ms{2000})); // grow P by Q
  auto before = f.output().heading_variance_deg2;
  f.update(make_gnss(0.0f, 0.1f, Ms{2000}));
  auto after = f.output().heading_variance_deg2;
  TEST_ASSERT_TRUE(after < before);
}

// ---------------------------------------------------------------------------
// predict()
// ---------------------------------------------------------------------------

/** @brief First predict() sets the timestamp baseline; no state advance. */
void test_first_predict_baseline_only() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(45.0f, 1.0f, Ms{1000}));
  f.predict(make_imu(1.0f, Ms{2000})); // 1 rad/s but first predict call
  auto out = f.output();
  TEST_ASSERT_EQUAL_FLOAT(45.0f, out.heading_deg);
}

/** @brief Gyro_z integrates into heading: 0.5 rad/s for 1 s ~= 28.6479 deg. */
void test_predict_integrates_gyro() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms{1000}));
  f.predict(make_imu(0.0f, Ms{1000})); // baseline
  f.predict(make_imu(0.5f, Ms{2000})); // 0.5 rad/s * 1 s
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.6479f, out.heading_deg);
}

/** @brief A predict step grows the heading variance (process noise added). */
void test_predict_grows_variance() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms{1000}));
  f.predict(make_imu(0.0f, Ms{1000})); // baseline
  auto before = f.output().heading_variance_deg2;
  f.predict(make_imu(0.0f, Ms{2000}));
  auto after = f.output().heading_variance_deg2;
  TEST_ASSERT_TRUE(after > before);
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
  return UNITY_END();
}
