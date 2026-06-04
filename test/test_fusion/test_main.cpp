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
  return UNITY_END();
}
