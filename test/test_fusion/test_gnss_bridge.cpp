/**
 * Host tests for the GNSS dual-antenna attitude bridge.
 */

#include "gnss_bridge.h"
#include <unity.h>

using namespace fusion;

namespace {

constexpr float TOLERANCE = 1e-4f;

// Mode 2 == heading + pitch from a fixed-ambiguity 2-antenna baseline.
constexpr uint16_t ATT_MODE_HEADING_PITCH = 2;
// Mode 1 == the same baseline resolved with float ambiguities.
constexpr uint16_t ATT_MODE_FLOAT_HEADING_PITCH = 1;

sbf::AttEuler make_att(float heading_deg,
                       uint16_t mode = ATT_MODE_HEADING_PITCH,
                       uint8_t error = 0) {
  sbf::AttEuler att {};
  att.tow = 1000;
  att.mode = mode;
  att.error = error;
  att.heading = heading_deg;
  att.pitch = 0.0f;
  att.roll = sbf::DNU_F4;
  return att;
}

sbf::AttCovEuler make_cov(float heading_variance_deg2) {
  sbf::AttCovEuler cov {};
  cov.cov_headhead = heading_variance_deg2;
  return cov;
}

} // namespace

void test_bridge_passes_heading_and_variance_through() {
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(45.0f), make_cov(4.0f), {});
  TEST_ASSERT_TRUE(s.valid);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 45.0f, s.heading_deg);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 4.0f, s.heading_variance_deg2);
}

void test_bridge_subtracts_baseline_offset() {
  const GnssAttitudeMount mount {.baseline_offset_deg = 20.0f};
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(50.0f), make_cov(4.0f), mount);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 30.0f, s.heading_deg);
}

void test_bridge_wraps_heading_into_plus_minus_180() {
  // 350 deg baseline, 20 deg offset -> 330 boat -> wraps to -30.
  const GnssAttitudeMount mount {.baseline_offset_deg = 20.0f};
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(350.0f), make_cov(4.0f), mount);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -30.0f, s.heading_deg);
}

void test_bridge_dnu_heading_is_invalid() {
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(sbf::DNU_F4), make_cov(4.0f), {});
  TEST_ASSERT_FALSE(s.valid);
}

void test_bridge_no_attitude_mode_is_invalid() {
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(45.0f, 0), make_cov(4.0f), {});
  TEST_ASSERT_FALSE(s.valid);
}

void test_bridge_float_ambiguity_mode_is_invalid() {
  const GnssSample s = att_euler_to_gnss_sample(
      make_att(45.0f, ATT_MODE_FLOAT_HEADING_PITCH), make_cov(4.0f), {});
  TEST_ASSERT_FALSE(s.valid);
}

void test_bridge_fixed_full_attitude_mode_is_valid() {
  const GnssSample s = att_euler_to_gnss_sample(
      make_att(45.0f, ATT_MODE_FIXED_FULL), make_cov(4.0f), {});
  TEST_ASSERT_TRUE(s.valid);
}

void test_bridge_baseline_error_is_invalid() {
  // Error bits 0-1 == 1: not enough measurements on the main-aux1 baseline.
  const GnssSample s = att_euler_to_gnss_sample(
      make_att(45.0f, ATT_MODE_HEADING_PITCH, 1), make_cov(4.0f), {});
  TEST_ASSERT_FALSE(s.valid);
}

void test_bridge_dnu_covariance_uses_fallback() {
  const GnssAttitudeMount mount {.fallback_heading_variance_deg2 = 9.0f};
  const GnssSample s =
      att_euler_to_gnss_sample(make_att(45.0f), make_cov(sbf::DNU_F4), mount);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 9.0f, s.heading_variance_deg2);
}
