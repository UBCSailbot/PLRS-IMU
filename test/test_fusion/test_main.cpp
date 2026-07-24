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

namespace {

/** @brief Converge heading to 0 with tight fixes so P_hh is small. */
TinyEkfFilter make_converged_filter() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000}));
  for (int i = 2; i <= 10; i++) {
    f.predict(make_imu(0.0f, Ms {1000 * i}));
    f.update(make_gnss(0.0f, 0.1f, Ms {1000 * i}));
  }
  return f;
}

} // namespace

/** @brief A fix far outside the innovation gate is rejected. */
void test_update_gates_outlier_fix() {
  TinyEkfFilter f = make_converged_filter();
  const float before = f.output().heading_deg;
  f.update(make_gnss(60.0f, 0.1f, Ms {11000}));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, before, f.output().heading_deg);
}

/** @brief Consistent fixes after a gated outlier are still accepted. */
void test_update_gate_passes_consistent_fix_after_outlier() {
  TinyEkfFilter f = make_converged_filter();
  f.update(make_gnss(60.0f, 0.1f, Ms {11000})); // gated
  auto before = f.output().heading_variance_deg2;
  f.update(make_gnss(0.0f, 0.05f, Ms {11000}));
  TEST_ASSERT_TRUE(f.output().heading_variance_deg2 < before);
}

/** @brief HEADING_GATE_LIMIT consecutive rejections force an acceptance, so a
 * persistently disagreeing receiver pulls the filter back eventually. */
void test_update_gate_reopens_after_limit() {
  TinyEkfFilter f = make_converged_filter();
  for (uint32_t i = 0; i < TinyEkfFilter::HEADING_GATE_LIMIT; i++) {
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, f.output().heading_deg);
    f.update(make_gnss(60.0f, 0.1f, Ms {11000 + i}));
  }
  // The limit-th fix was applied: heading moved decisively toward 60.
  TEST_ASSERT_TRUE(f.output().heading_deg > 5.0f);
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

/**
 * @brief Gyro_z integrates into heading. Heading is compass (CW-positive), so a
 * CCW gyro of 0.5 rad/s for 1 s drives it to ~= -28.6479 deg.
 */
void test_predict_integrates_gyro() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(0.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000})); // baseline
  f.predict(make_imu(0.5f, Ms {2000})); // 0.5 rad/s * 1 s
  auto out = f.output();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -28.6479f, out.heading_deg);
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
 * 1 rad/s, decoupled from heel, while roll and pitch hold steady. Heading is
 * compass (CW-positive), so the CCW world yaw shows up negative.
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
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -yaw_rate * RAD_TO_DEG, out.heading_deg);
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

// ---------------------------------------------------------------------------
// MTi yaw aiding (mag offset state)
// ---------------------------------------------------------------------------

namespace {

// True heading 45, mag yaw reading 65 in compass sign: the offset state must
// converge to their 20 deg difference once GNSS anchors absolute heading,
// which the tests observe through the heading staying at 45.
constexpr float TRUE_HEADING = 45.0f;
constexpr float MAG_COMPASS = 65.0f;

TinyEkfFilter::Config make_mag_config() {
  TinyEkfFilter::Config cfg = kTestConfig;
  cfg.mti_yaw = TinyEkfFilter::MtiYawConfig {
      .variance_deg2 = 4.0f,
      .q_offset_deg2 = 0.0001f,
      .p0_offset_deg2 = 100.0f,
  };
  return cfg;
}

/** @brief Stationary sample whose quaternion yaw reads MAG_COMPASS on the
 * compass (ENU yaw is CCW-positive, so the quaternion carries its negation),
 * with an optional raw gyro-z bias. */
ImuSample make_mag_imu(float gyro_bias_dps, Ms t) {
  return make_imu_with(plrs::Vec3 {0.0f, 0.0f, -gyro_bias_dps * DEG_TO_RAD},
                       axis_angle(0.0f, 0.0f, 1.0f, -MAG_COMPASS * DEG_TO_RAD),
                       t);
}

/** @brief Converge a mag-enabled filter: 10 s of 10 Hz IMU + 1 Hz GNSS. */
template <typename Fix>
void run_converged(TinyEkfFilter &f, Fix fix, float gyro_bias_dps = 0.0f) {
  for (int i = 0; i <= 100; i++) {
    const Ms t {1000 + 100 * i};
    f.predict(make_mag_imu(gyro_bias_dps, t));
    if (i % 10 == 0) {
      fix(f, t);
    }
  }
}

} // namespace

/** @brief With GNSS present the offset state converges to mag minus GNSS. */
void test_mag_offset_converges_under_gnss() {
  TinyEkfFilter f(make_mag_config());
  run_converged(f, [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  });
  TEST_ASSERT_FLOAT_WITHIN(1.0f, TRUE_HEADING, f.output().heading_deg);
}

/** @brief Through a GNSS outage the mag pins heading against a gyro bias the
 * filter never saw while GNSS was up, and reacquisition does not snap. */
void test_mag_holds_heading_through_gnss_dropout() {
  TinyEkfFilter f(make_mag_config());
  TinyEkfFilter bare(kTestConfig);
  auto fix = [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  };
  run_converged(f, fix);
  run_converged(bare, fix);

  // 30 s outage with a 0.3 deg/s raw gyro bias appearing mid-flight.
  for (int i = 1; i <= 300; i++) {
    const Ms t {11100 + 100 * i};
    f.predict(make_mag_imu(0.3f, t));
    bare.predict(make_mag_imu(0.3f, t));
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, TRUE_HEADING, f.output().heading_deg);
  // Without the mag the same bias integrates into a ~9 deg ramp.
  TEST_ASSERT_TRUE(std::abs(wrap180(bare.output().heading_deg - TRUE_HEADING)) >
                   5.0f);

  // Reacquisition: a fix at the held heading barely moves the estimate.
  const float before = f.output().heading_deg;
  f.update(make_gnss(TRUE_HEADING, 1.0f, Ms {41200}));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, before, f.output().heading_deg);
}

/** @brief A persistent mag shift (iron event) drains into the offset state;
 * GNSS keeps ownership of absolute heading. */
void test_mag_cannot_pull_heading_against_gnss() {
  TinyEkfFilter f(make_mag_config());
  run_converged(f, [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  });

  // The mag jumps +30 deg; GNSS keeps reporting the true heading for 60 s.
  for (int i = 1; i <= 600; i++) {
    const Ms t {11100 + 100 * i};
    ImuSample imu = make_mag_imu(0.0f, t);
    imu.orientation =
        axis_angle(0.0f, 0.0f, 1.0f, -(MAG_COMPASS + 30.0f) * DEG_TO_RAD);
    f.predict(imu);
    if (i % 10 == 0) {
      f.update(make_gnss(TRUE_HEADING, 1.0f, t));
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(5.0f, TRUE_HEADING, f.output().heading_deg);
}

/** @brief The first GNSS fix shifts the offset with the seed, so the next mag
 * sample does not drag heading back toward its pre-fix value. */
void test_mag_gnss_seed_keeps_offset_consistent() {
  TinyEkfFilter f(make_mag_config());
  // Mag-only warmup: heading converges toward the mag compass value.
  for (int i = 0; i <= 50; i++) {
    f.predict(make_mag_imu(0.0f, Ms {1000 + 100 * i}));
  }
  f.update(make_gnss(TRUE_HEADING, 1.0f, Ms {6100}));
  for (int i = 1; i <= 50; i++) {
    f.predict(make_mag_imu(0.0f, Ms {6100 + 100 * i}));
  }
  TEST_ASSERT_FLOAT_WITHIN(1.5f, TRUE_HEADING, f.output().heading_deg);
}

/** @brief A mag snap during a GNSS outage is gated: heading holds instead of
 * walking toward the snapped yaw. */
void test_mag_gate_rejects_snap_during_outage() {
  TinyEkfFilter f(make_mag_config());
  run_converged(f, [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  });

  // GNSS gone; the mag snaps +40 deg and stays there for 10 s.
  for (int i = 1; i <= 100; i++) {
    ImuSample imu = make_mag_imu(0.0f, Ms {11100 + 100 * i});
    imu.orientation =
        axis_angle(0.0f, 0.0f, 1.0f, -(MAG_COMPASS + 40.0f) * DEG_TO_RAD);
    f.predict(imu);
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, TRUE_HEADING, f.output().heading_deg);
  TEST_ASSERT_TRUE(f.debug().mag_gate_rejects > 0);
}

/** @brief Persistent mag disagreement is force-accepted once per
 * MTI_YAW_GATE_LIMIT rejects, so a filter converged wrong cannot ignore the
 * mag forever, while a transient snap still cannot own heading. */
void test_mag_gate_forced_accept_after_limit() {
  TinyEkfFilter f(make_mag_config());
  run_converged(f, [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  });

  const float before = f.output().heading_deg;
  auto step_snapped_mag = [&](uint32_t i) {
    ImuSample imu = make_mag_imu(0.0f, Ms {11100 + 100 * i});
    imu.orientation =
        axis_angle(0.0f, 0.0f, 1.0f, -(MAG_COMPASS + 40.0f) * DEG_TO_RAD);
    f.predict(imu);
  };

  // Right after convergence the heading variance is small, so the 40-deg snap
  // fails the gate: it is rejected and heading stays pinned. This proves the
  // gate holds the snap off, not that heading merely failed to drift.
  for (uint32_t i = 1; i <= 5; i++) {
    step_snapped_mag(i);
  }
  TEST_ASSERT_TRUE(std::abs(wrap180(f.output().heading_deg - before)) < 1.0f);
  TEST_ASSERT_EQUAL_UINT32(5, f.debug().mag_gate_rejects);

  // Held through a long snap, the mag eventually takes most of the authority:
  // correct, since after 30 s with no other reference it is the best guess.
  for (uint32_t i = 6; i <= TinyEkfFilter::MTI_YAW_GATE_LIMIT + 100; i++) {
    step_snapped_mag(i);
  }
  TEST_ASSERT_TRUE(std::abs(wrap180(f.output().heading_deg - before)) > 5.0f);
}

/** @brief Through an indefinite degraded outage (mag snapped 90 deg, no
 * GNSS) the offset cap engages and, with P_offset pinned, the mag updates
 * keep heading variance bounded near the same scale: no P entry ever
 * approaches float32 cancellation territory. */
void test_variances_bounded_during_long_degraded_outage() {
  TinyEkfFilter::Config cfg = make_mag_config();
  cfg.q_heading_deg2 = 10.0f; // reach the bounds within a few thousand steps
  cfg.mti_yaw->q_offset_deg2 = 10.0f;
  TinyEkfFilter f(cfg);
  f.update(make_gnss(TRUE_HEADING, 1.0f, Ms {1000}));
  for (int i = 1; i <= 100; i++) {
    f.predict(make_mag_imu(0.0f, Ms {1000 + 100 * i}));
  }
  for (int i = 1; i <= 5000; i++) {
    ImuSample imu = make_mag_imu(0.0f, Ms {11100 + 100 * i});
    imu.orientation =
        axis_angle(0.0f, 0.0f, 1.0f, -(MAG_COMPASS + 90.0f) * DEG_TO_RAD);
    f.predict(imu);
  }
  constexpr float cap_offset = TinyEkfFilter::MAG_OFFSET_SIGMA_CAP_DEG *
                               TinyEkfFilter::MAG_OFFSET_SIGMA_CAP_DEG;
  const float var_offset = f.debug().mag_offset_variance_deg2;
  TEST_ASSERT_TRUE(var_offset <= cap_offset * 1.001f);
  TEST_ASSERT_TRUE(var_offset > 0.5f * cap_offset); // the cap engaged
  TEST_ASSERT_TRUE(f.output().heading_variance_deg2 < 3.0f * cap_offset);
}

/** @brief With the shipped loose offset (absorbs mag wander while anchored),
 * heading confidence decays through a GNSS outage: nothing pins the offset, so
 * the mag cannot hold heading and the variance climbs past the steer-on bound.
 * Setting q_offset_outage_deg2 pins the offset once GNSS is gone, so a clean
 * mag holds heading AND keeps it confident enough to steer on. This is the
 * usable-coast lever; it is safe only when the mag is genuinely clean. */
void test_outage_offset_pin_extends_confident_hold() {
  constexpr float kSteerOnVariance = 25.0f; // rudder_task heading-valid bound
  TinyEkfFilter::Config loose = make_mag_config();
  loose.mti_yaw->q_offset_deg2 = 1.0f; // shipped tuning: loose while anchored
  TinyEkfFilter::Config pinned = loose;
  pinned.mti_yaw->q_offset_outage_deg2 = 1.0e-4f;

  TinyEkfFilter f_loose(loose);
  TinyEkfFilter f_pin(pinned);
  auto fix = [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  };
  run_converged(f_loose, fix);
  run_converged(f_pin, fix);

  // 60 s clean-mag outage: the mag keeps reading MAG_COMPASS truthfully.
  for (int i = 1; i <= 600; i++) {
    const Ms t {11100 + 100 * i};
    f_loose.predict(make_mag_imu(0.0f, t));
    f_pin.predict(make_mag_imu(0.0f, t));
  }

  // The pinned filter holds heading near truth and stays confident.
  TEST_ASSERT_FLOAT_WITHIN(3.0f, TRUE_HEADING, f_pin.output().heading_deg);
  TEST_ASSERT_TRUE(f_pin.output().heading_variance_deg2 < kSteerOnVariance);
  // The default (loose, unpinned) offset lets confidence decay past the bound.
  TEST_ASSERT_TRUE(f_loose.output().heading_variance_deg2 > kSteerOnVariance);
}

/** @brief Within the outage grace the offset still uses the anchored random
 * walk, so a brief GNSS gap does not switch behaviour. */
void test_outage_offset_pin_waits_for_grace() {
  TinyEkfFilter::Config loose = make_mag_config();
  loose.mti_yaw->q_offset_deg2 = 1.0f;
  TinyEkfFilter::Config pinned = loose;
  pinned.mti_yaw->q_offset_outage_deg2 = 1.0e-4f;

  TinyEkfFilter f_loose(loose);
  TinyEkfFilter f_pin(pinned);
  auto fix = [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  };
  run_converged(f_loose, fix);
  run_converged(f_pin, fix);

  // A gap shorter than MAG_OUTAGE_GRACE: the pin must not have engaged yet, so
  // the two filters still track together.
  for (int i = 1; i <= 20; i++) { // 2 s < 3 s grace
    const Ms t {11100 + 100 * i};
    f_loose.predict(make_mag_imu(0.0f, t));
    f_pin.predict(make_mag_imu(0.0f, t));
  }
  TEST_ASSERT_FLOAT_WITHIN(0.5f,
                           f_loose.output().heading_variance_deg2,
                           f_pin.output().heading_variance_deg2);
}

/** @brief The grace boundary is strict (>): at exactly MAG_OUTAGE_GRACE since
 * the last fix the pin is still disengaged, so the pinned filter matches the
 * loose one; one step past, the pin engages and the mag-offset variance
 * diverges. */
void test_outage_offset_pin_grace_boundary_is_strict() {
  TinyEkfFilter::Config loose = make_mag_config();
  loose.mti_yaw->q_offset_deg2 = 1.0f;
  TinyEkfFilter::Config pinned = loose;
  pinned.mti_yaw->q_offset_outage_deg2 = 1.0e-4f;

  TinyEkfFilter f_loose(loose);
  TinyEkfFilter f_pin(pinned);
  auto fix = [](TinyEkfFilter &g, Ms t) {
    g.update(make_gnss(TRUE_HEADING, 1.0f, t));
  };
  run_converged(f_loose, fix);
  run_converged(f_pin,
                fix); // last fix at t = 11000, so _last_gnss_time = 11000

  const int last_fix_ms = 11000;
  const int grace_ms =
      static_cast<int>(TinyEkfFilter::MAG_OUTAGE_GRACE.count());
  // Predict up to exactly grace (t - last == grace): the strict > keeps the pin
  // off, so the two filters use the same offset Q and stay together.
  for (int t = last_fix_ms + 100; t <= last_fix_ms + grace_ms; t += 100) {
    f_loose.predict(make_mag_imu(0.0f, Ms {t}));
    f_pin.predict(make_mag_imu(0.0f, Ms {t}));
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0e-3f,
                           f_loose.debug().mag_offset_variance_deg2,
                           f_pin.debug().mag_offset_variance_deg2);

  // One step past grace: the pin engages, so the pinned offset variance grows
  // more slowly than the loose one and the two separate.
  const Ms past {last_fix_ms + grace_ms + 100};
  f_loose.predict(make_mag_imu(0.0f, past));
  f_pin.predict(make_mag_imu(0.0f, past));
  TEST_ASSERT_TRUE(f_pin.debug().mag_offset_variance_deg2 <
                   f_loose.debug().mag_offset_variance_deg2);
}

/** @brief With no mag at all, nothing bounds heading variance through an
 * outage except its own cap. */
void test_heading_variance_capped_without_mag() {
  TinyEkfFilter::Config cfg = kTestConfig;
  cfg.q_heading_deg2 = 10.0f;
  TinyEkfFilter f(cfg);
  f.update(make_gnss(TRUE_HEADING, 1.0f, Ms {1000}));
  for (int i = 1; i <= 5000; i++) {
    f.predict(make_imu(0.0f, Ms {1000 + 100 * i}));
  }
  constexpr float cap_heading = TinyEkfFilter::HEADING_SIGMA_CAP_DEG *
                                TinyEkfFilter::HEADING_SIGMA_CAP_DEG;
  TEST_ASSERT_FLOAT_WITHIN(
      0.01f * cap_heading, cap_heading, f.output().heading_variance_deg2);
}

/** @brief A body-frame gyro-Y bias learned at level keeps correcting after
 * the boat heels: heading holds through a heeled GNSS outage, where a
 * compass-rate bias model would drift at ~sin(heel) * bias. */
void test_body_frame_bias_survives_heel_change() {
  TinyEkfFilter f(make_mag_config());
  const float bias_y_dps = 0.3f;
  const plrs::Vec3 biased_gyro {0.0f, bias_y_dps * DEG_TO_RAD, 0.0f};
  const UnitQuaternion yaw_q =
      axis_angle(0.0f, 0.0f, 1.0f, -MAG_COMPASS * DEG_TO_RAD);

  // Level convergence with GNSS: pitch residuals teach the Y bias.
  for (int i = 0; i <= 200; i++) {
    const Ms t {1000 + 100 * i};
    f.predict(make_imu_with(biased_gyro, yaw_q, t));
    if (i % 10 == 0) {
      f.update(make_gnss(TRUE_HEADING, 1.0f, t));
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(0.15f, bias_y_dps, f.debug().gyro_bias_y_dps);

  // Heel 75 deg and drop GNSS for 30 s; the same body-frame bias now maps
  // almost fully into the heading rate, but the learned state cancels it.
  const UnitQuaternion heeled = UnitQuaternion::multiply(
      yaw_q, axis_angle(1.0f, 0.0f, 0.0f, 75.0f * DEG_TO_RAD));
  for (int i = 1; i <= 300; i++) {
    f.predict(make_imu_with(biased_gyro, heeled, Ms {21100 + 100 * i}));
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, TRUE_HEADING, f.output().heading_deg);
}

/**
 * @brief A learned vertical-gyro bias cancels at moderate pitch trim, where
 * sec(pitch) amplifies it into the heading rate. The pitch companion to the
 * heel test; past ~85 deg the ZYX-Euler kinematics go singular and this no
 * longer holds (the field bench regime), where the clamp keeps heading finite
 * rather than accurate -- see the recovery test below and
 * tests/test_attitude_drift.py.
 */
void test_body_frame_bias_survives_moderate_trim() {
  TinyEkfFilter f(make_mag_config());
  const float bias_z_dps = 0.3f;
  const plrs::Vec3 biased_gyro {0.0f, 0.0f, bias_z_dps * DEG_TO_RAD};
  const UnitQuaternion yaw_q =
      axis_angle(0.0f, 0.0f, 1.0f, -MAG_COMPASS * DEG_TO_RAD);

  // Level convergence with GNSS: heading residuals teach the vertical bias.
  for (int i = 0; i <= 400; i++) {
    const Ms t {1000 + 100 * i};
    f.predict(make_imu_with(biased_gyro, yaw_q, t));
    if (i % 5 == 0) {
      f.update(make_gnss(TRUE_HEADING, 1.0f, t));
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(0.15f, bias_z_dps, f.debug().gyro_bias_dps);

  // Trim 60 deg (pitch about body Y) and drop GNSS for 30 s; sec(60) doubles
  // the vertical bias into the heading rate, but the learned state cancels it.
  const UnitQuaternion trimmed = UnitQuaternion::multiply(
      yaw_q, axis_angle(0.0f, 1.0f, 0.0f, 60.0f * DEG_TO_RAD));
  for (int i = 1; i <= 300; i++) {
    f.predict(make_imu_with(biased_gyro, trimmed, Ms {41100 + 100 * i}));
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, TRUE_HEADING, f.output().heading_deg);
}

/**
 * @brief The pitch clamp keeps the filter finite through a near-vertical
 * excursion, so heading re-anchors once the boat drops back to level. Without
 * PITCH_KINEMATICS_LIMIT_DEG the covariance would go NaN near 90 deg and never
 * recover; this is the graceful-recovery guarantee.
 */
void test_heading_recovers_after_near_vertical_excursion() {
  TinyEkfFilter f(make_mag_config());
  const plrs::Vec3 gyro {0.0f, 0.0f, 0.0f};
  const UnitQuaternion yaw_q =
      axis_angle(0.0f, 0.0f, 1.0f, -MAG_COMPASS * DEG_TO_RAD);

  // Converge level with GNSS.
  for (int i = 0; i <= 200; i++) {
    const Ms t {1000 + 100 * i};
    f.predict(make_imu_with(gyro, yaw_q, t));
    if (i % 5 == 0) {
      f.update(make_gnss(TRUE_HEADING, 1.0f, t));
    }
  }

  // Tip to 88 deg pitch with GNSS out for 20 s; the state must stay finite.
  const UnitQuaternion vertical = UnitQuaternion::multiply(
      yaw_q, axis_angle(0.0f, 1.0f, 0.0f, 88.0f * DEG_TO_RAD));
  for (int i = 1; i <= 200; i++) {
    f.predict(make_imu_with(gyro, vertical, Ms {21100 + 100 * i}));
  }
  TEST_ASSERT_TRUE(std::isfinite(f.output().heading_deg));

  // Drop back to level with GNSS; heading must re-anchor to truth.
  for (int i = 1; i <= 200; i++) {
    const Ms t {41200 + 100 * i};
    f.predict(make_imu_with(gyro, yaw_q, t));
    if (i % 5 == 0) {
      f.update(make_gnss(TRUE_HEADING, 1.0f, t));
    }
  }
  TEST_ASSERT_TRUE(std::isfinite(f.output().heading_deg));
  TEST_ASSERT_FLOAT_WITHIN(2.0f, TRUE_HEADING, f.output().heading_deg);
}

/** @brief heading_trustworthy gates on finiteness, variance, and pitch. */
void test_heading_trustworthy_gates_variance_pitch_and_finiteness() {
  const float max_var = 25.0f;
  const float max_pitch = 80.0f;
  fusion::FusionOutput ok {};
  ok.heading_deg = 90.0f;
  ok.heading_variance_deg2 = 4.0f;
  ok.pitch_deg = 30.0f;
  TEST_ASSERT_TRUE(fusion::heading_trustworthy(ok, max_var, max_pitch));

  fusion::FusionOutput noisy = ok;
  noisy.heading_variance_deg2 = 30.0f;
  TEST_ASSERT_FALSE(fusion::heading_trustworthy(noisy, max_var, max_pitch));

  fusion::FusionOutput steep = ok;
  steep.pitch_deg = 85.0f;
  TEST_ASSERT_FALSE(fusion::heading_trustworthy(steep, max_var, max_pitch));

  fusion::FusionOutput steep_neg = ok;
  steep_neg.pitch_deg = -85.0f;
  TEST_ASSERT_FALSE(fusion::heading_trustworthy(steep_neg, max_var, max_pitch));

  fusion::FusionOutput nan = ok;
  nan.heading_deg = NAN;
  TEST_ASSERT_FALSE(fusion::heading_trustworthy(nan, max_var, max_pitch));
}

/** @brief Heading stays wrapped to (-180, 180] after integrating past 180. */
void test_predict_wraps_heading_past_180() {
  TinyEkfFilter f(kTestConfig);
  f.update(make_gnss(170.0f, 1.0f, Ms {1000}));
  f.predict(make_imu(0.0f, Ms {1000}));
  // Compass: a CW (negative gyro) turn raises heading +28.6 -> 198.6 -> -161.4.
  f.predict(make_imu(-0.5f, Ms {2000}));
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
void test_bridge_float_ambiguity_mode_is_invalid();
void test_bridge_fixed_full_attitude_mode_is_valid();
void test_bridge_baseline_error_is_invalid();
void test_bridge_dnu_covariance_uses_fallback();

// ---------------------------------------------------------------------------

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_initial_state_uninitialized);
  RUN_TEST(test_invalid_gnss_is_noop);
  RUN_TEST(test_first_valid_gnss_seeds_heading);
  RUN_TEST(test_update_pulls_toward_measurement);
  RUN_TEST(test_update_shrinks_variance);
  RUN_TEST(test_update_gates_outlier_fix);
  RUN_TEST(test_update_gate_passes_consistent_fix_after_outlier);
  RUN_TEST(test_update_gate_reopens_after_limit);
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
  RUN_TEST(test_mag_offset_converges_under_gnss);
  RUN_TEST(test_mag_holds_heading_through_gnss_dropout);
  RUN_TEST(test_mag_cannot_pull_heading_against_gnss);
  RUN_TEST(test_mag_gnss_seed_keeps_offset_consistent);
  RUN_TEST(test_mag_gate_rejects_snap_during_outage);
  RUN_TEST(test_mag_gate_forced_accept_after_limit);
  RUN_TEST(test_variances_bounded_during_long_degraded_outage);
  RUN_TEST(test_outage_offset_pin_extends_confident_hold);
  RUN_TEST(test_outage_offset_pin_waits_for_grace);
  RUN_TEST(test_outage_offset_pin_grace_boundary_is_strict);
  RUN_TEST(test_heading_variance_capped_without_mag);
  RUN_TEST(test_body_frame_bias_survives_heel_change);
  RUN_TEST(test_body_frame_bias_survives_moderate_trim);
  RUN_TEST(test_heading_recovers_after_near_vertical_excursion);
  RUN_TEST(test_heading_trustworthy_gates_variance_pitch_and_finiteness);
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
  RUN_TEST(test_bridge_float_ambiguity_mode_is_invalid);
  RUN_TEST(test_bridge_fixed_full_attitude_mode_is_valid);
  RUN_TEST(test_bridge_baseline_error_is_invalid);
  RUN_TEST(test_bridge_dnu_covariance_uses_fallback);
  return UNITY_END();
}
