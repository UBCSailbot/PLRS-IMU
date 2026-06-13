/**
 * Four-state EKF for heading + roll + pitch + gyro_z bias.
 *
 * Satisfies the FusionFilter concept. State: x[IDX_HEADING] = heading (deg),
 * x[IDX_ROLL] = roll (deg), x[IDX_PITCH] = pitch (deg), x[IDX_GYRO_BIAS] =
 * gyro_z bias (deg/s). Roll/pitch/yaw propagate from the body gyro through the
 * ZYX Euler kinematics; the MTi quaternion is a roll/pitch measurement and the
 * GNSS fix is the heading measurement, both applied as scalar updates.
 *
 * TinyEKF dimensions are passed in as EKF_N / EKF_M macros, captured as
 * N_STATE / N_MEAS constants, then #undef'd so they do not leak into
 * translation units that include this header.
 */

#pragma once

#include "attitude.h"
#include "fusion.h"
#include <cfloat>
#include <cstddef>

#define EKF_N 4
#define EKF_M 1
#include <tinyekf.h>
namespace fusion {
constexpr std::size_t N_STATE = EKF_N;
constexpr std::size_t N_MEAS = EKF_M;
} // namespace fusion
#undef EKF_N
#undef EKF_M

namespace fusion {

constexpr std::size_t IDX_HEADING = 0;
constexpr std::size_t IDX_ROLL = 1;
constexpr std::size_t IDX_PITCH = 2;
constexpr std::size_t IDX_GYRO_BIAS = 3;

/**
 * Observation Jacobians for the scalar updates: each measurement reads one
 * state directly. Heading is shared by the GNSS fix and (in future) any
 * heading source; roll and pitch are read from the MTi quaternion.
 */
constexpr float H_HEADING[N_STATE] = {1.0f, 0.0f, 0.0f, 0.0f};
constexpr float H_ROLL[N_STATE] = {0.0f, 1.0f, 0.0f, 0.0f};
constexpr float H_PITCH[N_STATE] = {0.0f, 0.0f, 1.0f, 0.0f};

class TinyEkfFilter {
public:
  /**
   * Filter tuning parameters. See docs/tuning.md.
   */
  struct Config {
    float q_heading_deg2;
    float q_roll_deg2;
    float q_pitch_deg2;
    float q_bias_deg2_s2;
    float p0_heading_deg2;
    float p0_roll_deg2;
    float p0_pitch_deg2;
    float p0_bias_deg2_s2;
    float mti_roll_variance_deg2;
    float mti_pitch_variance_deg2;
    MountRotation mount {};
  };

  /**
   * @brief Construct with tuning parameters.
   *
   * @param cfg. Filter tuning parameters.
   */
  explicit TinyEkfFilter(Config cfg)
      : _cfg(cfg), _Q {
                       cfg.q_heading_deg2,
                       0.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       cfg.q_roll_deg2,
                       0.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       cfg.q_pitch_deg2,
                       0.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       cfg.q_bias_deg2_s2,
                   } {
    const float pdiag[N_STATE] = {
        _cfg.p0_heading_deg2,
        _cfg.p0_roll_deg2,
        _cfg.p0_pitch_deg2,
        _cfg.p0_bias_deg2_s2,
    };
    ekf_initialize(&_ekf, pdiag);
  }

  /**
   * @brief Advance the filter with the gyro, then correct roll/pitch from the
   * MTi quaternion. Called at IMU rate.
   *
   * The first call sets the timestamp baseline and seeds roll/pitch from the
   * MTi quaternion without advancing state.
   *
   * @param imu. IMU sample.
   */
  void predict(ImuSample imu) {
    const EulerZyx attitude = measured_attitude(imu.orientation);
    if (!_has_predicted) {
      _last_predict_time = imu.timestamp;
      _ekf.x[IDX_ROLL] = attitude.roll_deg;
      _ekf.x[IDX_PITCH] = attitude.pitch_deg;
      _has_predicted = true;
      return;
    }

    const float dt_s =
        std::chrono::duration<float>(imu.timestamp - _last_predict_time)
            .count();
    _last_predict_time = imu.timestamp;

    const float roll_rad = _ekf.x[IDX_ROLL] * DEG_TO_RAD;
    const float pitch_rad = _ekf.x[IDX_PITCH] * DEG_TO_RAD;
    const EulerRates rates =
        euler_rates_zyx(roll_rad, pitch_rad, imu.angular_velocity_rad_s);
    const EulerRatesJacobian jac =
        euler_rates_jacobian(roll_rad, pitch_rad, imu.angular_velocity_rad_s);

    const float fx[N_STATE] = {
        _ekf.x[IDX_HEADING] +
            (rates.yaw_dot * RAD_TO_DEG - _ekf.x[IDX_GYRO_BIAS]) * dt_s,
        _ekf.x[IDX_ROLL] + rates.roll_dot * RAD_TO_DEG * dt_s,
        _ekf.x[IDX_PITCH] + rates.pitch_dot * RAD_TO_DEG * dt_s,
        _ekf.x[IDX_GYRO_BIAS],
    };

    // F = d fx / d x. Euler rates depend on roll/pitch but not heading, so
    // those columns carry the kinematic coupling; gyro bias couples only into
    // heading. The per-radian Jacobian entries times dt land in degree space:
    // the RAD_TO_DEG and DEG_TO_RAD conversions cancel.
    const float F[N_STATE * N_STATE] = {
        1.0f,
        jac.dyaw_droll * dt_s,
        jac.dyaw_dpitch * dt_s,
        -dt_s,
        0.0f,
        1.0f + jac.droll_droll * dt_s,
        jac.droll_dpitch * dt_s,
        0.0f,
        0.0f,
        jac.dpitch_droll * dt_s,
        1.0f + jac.dpitch_dpitch * dt_s,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
    };

    ekf_predict(&_ekf, fx, F, _Q);
    _ekf.x[IDX_HEADING] = wrap180(_ekf.x[IDX_HEADING]);

    scalar_update(H_ROLL,
                  _ekf.x[IDX_ROLL],
                  attitude.roll_deg,
                  _cfg.mti_roll_variance_deg2);
    scalar_update(H_PITCH,
                  _ekf.x[IDX_PITCH],
                  attitude.pitch_deg,
                  _cfg.mti_pitch_variance_deg2);
  }

  /**
   * @brief Correct the heading estimate from a GNSS measurement.
   *
   * No-op if !gnss.valid. The first valid sample seeds x[IDX_HEADING]
   * directly and marks the filter initialized; subsequent valid samples run a
   * scalar update with the innovation wrapped to +-180 so a fix across the
   * +-180 seam does not drag heading the long way around.
   *
   * @param gnss. GNSS sample.
   */
  void update(GnssSample gnss) {
    if (!gnss.valid) {
      return;
    }
    if (!_initialized) {
      _ekf.x[IDX_HEADING] = wrap180(gnss.heading_deg);
      _initialized = true;
      return;
    }
    const float hx = _ekf.x[IDX_HEADING];
    const float z = hx + wrap180(gnss.heading_deg - hx);
    scalar_update(H_HEADING, hx, z, gnss.heading_variance_deg2);
    _ekf.x[IDX_HEADING] = wrap180(_ekf.x[IDX_HEADING]);
  }

  /**
   * @brief Read the current fused estimate.
   *
   * @return Current FusionOutput. heading_variance_deg2 is FLT_MAX before the
   * first valid GNSS update; roll/pitch variances are FLT_MAX before the first
   * predict seeds them. Afterward each is the matching P diagonal entry.
   */
  FusionOutput output() const {
    return FusionOutput {
        .heading_deg = _ekf.x[IDX_HEADING],
        .heading_variance_deg2 =
            _initialized ? _ekf.P[IDX_HEADING * N_STATE + IDX_HEADING]
                         : FLT_MAX,
        .roll_deg = _ekf.x[IDX_ROLL],
        .roll_variance_deg2 =
            _has_predicted ? _ekf.P[IDX_ROLL * N_STATE + IDX_ROLL] : FLT_MAX,
        .pitch_deg = _ekf.x[IDX_PITCH],
        .pitch_variance_deg2 =
            _has_predicted ? _ekf.P[IDX_PITCH * N_STATE + IDX_PITCH] : FLT_MAX,
        .timestamp = _last_predict_time,
    };
  }

private:
  /**
   * @brief Boat-frame attitude from the raw MTi quaternion via the mount
   * rotation.
   */
  EulerZyx measured_attitude(UnitQuaternion orientation) const {
    const UnitQuaternion boat =
        UnitQuaternion::multiply(orientation, _cfg.mount.boat_to_imu);
    return quaternion_to_euler_zyx(boat);
  }

  /**
   * @brief One-dimensional EKF update reading a single state through H.
   *
   * @param H. Observation row selecting the measured state.
   * @param hx. Predicted measurement (the current value of that state).
   * @param z. Measurement, already innovation-wrapped where it matters.
   * @param variance. Measurement noise variance (R).
   */
  void
  scalar_update(const float H[N_STATE], float hx, float z, float variance) {
    const float zv[N_MEAS] = {z};
    const float hxv[N_MEAS] = {hx};
    const float R[N_MEAS * N_MEAS] = {variance};
    ekf_update(&_ekf, zv, hxv, H, R);
  }

  ekf_t _ekf;
  Config _cfg;
  float _Q[N_STATE * N_STATE];
  Ms _last_predict_time {0};
  bool _initialized = false;
  bool _has_predicted = false;
};

} // namespace fusion
