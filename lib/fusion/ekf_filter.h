/**
 * Two-state EKF for heading + gyro_z bias.
 *
 * Satisfies the FusionFilter concept. State: x[IDX_HEADING] = heading (deg),
 * x[IDX_GYRO_BIAS] = gyro_z bias (deg/s). Measurement:
 * z[IDX_GNSS_HEADING] = GNSS heading (deg).
 *
 * TinyEKF dimensions are passed in as EKF_N / EKF_M macros, captured as
 * N_STATE / N_MEAS constants, then #undef'd so they do not leak into
 * translation units that include this header.
 */

#pragma once

#include "fusion.h"
#include <cfloat>
#include <cstddef>
#include <numbers>

#define EKF_N 2
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
constexpr std::size_t IDX_GYRO_BIAS = 1;
constexpr std::size_t IDX_GNSS_HEADING = 0;

constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>;

/**
 * Observation Jacobian: heading is read directly from x[IDX_HEADING],
 * with no contribution from x[IDX_GYRO_BIAS]. Constant across all updates.
 */
constexpr float H_GNSS_HEADING[N_MEAS * N_STATE] = {
    1.0f,
    0.0f,
};

class TinyEkfFilter {
public:
  /**
   * Filter tuning parameters. See docs/tuning.md.
   */
  struct Config {
    float q_heading_deg2;
    float q_bias_deg2_s2;
    float p0_heading_deg2;
    float p0_bias_deg2_s2;
  };

  /**
   * @brief Construct with tuning parameters.
   *
   * @param cfg. Filter tuning parameters.
   */
  explicit TinyEkfFilter(Config cfg)
      : _cfg(cfg), _Q{cfg.q_heading_deg2, 0.0f, 0.0f, cfg.q_bias_deg2_s2} {
    const float pdiag[N_STATE] = {
        _cfg.p0_heading_deg2,
        _cfg.p0_bias_deg2_s2,
    };
    ekf_initialize(&_ekf, pdiag);
  }

  /**
   * @brief Advance the filter using the gyro_z rate. Called at IMU rate.
   *
   * The first call sets the timestamp baseline without advancing state.
   *
   * @param imu. IMU sample.
   */
  void predict(ImuSample imu) {
    if (!_has_predicted) {
      _last_predict_time = imu.timestamp;
      _has_predicted = true;
      return;
    }

    const float dt_s =
        std::chrono::duration<float>(imu.timestamp - _last_predict_time)
            .count();
    _last_predict_time = imu.timestamp;

    const float gyro_z_deg_s = imu.rate_of_turn_z_rad_s * RAD_TO_DEG;

    const float fx[N_STATE] = {
        _ekf.x[IDX_HEADING] + (gyro_z_deg_s - _ekf.x[IDX_GYRO_BIAS]) * dt_s,
        _ekf.x[IDX_GYRO_BIAS],
    };
    const float F[N_STATE * N_STATE] = {
        1.0f,
        -dt_s,
        0.0f,
        1.0f,
    };

    ekf_predict(&_ekf, fx, F, _Q);
  }

  /**
   * @brief Correct the heading estimate from a GNSS measurement.
   *
   * No-op if !gnss.valid. The first valid sample seeds x[IDX_HEADING]
   * directly and marks the filter initialized; subsequent valid samples
   * run the standard EKF update. The return value of ekf_update (false on
   * Cholesky failure) is dropped because state is left unchanged on
   * failure and no caller-visible action would help.
   *
   * @param gnss. GNSS sample.
   */
  void update(GnssSample gnss) {
    if (!gnss.valid) {
      return;
    }
    if (!_initialized) {
      _ekf.x[IDX_HEADING] = gnss.heading_deg;
      _initialized = true;
      return;
    }
    const float z[N_MEAS] = {gnss.heading_deg};
    const float hx[N_MEAS] = {_ekf.x[IDX_HEADING]};
    const float R[N_MEAS * N_MEAS] = {gnss.heading_variance_deg2};
    ekf_update(&_ekf, z, hx, H_GNSS_HEADING, R);
  }

  /**
   * @brief Read the current fused estimate.
   *
   * @return Current FusionOutput. heading_variance_deg2 is FLT_MAX before
   * the first valid GNSS update; afterward it is P[IDX_HEADING][IDX_HEADING].
   */
  FusionOutput output() const {
    return FusionOutput{
        .heading_deg = _ekf.x[IDX_HEADING],
        .heading_variance_deg2 =
            _initialized ? _ekf.P[IDX_HEADING * N_STATE + IDX_HEADING]
                         : FLT_MAX,
        .roll_deg = 0.0f,
        .roll_variance_deg2 = FLT_MAX,
        .pitch_deg = 0.0f,
        .pitch_variance_deg2 = FLT_MAX,
        .timestamp = _last_predict_time,
    };
  }

private:
  ekf_t _ekf;
  Config _cfg;
  float _Q[N_STATE * N_STATE];
  Ms _last_predict_time{0};
  bool _initialized = false;
  bool _has_predicted = false;
};

} // namespace fusion
