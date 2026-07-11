/**
 * Five-state EKF for heading + roll + pitch + gyro_z bias + mag offset.
 *
 * Satisfies the FusionFilter concept. State: x[IDX_HEADING] = heading (deg),
 * x[IDX_ROLL] = roll (deg), x[IDX_PITCH] = pitch (deg), x[IDX_GYRO_BIAS] =
 * gyro_z bias (deg/s), x[IDX_MAG_OFFSET] = MTi-yaw-to-heading offset (deg).
 * Roll/pitch/yaw propagate from the body gyro through the ZYX Euler
 * kinematics; the MTi quaternion is a roll/pitch measurement and the GNSS fix
 * is the heading measurement, both applied as scalar updates.
 *
 * When Config::mti_yaw is set, the MTi yaw is a third measurement, of
 * heading + mag_offset. The offset state absorbs whatever separates the MTi's
 * magnetic yaw from GNSS heading (declination, boat iron, the ENU-to-compass
 * frame constant), so the mag stiffens heading between fixes and keeps the
 * gyro bias observable through an outage, but can never pull absolute heading
 * against GNSS.
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
#include <cstdint>
#include <optional>

#define EKF_N 5
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
constexpr std::size_t IDX_MAG_OFFSET = 4;

/**
 * Observation Jacobians for the scalar updates. The GNSS fix reads heading
 * directly; roll and pitch are read from the MTi quaternion; the MTi yaw
 * reads heading + mag_offset, which is what makes the offset observable only
 * when GNSS is also present.
 */
constexpr float H_HEADING[N_STATE] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
constexpr float H_ROLL[N_STATE] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
constexpr float H_PITCH[N_STATE] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
constexpr float H_MAG_YAW[N_STATE] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

class TinyEkfFilter {
public:
  /**
   * Reject a heading fix whose innovation exceeds this many sigma of the
   * combined state + measurement uncertainty (chi-square gate, 1 dof).
   */
  static constexpr float HEADING_GATE_SIGMA = 3.0f;

  /**
   * Consecutive gated fixes after which one is accepted anyway, so a filter
   * converged on a wrong heading cannot reject reacquisition forever.
   */
  static constexpr uint32_t HEADING_GATE_LIMIT = 10;

  /**
   * Same chi-square gate for the MTi yaw measurement. The mag is a
   * stabilizer, never the heading authority: a snap or re-convergence in the
   * MTi's mag-referenced yaw (docs/internal/heading_drift.md) must not steer
   * heading, so its forced-accept horizon is long (30 s at 100 Hz) where the
   * GNSS one is short.
   */
  static constexpr float MTI_YAW_GATE_SIGMA = 3.0f;
  static constexpr uint32_t MTI_YAW_GATE_LIMIT = 3000;

  /**
   * MTi yaw (magnetometer-referenced) heading aiding. variance_deg2 is the
   * per-sample measurement noise; q_offset_deg2 and p0_offset_deg2 shape the
   * mag-offset state (slow iron/declination wander, and how much the mag yaw
   * is trusted as absolute heading before the first GNSS fix).
   */
  struct MtiYawConfig {
    float variance_deg2;
    float q_offset_deg2;
    float p0_offset_deg2;
  };

  /**
   * Filter tuning parameters. See docs/tuning.md. mti_yaw disengages the mag
   * measurement entirely when unset; the filter is then numerically identical
   * to the four-state version.
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
    std::optional<MtiYawConfig> mti_yaw {};
    MountRotation mount {};
  };

  /**
   * @brief Construct with tuning parameters.
   *
   * @param cfg. Filter tuning parameters.
   */
  explicit TinyEkfFilter(Config cfg) : _cfg(cfg) {
    _Q[IDX_HEADING * N_STATE + IDX_HEADING] = cfg.q_heading_deg2;
    _Q[IDX_ROLL * N_STATE + IDX_ROLL] = cfg.q_roll_deg2;
    _Q[IDX_PITCH * N_STATE + IDX_PITCH] = cfg.q_pitch_deg2;
    _Q[IDX_GYRO_BIAS * N_STATE + IDX_GYRO_BIAS] = cfg.q_bias_deg2_s2;
    _Q[IDX_MAG_OFFSET * N_STATE + IDX_MAG_OFFSET] =
        cfg.mti_yaw ? cfg.mti_yaw->q_offset_deg2 : 0.0f;
    const float pdiag[N_STATE] = {
        _cfg.p0_heading_deg2,
        _cfg.p0_roll_deg2,
        _cfg.p0_pitch_deg2,
        _cfg.p0_bias_deg2_s2,
        _cfg.mti_yaw ? _cfg.mti_yaw->p0_offset_deg2 : 0.0f,
    };
    ekf_initialize(&_ekf, pdiag);
  }

  /**
   * @brief Advance the filter with the gyro, then correct roll/pitch from the
   * MTi quaternion and, when Config::mti_yaw is set, heading + mag offset
   * from the MTi yaw. Called at IMU rate.
   *
   * The first call sets the timestamp baseline and seeds roll/pitch from the
   * MTi quaternion without advancing state.
   *
   * @param imu. IMU sample.
   */
  void predict(ImuSample imu) {
    const EulerZyx attitude = measured_attitude(imu.orientation);

    // Pre-filter values off the MTi orientation, for the raw rudder link. Roll
    // is the measured boat-frame roll; the yaw rate uses the same heel-aware
    // ZYX mapping as the fused path but reads the measured angles and skips the
    // gyro-bias term, so it owes nothing to the EKF state. Compass sign (CW
    // positive), matching yaw_rate_dps; see docs/attitude.md.
    _raw_roll_deg = attitude.roll_deg;
    _raw_yaw_rate_dps = -euler_rates_zyx(attitude.roll_deg * DEG_TO_RAD,
                                         attitude.pitch_deg * DEG_TO_RAD,
                                         imu.angular_velocity_rad_s)
                             .yaw_dot *
                        RAD_TO_DEG;

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
    _yaw_rate_dps = -rates.yaw_dot * RAD_TO_DEG - _ekf.x[IDX_GYRO_BIAS];
    const EulerRatesJacobian jac =
        euler_rates_jacobian(roll_rad, pitch_rad, imu.angular_velocity_rad_s);

    // ENU yaw is CCW-positive; compass heading is CW-positive. Negate the ENU
    // yaw rate so the heading state stays in compass convention. See
    // docs/attitude.md.
    const float fx[N_STATE] = {
        _ekf.x[IDX_HEADING] +
            (-rates.yaw_dot * RAD_TO_DEG - _ekf.x[IDX_GYRO_BIAS]) * dt_s,
        _ekf.x[IDX_ROLL] + rates.roll_dot * RAD_TO_DEG * dt_s,
        _ekf.x[IDX_PITCH] + rates.pitch_dot * RAD_TO_DEG * dt_s,
        _ekf.x[IDX_GYRO_BIAS],
        _ekf.x[IDX_MAG_OFFSET],
    };

    // F = d fx / d x: identity plus the kinematic couplings. Euler rates
    // depend on roll/pitch but not heading or the mag offset; gyro bias
    // couples only into heading. The per-radian Jacobian entries times dt
    // land in degree space: the RAD_TO_DEG and DEG_TO_RAD conversions cancel.
    float F[N_STATE * N_STATE] {};
    for (std::size_t i = 0; i < N_STATE; i++) {
      F[i * N_STATE + i] = 1.0f;
    }
    F[IDX_HEADING * N_STATE + IDX_ROLL] = -jac.dyaw_droll * dt_s;
    F[IDX_HEADING * N_STATE + IDX_PITCH] = -jac.dyaw_dpitch * dt_s;
    F[IDX_HEADING * N_STATE + IDX_GYRO_BIAS] = -dt_s;
    F[IDX_ROLL * N_STATE + IDX_ROLL] += jac.droll_droll * dt_s;
    F[IDX_ROLL * N_STATE + IDX_PITCH] = jac.droll_dpitch * dt_s;
    F[IDX_PITCH * N_STATE + IDX_ROLL] = jac.dpitch_droll * dt_s;
    F[IDX_PITCH * N_STATE + IDX_PITCH] += jac.dpitch_dpitch * dt_s;

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

    if (_cfg.mti_yaw) {
      // ENU yaw negated into compass sign (see docs/attitude.md); the offset
      // state absorbs the remaining frame constant along with declination and
      // iron, so only the sign has to be right here.
      const float hx = _ekf.x[IDX_HEADING] + _ekf.x[IDX_MAG_OFFSET];
      const float innovation = wrap180(-attitude.yaw_deg - hx);
      const float s = _ekf.P[IDX_HEADING * N_STATE + IDX_HEADING] +
                      _ekf.P[IDX_HEADING * N_STATE + IDX_MAG_OFFSET] +
                      _ekf.P[IDX_MAG_OFFSET * N_STATE + IDX_HEADING] +
                      _ekf.P[IDX_MAG_OFFSET * N_STATE + IDX_MAG_OFFSET] +
                      _cfg.mti_yaw->variance_deg2;
      if (innovation * innovation >
          MTI_YAW_GATE_SIGMA * MTI_YAW_GATE_SIGMA * s) {
        _mag_gate_rejects++;
        if (_mag_gate_rejects < MTI_YAW_GATE_LIMIT) {
          return;
        }
      }
      _mag_gate_rejects = 0;
      scalar_update(
          H_MAG_YAW, hx, hx + innovation, _cfg.mti_yaw->variance_deg2);
      _ekf.x[IDX_HEADING] = wrap180(_ekf.x[IDX_HEADING]);
    }
  }

  /**
   * @brief Correct the heading estimate from a GNSS measurement.
   *
   * No-op if !gnss.valid. The first valid sample seeds x[IDX_HEADING]
   * directly and marks the filter initialized; subsequent valid samples run a
   * scalar update with the innovation wrapped to +-180 so a fix across the
   * +-180 seam does not drag heading the long way around.
   *
   * A fix whose innovation fails the HEADING_GATE_SIGMA gate is dropped, so a
   * lone bad fix cannot slam the state (and, through the covariance cross
   * terms, the gyro-bias estimate). After HEADING_GATE_LIMIT consecutive
   * rejections the fix is accepted: persistent disagreement means the filter,
   * not the receiver, is wrong.
   *
   * @param gnss. GNSS sample.
   */
  void update(GnssSample gnss) {
    if (!gnss.valid) {
      return;
    }
    if (!_initialized) {
      // Shift the mag offset by the seed jump so heading + offset is
      // unchanged and the next MTi yaw sample does not fight the seed.
      const float delta = wrap180(gnss.heading_deg - _ekf.x[IDX_HEADING]);
      _ekf.x[IDX_HEADING] = wrap180(gnss.heading_deg);
      _ekf.x[IDX_MAG_OFFSET] = wrap180(_ekf.x[IDX_MAG_OFFSET] - delta);
      _initialized = true;
      return;
    }
    const float hx = _ekf.x[IDX_HEADING];
    const float innovation = wrap180(gnss.heading_deg - hx);
    const float s = _ekf.P[IDX_HEADING * N_STATE + IDX_HEADING] +
                    gnss.heading_variance_deg2;
    if (innovation * innovation > HEADING_GATE_SIGMA * HEADING_GATE_SIGMA * s) {
      _gate_rejects++;
      if (_gate_rejects < HEADING_GATE_LIMIT) {
        return;
      }
    }
    _gate_rejects = 0;
    scalar_update(H_HEADING, hx, hx + innovation, gnss.heading_variance_deg2);
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
        .yaw_rate_dps = _yaw_rate_dps,
        .raw_roll_deg = _raw_roll_deg,
        .raw_yaw_rate_dps = _raw_yaw_rate_dps,
    };
  }

  /**
   * Internal state usually hidden behind FusionOutput, for telemetry. A
   * healthy filter keeps the bias within its prior (see tuning.toml); a bias
   * far outside it with GNSS absent is the drift signature described in
   * docs/internal/heading_drift.md.
   */
  struct Debug {
    float gyro_bias_dps;
    float gyro_bias_variance_deg2_s2;
    float mag_offset_deg;
    float mag_offset_variance_deg2;
    uint32_t gate_rejects;
    uint32_t mag_gate_rejects;
  };

  /**
   * @brief Read the bias/offset states and their variances.
   *
   * @return Current Debug snapshot.
   */
  Debug debug() const {
    return Debug {
        .gyro_bias_dps = _ekf.x[IDX_GYRO_BIAS],
        .gyro_bias_variance_deg2_s2 =
            _ekf.P[IDX_GYRO_BIAS * N_STATE + IDX_GYRO_BIAS],
        .mag_offset_deg = _ekf.x[IDX_MAG_OFFSET],
        .mag_offset_variance_deg2 =
            _ekf.P[IDX_MAG_OFFSET * N_STATE + IDX_MAG_OFFSET],
        .gate_rejects = _gate_rejects,
        .mag_gate_rejects = _mag_gate_rejects,
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
    symmetrize_covariance();
  }

  /**
   * @brief Restore P = P^T after an update.
   *
   * TinyEKF's `(I-GH)P` update form lets float32 roundoff skew the cross
   * terms over the ~300 updates/s this filter runs; averaging the halves
   * keeps the gates and gains honest over long runs.
   */
  void symmetrize_covariance() {
    for (std::size_t i = 0; i < N_STATE; i++) {
      for (std::size_t j = i + 1; j < N_STATE; j++) {
        const float mean =
            0.5f * (_ekf.P[i * N_STATE + j] + _ekf.P[j * N_STATE + i]);
        _ekf.P[i * N_STATE + j] = mean;
        _ekf.P[j * N_STATE + i] = mean;
      }
    }
  }

  ekf_t _ekf;
  Config _cfg;
  float _Q[N_STATE * N_STATE] {};
  Ms _last_predict_time {0};
  float _yaw_rate_dps = 0.0f;
  float _raw_roll_deg = 0.0f;
  float _raw_yaw_rate_dps = 0.0f;
  uint32_t _gate_rejects = 0;
  uint32_t _mag_gate_rejects = 0;
  bool _initialized = false;
  bool _has_predicted = false;
};

} // namespace fusion
