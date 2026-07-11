"""Tests for the runner: source -> EKF -> Trace."""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim import (
    ConstantHeel,
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Sinusoidal,
    Static,
    load_tuning,
)
from plrs_sim.attitude import euler_to_quaternion
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

CFG = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)


def _src(
    yaw,
    *,
    imu_noise: ImuNoiseModel | None = None,
    gnss_noise: GnssNoiseModel | None = None,
    duration_s: float = 5.0,
    seed: int = 0,
) -> SimulatedSource:
    return SimulatedSource(
        scenario=Scenario(yaw=yaw),
        imu_noise=imu_noise or ImuNoiseModel(),
        gnss_noise=gnss_noise or GnssNoiseModel(),
        duration_s=duration_s,
        seed=seed,
    )


def _wrapped_residual(ch) -> np.ndarray:
    """Estimate-minus-truth, the short way around the +-180 seam."""
    return (ch.estimate - ch.truth + 180.0) % 360.0 - 180.0


def test_trace_arrays_have_matching_length() -> None:
    trace = run(_src(Static(heading_deg=0.0), duration_s=1.0), CFG)
    n = len(trace.t_ms)
    assert n == 101
    assert len(trace.channels["heading"].truth) == n
    assert len(trace.channels["heading"].estimate) == n
    assert len(trace.channels["heading"].estimate_std) == n
    assert len(trace.channels["heading"].openloop) == n


def test_gnss_arrays_match_emitted_count() -> None:
    src = _src(Static(heading_deg=0.0), duration_s=1.0)
    trace = run(src, CFG)
    expected = sum(1 for tick in src if tick.gnss is not None)
    assert len(trace.channels["heading"].measurement_t_ms) == expected
    assert len(trace.channels["heading"].measurement) == expected


def test_timestamps_are_monotone() -> None:
    trace = run(_src(Static(heading_deg=0.0), duration_s=1.0), CFG)
    assert np.all(np.diff(trace.t_ms) > 0)


def test_heading_channel_is_marked_circular() -> None:
    trace = run(_src(Static(heading_deg=0.0), duration_s=1.0), CFG)
    assert trace.channels["heading"].wrap is True
    assert trace.channels["roll"].wrap is False


def test_filter_tracks_heading_across_the_180_seam() -> None:
    # Turn from 0 through +180 and out the other side. The estimate stays on
    # truth the whole way: the wrapped residual never blows up at the seam.
    src = _src(
        ConstantTurn(rate_deg_s=10.0),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
        duration_s=40.0,
    )
    ch = run(src, CFG).channels["heading"]
    residual = _wrapped_residual(ch)
    # Skip the cold-start transient; after convergence the error is small and,
    # crucially, has no spike where truth crosses 180 deg (~36 s in).
    steady = residual[len(residual) // 4 :]
    assert np.max(np.abs(steady)) < 2.0


def test_filter_recovers_attitude_through_a_tilted_mount() -> None:
    # The IMU is bolted 8 deg out of square; the boat sits at 20 deg heel.
    # With the matching mount in its config, the filter un-rotates the MTi
    # quaternion and the fused roll tracks the true heel, not the tilted read.
    cfg = EkfConfig(
        q_heading_deg2=0.01,
        q_bias_deg2_s2=0.0001,
        p0_heading_deg2=1000.0,
        p0_bias_deg2_s2=1.0,
        mount_roll_deg=8.0,
    )
    src = SimulatedSource(
        scenario=Scenario(
            yaw=Static(heading_deg=0.0), attitude=ConstantHeel(angle_deg=20.0)
        ),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=2.0,
        seed=0,
        imu_mount=euler_to_quaternion(8.0, 0.0, 0.0),
    )
    ch = run(src, cfg).channels["roll"]
    assert ch.truth[-1] == pytest.approx(20.0, abs=1e-6)
    assert ch.estimate[-1] == pytest.approx(20.0, abs=0.5)


def test_est_std_finite_after_first_gnss() -> None:
    trace = run(_src(Static(heading_deg=45.0), duration_s=1.0), CFG)
    # GNSS at t=0 seeds the filter; std should be finite from index 0.
    assert np.all(np.isfinite(trace.channels["heading"].estimate_std))


def test_estimate_converges_to_truth_without_noise() -> None:
    trace = run(
        _src(ConstantTurn(rate_deg_s=5.0), duration_s=10.0),
        CFG,
    )
    final_error = abs(
        trace.channels["heading"].estimate[-1] - trace.channels["heading"].truth[-1]
    )
    assert final_error < 0.1


def test_tracks_through_realistic_noise_with_mag_aiding() -> None:
    # The docs-image scenario under the shipped tuning.toml: full noise
    # model, gate and mag yaw active. Guards the sim/firmware convention
    # agreement; a sign flip on either side turns this into a
    # tens-of-degrees staircase, not a small miss.
    src = _src(
        Sinusoidal(amplitude_deg=30.0, period_s=20.0),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=0.01,
            gyro_constant_bias_rad_s=0.005,
            gyro_bias_walk_std_rad_s_sqrt_s=0.001,
            mti_attitude_std_deg=1.0,
        ),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
        duration_s=10.0,
    )
    ch = run(src, load_tuning()).channels["heading"]
    residual = _wrapped_residual(ch)
    assert math.sqrt(np.mean(residual**2)) < 2.0


def test_openloop_tracks_truth_without_noise() -> None:
    # With a clean gyro the dead-reckon must equal truth, not its mirror.
    trace = run(_src(ConstantTurn(rate_deg_s=10.0), duration_s=5.0), CFG)
    ch = trace.channels["heading"]
    assert ch.openloop[-1] == pytest.approx(ch.truth[-1], abs=0.1)


def test_openloop_drifts_with_gyro_bias() -> None:
    bias = 0.05  # rad/s ~= 2.86 deg/s
    trace = run(
        _src(
            Static(heading_deg=0.0),
            imu_noise=ImuNoiseModel(gyro_constant_bias_rad_s=bias),
            duration_s=5.0,
        ),
        CFG,
    )
    # A +bias on the ENU gyro dead-reckons compass heading negative.
    expected_drift = -bias * (180.0 / math.pi) * 5.0
    assert abs(trace.channels["heading"].openloop[-1] - expected_drift) < 1.0


def test_ekf_cancels_bias_better_than_openloop() -> None:
    bias = 0.05
    trace = run(
        _src(
            Static(heading_deg=0.0),
            imu_noise=ImuNoiseModel(gyro_constant_bias_rad_s=bias),
            gnss_noise=GnssNoiseModel(heading_std_deg=0.5),
            duration_s=10.0,
            seed=3,
        ),
        CFG,
    )
    ekf_error = abs(
        trace.channels["heading"].estimate[-1] - trace.channels["heading"].truth[-1]
    )
    openloop_error = abs(
        trace.channels["heading"].openloop[-1] - trace.channels["heading"].truth[-1]
    )
    assert ekf_error < openloop_error / 5.0


def test_run_is_deterministic_for_same_seed() -> None:
    a = run(_src(ConstantTurn(rate_deg_s=10.0), duration_s=1.0, seed=7), CFG)
    b = run(_src(ConstantTurn(rate_deg_s=10.0), duration_s=1.0, seed=7), CFG)
    assert np.array_equal(
        a.channels["heading"].estimate, b.channels["heading"].estimate
    )
    assert np.array_equal(
        a.channels["heading"].measurement, b.channels["heading"].measurement
    )


def test_openloop_nan_before_first_gnss() -> None:
    # GnssNoiseModel with dropout_prob=1.0 means no GNSS will ever fire.
    trace = run(
        _src(
            Static(heading_deg=0.0),
            gnss_noise=GnssNoiseModel(dropout_prob=1.0),
            duration_s=0.1,
        ),
        CFG,
    )
    assert pytest.approx(0, abs=0) == np.sum(
        np.isfinite(trace.channels["heading"].openloop)
    )
