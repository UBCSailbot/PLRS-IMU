"""Tests for the runner: source -> EKF -> Trace."""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim import (
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Static,
)
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
    assert final_error < 0.5


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
    expected_drift = bias * (180.0 / math.pi) * 5.0
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
