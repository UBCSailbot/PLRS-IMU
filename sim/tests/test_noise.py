"""Tests for sensor noise corruption.

Statistical assertions use large N and loose bounds so they don't flake;
the precision of np.random is not under test here.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim import (
    GRAVITY_MS2,
    GnssNoiseModel,
    GnssSample,
    ImuNoiseModel,
    ImuSample,
    Vec3,
)
from plrs_sim.noise import GnssNoise, ImuNoise


def _imu(rate_z: float = 0.5) -> ImuSample:
    return ImuSample(
        angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=rate_z),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        timestamp_ms=0,
    )


def _gnss(heading: float = 45.0) -> GnssSample:
    return GnssSample(
        heading_deg=heading,
        heading_variance_deg2=0.0,
        timestamp_ms=0,
        valid=True,
    )


def test_imu_no_model_returns_input() -> None:
    n = ImuNoise(ImuNoiseModel(), np.random.default_rng(0))
    clean = _imu(0.5)
    out = n.corrupt(clean, dt_s=0.01)
    assert out == clean


def test_imu_constant_bias_added_each_call() -> None:
    n = ImuNoise(ImuNoiseModel(gyro_constant_bias_rad_s=0.1), np.random.default_rng(0))
    for _ in range(5):
        out = n.corrupt(_imu(0.5), dt_s=0.01)
        assert out.angular_velocity_rad_s.z == pytest.approx(0.6)


def test_imu_white_noise_mean_and_std() -> None:
    std = 0.05
    n = ImuNoise(ImuNoiseModel(gyro_white_std_rad_s=std), np.random.default_rng(42))
    samples = np.array(
        [
            n.corrupt(_imu(0.0), dt_s=0.01).angular_velocity_rad_s.z
            for _ in range(10_000)
        ]
    )
    assert abs(samples.mean()) < 0.005
    assert abs(samples.std() - std) / std < 0.05


def test_imu_bias_random_walk_grows_with_time() -> None:
    seed = 7
    walk = 0.01
    cfg = ImuNoiseModel(gyro_bias_walk_std_rad_s_sqrt_s=walk)

    finals = []
    for trial_seed in range(seed, seed + 200):
        n = ImuNoise(cfg, np.random.default_rng(trial_seed))
        for _ in range(1000):
            n.corrupt(_imu(0.0), dt_s=0.01)
        finals.append(n.bias_rad_s)

    expected_std = walk * math.sqrt(0.01 * 1000)
    observed_std = float(np.std(finals))
    assert abs(observed_std - expected_std) / expected_std < 0.15


def test_imu_same_seed_same_output() -> None:
    cfg = ImuNoiseModel(
        gyro_white_std_rad_s=0.01,
        gyro_constant_bias_rad_s=0.005,
        gyro_bias_walk_std_rad_s_sqrt_s=0.001,
    )
    a = ImuNoise(cfg, np.random.default_rng(123))
    b = ImuNoise(cfg, np.random.default_rng(123))
    for _ in range(50):
        assert a.corrupt(_imu(0.3), dt_s=0.01) == b.corrupt(_imu(0.3), dt_s=0.01)


def test_gnss_no_model_returns_input() -> None:
    n = GnssNoise(GnssNoiseModel(), np.random.default_rng(0))
    clean = _gnss(45.0)
    assert n.corrupt(clean) == clean


def test_gnss_noise_sets_variance_to_std_squared() -> None:
    std = 1.5
    n = GnssNoise(GnssNoiseModel(heading_std_deg=std), np.random.default_rng(0))
    out = n.corrupt(_gnss(45.0))
    assert out is not None
    assert out.heading_variance_deg2 == pytest.approx(std * std)


def test_gnss_heading_noise_mean_and_std() -> None:
    std = 1.5
    n = GnssNoise(GnssNoiseModel(heading_std_deg=std), np.random.default_rng(11))
    deltas = []
    for _ in range(10_000):
        out = n.corrupt(_gnss(0.0))
        assert out is not None
        deltas.append(out.heading_deg)
    arr = np.array(deltas)
    assert abs(arr.mean()) < 0.1
    assert abs(arr.std() - std) / std < 0.05


def test_gnss_dropout_rate_matches_config() -> None:
    p = 0.25
    n = GnssNoise(GnssNoiseModel(dropout_prob=p), np.random.default_rng(99))
    drops = sum(n.corrupt(_gnss(0.0)) is None for _ in range(10_000))
    observed = drops / 10_000
    assert abs(observed - p) < 0.02


def test_gnss_same_seed_same_output() -> None:
    cfg = GnssNoiseModel(heading_std_deg=2.0, dropout_prob=0.1)
    a = GnssNoise(cfg, np.random.default_rng(321))
    b = GnssNoise(cfg, np.random.default_rng(321))
    for _ in range(50):
        assert a.corrupt(_gnss(30.0)) == b.corrupt(_gnss(30.0))
