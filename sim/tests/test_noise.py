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
    MagNoiseModel,
    Vec3,
)
from plrs_sim.attitude import quaternion_to_euler_zyx
from plrs_sim.noise import GnssNoise, ImuNoise


def _imu(rate_z: float = 0.5) -> ImuSample:
    return ImuSample(
        angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=rate_z),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        timestamp_ms=0,
    )


def _gnss(heading: float = 45.0, timestamp_ms: int = 0) -> GnssSample:
    return GnssSample(
        heading_deg=heading,
        heading_variance_deg2=0.0,
        timestamp_ms=timestamp_ms,
        valid=True,
    )


def test_imu_no_model_returns_input() -> None:
    n = ImuNoise(ImuNoiseModel(), np.random.default_rng(0))
    clean = _imu(0.5)
    out = n.corrupt(clean, dt_s=0.01)
    assert out == clean


def test_imu_constant_bias_added_each_call() -> None:
    n = ImuNoise(
        ImuNoiseModel(gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.1)),
        np.random.default_rng(0),
    )
    for _ in range(5):
        out = n.corrupt(_imu(0.5), dt_s=0.01)
        assert out.angular_velocity_rad_s.z == pytest.approx(0.6)


def test_imu_constant_bias_is_three_axis() -> None:
    # A body-frame turn-on bias corrupts all three gyro axes, not just z; this
    # is what lets a heeled X/Y bias project into heading (see ImuNoiseModel).
    n = ImuNoise(
        ImuNoiseModel(gyro_constant_bias_rad_s=Vec3(x=0.02, y=-0.03, z=0.1)),
        np.random.default_rng(0),
    )
    clean = ImuSample(
        angular_velocity_rad_s=Vec3(x=0.5, y=0.5, z=0.5),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        timestamp_ms=0,
    )
    out = n.corrupt(clean, dt_s=0.01)
    assert out.angular_velocity_rad_s.x == pytest.approx(0.52)
    assert out.angular_velocity_rad_s.y == pytest.approx(0.47)
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


def test_imu_no_attitude_noise_leaves_orientation_identity() -> None:
    n = ImuNoise(ImuNoiseModel(), np.random.default_rng(0))
    o = n.corrupt(_imu(0.0), dt_s=0.01).orientation
    assert (o.w, o.x, o.y, o.z) == (1.0, 0.0, 0.0, 0.0)


def test_imu_attitude_noise_perturbs_roll_and_pitch() -> None:
    std = 1.0
    n = ImuNoise(ImuNoiseModel(mti_attitude_std_deg=std), np.random.default_rng(5))
    rolls, pitches = [], []
    for _ in range(10_000):
        roll, pitch, _ = quaternion_to_euler_zyx(
            n.corrupt(_imu(0.0), dt_s=0.01).orientation
        )
        rolls.append(roll)
        pitches.append(pitch)
    assert abs(float(np.mean(rolls))) < 0.1
    assert abs(float(np.std(rolls)) - std) / std < 0.1
    assert abs(float(np.std(pitches)) - std) / std < 0.1


def test_imu_same_seed_same_output() -> None:
    cfg = ImuNoiseModel(
        gyro_white_std_rad_s=0.01,
        gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.005),
        gyro_bias_walk_std_rad_s_sqrt_s=0.001,
        mti_attitude_std_deg=1.0,
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


def test_gnss_outage_window_drops_fixes_in_range() -> None:
    # A sustained outage nulls every fix while start <= t < end, and passes
    # fixes on either side through untouched.
    n = GnssNoise(
        GnssNoiseModel(outage_start_s=10.0, outage_end_s=40.0),
        np.random.default_rng(0),
    )
    assert n.corrupt(_gnss(timestamp_ms=9_000)) is not None
    assert n.corrupt(_gnss(timestamp_ms=10_000)) is None
    assert n.corrupt(_gnss(timestamp_ms=39_000)) is None
    assert n.corrupt(_gnss(timestamp_ms=40_000)) is not None


def test_gnss_outage_without_end_never_recovers() -> None:
    # outage_end_s=None models a dropout that stays out for the rest of the run.
    n = GnssNoise(GnssNoiseModel(outage_start_s=30.0), np.random.default_rng(0))
    assert n.corrupt(_gnss(timestamp_ms=29_000)) is not None
    assert n.corrupt(_gnss(timestamp_ms=30_000)) is None
    assert n.corrupt(_gnss(timestamp_ms=600_000)) is None


def test_gnss_same_seed_same_output() -> None:
    cfg = GnssNoiseModel(heading_std_deg=2.0, dropout_prob=0.1)
    a = GnssNoise(cfg, np.random.default_rng(321))
    b = GnssNoise(cfg, np.random.default_rng(321))
    for _ in range(50):
        assert a.corrupt(_gnss(30.0)) == b.corrupt(_gnss(30.0))


def _imu_at_yaw(yaw_deg: float) -> ImuSample:
    from plrs_sim.attitude import euler_to_quaternion

    return ImuSample(
        angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=0.0),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        orientation=euler_to_quaternion(0.0, 0.0, yaw_deg),
        timestamp_ms=0,
    )


def test_mag_none_leaves_orientation_untouched() -> None:
    n = ImuNoise(ImuNoiseModel(), np.random.default_rng(0))
    clean = _imu_at_yaw(30.0)
    assert n.corrupt(clean, dt_s=0.01).orientation == clean.orientation


def test_mag_iron_error_depends_on_orientation() -> None:
    n = ImuNoise(
        ImuNoiseModel(mag=MagNoiseModel(iron_deg=30.0)),
        np.random.default_rng(0),
    )
    errs = []
    for yaw in (0.0, 90.0, 180.0, -90.0):
        out = n.corrupt(_imu_at_yaw(yaw), dt_s=0.01)
        roll, pitch, out_yaw = quaternion_to_euler_zyx(out.orientation)
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert pitch == pytest.approx(0.0, abs=1e-6)
        err = (out_yaw - yaw + 180.0) % 360.0 - 180.0
        assert abs(err) <= 30.0 + 1e-6
        errs.append(err)
    # A hard-iron lobe varies with heading; opposite headings flip its sign.
    assert errs[0] == pytest.approx(-errs[2], abs=1e-6)
    assert max(errs) - min(errs) > 1.0


def test_mag_iron_leaves_gyro_truthful() -> None:
    n = ImuNoise(
        ImuNoiseModel(mag=MagNoiseModel(iron_deg=30.0)),
        np.random.default_rng(0),
    )
    out = n.corrupt(_imu(0.5), dt_s=0.01)
    assert out.angular_velocity_rad_s.z == pytest.approx(0.5)


def test_mag_snap_steps_then_decays() -> None:
    tau = 5.0
    n = ImuNoise(
        ImuNoiseModel(mag=MagNoiseModel(snap_deg=40.0, snap_tau_s=tau)),
        np.random.default_rng(3),
    )
    n._mag_snap_err_deg = 40.0  # injected snap; interval left long so no more
    for _ in range(500):  # 5 s at 100 Hz = one time constant
        out = n.corrupt(_imu_at_yaw(0.0), dt_s=0.01)
    _, _, yaw = quaternion_to_euler_zyx(out.orientation)
    assert yaw == pytest.approx(40.0 * math.exp(-1.0), rel=0.05)


def test_mag_snap_fires_at_configured_rate() -> None:
    n = ImuNoise(
        ImuNoiseModel(mag=MagNoiseModel(snap_deg=40.0, snap_interval_s=0.001)),
        np.random.default_rng(4),
    )
    out = n.corrupt(_imu_at_yaw(0.0), dt_s=0.01)  # fires with certainty
    _, _, yaw = quaternion_to_euler_zyx(out.orientation)
    assert yaw != pytest.approx(0.0, abs=1e-3)
    assert abs(yaw) <= 40.0
