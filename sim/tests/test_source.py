"""Tests for SimulatedSource."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest

from plrs_sim import (
    ConstantHeel,
    ConstantTurn,
    GnssAttitudeMount,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Static,
)
from plrs_sim.attitude import euler_to_quaternion, multiply, quaternion_to_euler_zyx
from plrs_sim.source import SimulatedSource


def _bare(yaw, duration_s: float = 1.0, seed: int = 0) -> SimulatedSource:
    return SimulatedSource(
        scenario=Scenario(yaw=yaw),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=duration_s,
        seed=seed,
    )


def test_imu_ticks_at_configured_rate() -> None:
    src = _bare(Static(heading_deg=0.0), duration_s=1.0)
    ticks = list(src)
    assert len(ticks) == 101  # 0, 10, 20, ..., 1000 ms inclusive
    assert ticks[0].timestamp_ms == 0
    assert ticks[-1].timestamp_ms == 1000
    assert all(b.timestamp_ms - a.timestamp_ms == 10 for a, b in pairwise(ticks))


def test_gnss_appears_at_configured_cadence() -> None:
    src = _bare(Static(heading_deg=0.0), duration_s=1.0)
    gnss_ts = [t.timestamp_ms for t in src if t.gnss is not None]
    assert gnss_ts == [0, 200, 400, 600, 800, 1000]


def test_truth_heading_tracks_trajectory() -> None:
    src = _bare(ConstantTurn(rate_deg_s=10.0, heading0_deg=0.0), duration_s=0.5)
    for tick in src:
        assert tick.truth_heading_deg == pytest.approx(
            10.0 * tick.timestamp_ms / 1000.0
        )


def test_no_noise_passes_clean_samples() -> None:
    # A CW (compass-positive) turn is a negative ENU rotation about up.
    src = _bare(ConstantTurn(rate_deg_s=180.0 / math.pi), duration_s=0.1)
    for tick in src:
        assert tick.imu.angular_velocity_rad_s.z == pytest.approx(-1.0, rel=1e-4)
        if tick.gnss is not None:
            assert tick.gnss.heading_deg == tick.truth_heading_deg
            assert tick.gnss.heading_variance_deg2 == 0.0
            assert tick.gnss.valid is True


def test_mti_orientation_carries_enu_yaw() -> None:
    src = _bare(ConstantTurn(rate_deg_s=10.0), duration_s=0.5)
    for tick in src:
        _, _, yaw = quaternion_to_euler_zyx(tick.imu.orientation)
        assert yaw == pytest.approx(-tick.truth_heading_deg, abs=1e-6)


def test_iteration_is_replayable() -> None:
    src = SimulatedSource(
        scenario=Scenario(yaw=ConstantTurn(rate_deg_s=5.0)),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=0.01,
            gyro_constant_bias_rad_s=0.005,
            gyro_bias_walk_std_rad_s_sqrt_s=0.001,
        ),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0, dropout_prob=0.1),
        duration_s=0.5,
        seed=42,
    )
    a = list(src)
    b = list(src)
    assert a == b


def test_different_seeds_diverge() -> None:
    cfg = dict(
        scenario=Scenario(yaw=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(gyro_white_std_rad_s=0.01),
        gnss_noise=GnssNoiseModel(),
        duration_s=0.5,
    )
    a = list(SimulatedSource(seed=1, **cfg))
    b = list(SimulatedSource(seed=2, **cfg))
    assert any(x.imu != y.imu for x, y in zip(a, b, strict=True))


def test_custom_rates_respected() -> None:
    src = SimulatedSource(
        scenario=Scenario(yaw=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=1.0,
        seed=0,
        imu_rate_hz=50.0,
        gnss_rate_hz=10.0,
    )
    ticks = list(src)
    assert ticks[1].timestamp_ms - ticks[0].timestamp_ms == 20
    gnss_ts = [t.timestamp_ms for t in ticks if t.gnss is not None]
    assert gnss_ts[1] - gnss_ts[0] == 100


def test_baseline_offset_round_trips_to_boat_heading() -> None:
    # A nonzero mount offset is added in the baseline frame and removed by the
    # bridge, so the GNSS heading the filter sees is boat-forward truth again.
    src = SimulatedSource(
        scenario=Scenario(yaw=ConstantTurn(rate_deg_s=10.0)),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=0.5,
        seed=0,
        mount=GnssAttitudeMount(baseline_offset_deg=30.0),
    )
    for tick in src:
        if tick.gnss is not None:
            assert tick.gnss.heading_deg == pytest.approx(tick.truth_heading_deg)


def test_dropout_emits_an_invalid_sample() -> None:
    src = SimulatedSource(
        scenario=Scenario(yaw=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(dropout_prob=1.0),
        duration_s=0.5,
        seed=0,
    )
    scheduled = [t.gnss for t in src if t.gnss is not None]
    assert scheduled  # samples are still scheduled, just invalid
    assert all(not g.valid for g in scheduled)


def test_imu_mount_tilts_the_reported_orientation() -> None:
    # An 8 deg roll mount: the MTi reports the boat attitude pre-rotated by the
    # inverse mount, so re-applying the mount (as the filter does) recovers it.
    mount = euler_to_quaternion(8.0, 0.0, 0.0)
    src = SimulatedSource(
        scenario=Scenario(
            yaw=Static(heading_deg=0.0), attitude=ConstantHeel(angle_deg=20.0)
        ),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=0.1,
        seed=0,
        imu_mount=mount,
    )
    tick = next(iter(src))
    recovered_roll, recovered_pitch, _ = quaternion_to_euler_zyx(
        multiply(tick.imu.orientation, mount)
    )
    assert recovered_roll == pytest.approx(tick.truth_roll_deg, abs=1e-6)
    assert recovered_pitch == pytest.approx(tick.truth_pitch_deg, abs=1e-6)
    # The raw MTi reading is tilted away from the true boat roll.
    raw_roll, _, _ = quaternion_to_euler_zyx(tick.imu.orientation)
    assert abs(raw_roll - tick.truth_roll_deg) > 1.0
