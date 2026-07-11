"""Tests for the Python wrapper layer over the bound EKF.

Parity tests in test_ekf_parity.py prove the filter math survives the FFI;
these tests prove the Rust-shaped surface (frozen dataclasses, kw-only
construction, no leaked binding types) actually behaves that way.
"""

from __future__ import annotations

from dataclasses import FrozenInstanceError, is_dataclass

import pytest

from plrs_sim import (
    GRAVITY_MS2,
    EkfConfig,
    FusionOutput,
    GnssAttitudeMount,
    GnssSample,
    ImuSample,
    MtiYawConfig,
    TinyEkfFilter,
    Vec3,
    gnss_sample_from_attitude,
)
from plrs_sim.attitude import euler_to_quaternion

CFG = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)


def _imu(rate_z: float, t_ms: int) -> ImuSample:
    return ImuSample(
        angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=rate_z),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        timestamp_ms=t_ms,
    )


def _gnss(heading: float, var: float, t_ms: int) -> GnssSample:
    return GnssSample(
        heading_deg=heading,
        heading_variance_deg2=var,
        timestamp_ms=t_ms,
        valid=True,
    )


def test_imu_sample_is_frozen() -> None:
    s = _imu(0.0, 0)
    with pytest.raises(FrozenInstanceError):
        s.angular_velocity_rad_s = Vec3(x=0.0, y=0.0, z=1.0)  # type: ignore[misc]


def test_gnss_sample_is_frozen() -> None:
    s = _gnss(0.0, 1.0, 0)
    with pytest.raises(FrozenInstanceError):
        s.heading_deg = 90.0  # type: ignore[misc]


def test_output_is_frozen() -> None:
    f = TinyEkfFilter(CFG)
    f.update(_gnss(45.0, 1.0, 1000))
    out = f.output()
    with pytest.raises(FrozenInstanceError):
        out.heading_deg = 0.0  # type: ignore[misc]


def test_config_rejects_positional_args() -> None:
    with pytest.raises(TypeError):
        EkfConfig(0.01, 0.0001, 1000.0, 1.0)  # type: ignore[misc]


def test_imu_sample_rejects_positional_args() -> None:
    zero = Vec3(x=0.0, y=0.0, z=0.0)
    grav = Vec3(x=0.0, y=0.0, z=GRAVITY_MS2)
    with pytest.raises(TypeError):
        ImuSample(zero, grav, 0)  # type: ignore[misc]


def test_output_is_user_facing_dataclass() -> None:
    f = TinyEkfFilter(CFG)
    f.update(_gnss(45.0, 1.0, 1000))
    out = f.output()
    assert isinstance(out, FusionOutput)
    assert is_dataclass(out)


def test_timestamp_round_trips_through_filter() -> None:
    f = TinyEkfFilter(CFG)
    f.update(_gnss(0.0, 1.0, 1000))
    f.predict(_imu(0.0, 1000))
    f.predict(_imu(0.0, 7777))
    assert f.output().timestamp_ms == 7777


def test_input_dataclass_unaffected_by_filter_call() -> None:
    imu = _imu(0.5, 2500)
    f = TinyEkfFilter(CFG)
    f.update(_gnss(0.0, 1.0, 1000))
    f.predict(_imu(0.0, 1000))
    f.predict(imu)
    assert imu.angular_velocity_rad_s.z == 0.5
    assert imu.timestamp_ms == 2500


def test_mti_yaw_config_round_trips_to_filter() -> None:
    """A mag-enabled config engages the MTi yaw measurement through the FFI.

    With no GNSS at all, a filter watching a quaternion whose compass yaw is
    65 deg pulls heading decisively toward it (split with the offset prior),
    while the mag-less filter never moves off 0.
    """
    cfg = EkfConfig(
        q_heading_deg2=0.01,
        q_bias_deg2_s2=0.0001,
        p0_heading_deg2=1000.0,
        p0_bias_deg2_s2=1.0,
        mti_yaw=MtiYawConfig(
            variance_deg2=4.0, q_offset_deg2=0.0001, p0_offset_deg2=100.0
        ),
    )
    # ENU yaw is CCW-positive; -65 reads as 65 on the compass.
    orientation = euler_to_quaternion(0.0, 0.0, -65.0)
    mag = TinyEkfFilter(cfg)
    bare = TinyEkfFilter(CFG)
    for i in range(51):
        imu = ImuSample(
            angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=0.0),
            accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
            orientation=orientation,
            timestamp_ms=1000 + 100 * i,
        )
        mag.predict(imu)
        bare.predict(imu)
    assert mag.output().heading_deg > 40.0
    assert bare.output().heading_deg == pytest.approx(0.0)


def test_bridge_wrapper_subtracts_offset_and_wraps() -> None:
    mount = GnssAttitudeMount(baseline_offset_deg=20.0)
    s = gnss_sample_from_attitude(
        heading_deg=350.0,  # baseline frame; 350 - 20 = 330 -> -30
        heading_variance_deg2=4.0,
        valid=True,
        tow_ms=1000,
        mount=mount,
    )
    assert isinstance(s, GnssSample)
    assert s.valid is True
    assert s.heading_deg == pytest.approx(-30.0)
    assert s.heading_variance_deg2 == pytest.approx(4.0)


def test_bridge_wrapper_dropout_is_invalid() -> None:
    s = gnss_sample_from_attitude(
        heading_deg=45.0,
        heading_variance_deg2=0.0,
        valid=False,
        tow_ms=1000,
        mount=GnssAttitudeMount(),
    )
    assert s.valid is False


def test_debug_exposes_bias_and_offset_states() -> None:
    f = TinyEkfFilter(CFG)
    d = f.debug()
    assert d.gyro_bias_dps == pytest.approx(0.0)
    assert d.gyro_bias_variance_deg2_s2 == pytest.approx(CFG.p0_bias_deg2_s2)
    assert d.mag_offset_variance_deg2 == pytest.approx(0.0)  # mag disabled
    assert d.gate_rejects == 0
    assert is_dataclass(d)
