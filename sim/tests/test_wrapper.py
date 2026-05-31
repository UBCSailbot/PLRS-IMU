"""Tests for the Python wrapper layer over the bound EKF.

Parity tests in test_ekf_parity.py prove the filter math survives the FFI;
these tests prove the Rust-shaped surface (frozen dataclasses, kw-only
construction, no leaked binding types) actually behaves that way.
"""

from __future__ import annotations

from dataclasses import FrozenInstanceError, is_dataclass

import pytest

from plrs_sim import EkfConfig, FusionOutput, GnssSample, ImuSample, TinyEkfFilter

CFG = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)


def _imu(rate_z: float, t_ms: int) -> ImuSample:
    return ImuSample(
        rate_of_turn_x_rad_s=0.0,
        rate_of_turn_y_rad_s=0.0,
        rate_of_turn_z_rad_s=rate_z,
        accel_x_ms2=0.0,
        accel_y_ms2=0.0,
        accel_z_ms2=9.81,
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
        s.rate_of_turn_z_rad_s = 1.0  # type: ignore[misc]


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
    with pytest.raises(TypeError):
        ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 0)  # type: ignore[misc]


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
    assert imu.rate_of_turn_z_rad_s == 0.5
    assert imu.timestamp_ms == 2500
