"""Parity tests against test/test_fusion/test_main.cpp.

Each test mirrors its C++ counterpart by name, inputs, and tolerances. If
these pass with the same numerics as the firmware suite, the binding and
the dataclass conversion layer are faithful.
"""

from __future__ import annotations

import numpy as np

from plrs_sim import GRAVITY_MS2, EkfConfig, GnssSample, ImuSample, TinyEkfFilter, Vec3

# FusionOutput::heading_variance_deg2 is a C float; uninitialised reads back
# as the C FLT_MAX, not Python's double-precision sys.float_info.max.
FLT_MAX = float(np.finfo(np.float32).max)

CFG = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)


def make_imu(rate_z_rad_s: float, t_ms: int) -> ImuSample:
    return ImuSample(
        angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=rate_z_rad_s),
        accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
        timestamp_ms=t_ms,
    )


def make_gnss(
    heading_deg: float, variance_deg2: float, t_ms: int, valid: bool = True
) -> GnssSample:
    return GnssSample(
        heading_deg=heading_deg,
        heading_variance_deg2=variance_deg2,
        timestamp_ms=t_ms,
        valid=valid,
    )


def test_initial_state_uninitialized() -> None:
    f = TinyEkfFilter(CFG)
    out = f.output()
    assert out.heading_variance_deg2 == FLT_MAX
    assert out.heading_deg == 0.0


def test_invalid_gnss_is_noop() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(45.0, 1.0, 1000, valid=False))
    out = f.output()
    assert out.heading_variance_deg2 == FLT_MAX
    assert out.heading_deg == 0.0


def test_first_valid_gnss_seeds_heading() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(45.0, 1.0, 1000))
    out = f.output()
    assert out.heading_deg == 45.0
    assert out.heading_variance_deg2 == 1000.0


def test_update_pulls_toward_measurement() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(0.0, 1.0, 1000))
    f.predict(make_imu(0.0, 1000))
    f.predict(make_imu(0.0, 5000))
    f.update(make_gnss(45.0, 0.001, 5000))
    assert abs(f.output().heading_deg - 45.0) < 1.0


def test_update_shrinks_variance() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(0.0, 1.0, 1000))
    f.predict(make_imu(0.0, 1000))
    f.predict(make_imu(0.0, 2000))
    before = f.output().heading_variance_deg2
    f.update(make_gnss(0.0, 0.1, 2000))
    after = f.output().heading_variance_deg2
    assert after < before


def test_first_predict_baseline_only() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(45.0, 1.0, 1000))
    f.predict(make_imu(1.0, 2000))
    assert f.output().heading_deg == 45.0


def test_predict_integrates_gyro() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(0.0, 1.0, 1000))
    f.predict(make_imu(0.0, 1000))
    f.predict(make_imu(0.5, 2000))
    # Heading is compass (CW-positive); a CCW gyro drives it negative.
    assert abs(f.output().heading_deg + 28.6479) < 0.01


def test_predict_grows_variance() -> None:
    f = TinyEkfFilter(CFG)
    f.update(make_gnss(0.0, 1.0, 1000))
    f.predict(make_imu(0.0, 1000))
    before = f.output().heading_variance_deg2
    f.predict(make_imu(0.0, 2000))
    after = f.output().heading_variance_deg2
    assert after > before
