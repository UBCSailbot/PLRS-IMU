"""Python wrapper over the bound C++ EKF.

Converts dataclasses to/from the FFI structs at the call boundary so the
public surface stays Rust-shaped (frozen value types in, value type out)
even though the filter itself is C++.
"""

from __future__ import annotations

from . import _native
from .types import EkfConfig, FusionOutput, GnssSample, ImuSample, Vec3


def _to_native_vec3(v: Vec3) -> _native.Vec3:
    return _native.Vec3(v.x, v.y, v.z)


def _to_native_imu(s: ImuSample) -> _native.ImuSample:
    n = _native.ImuSample()
    n.angular_velocity_rad_s = _to_native_vec3(s.angular_velocity_rad_s)
    n.accel_ms2 = _to_native_vec3(s.accel_ms2)
    n.timestamp_ms = s.timestamp_ms
    return n


def _to_native_gnss(s: GnssSample) -> _native.GnssSample:
    n = _native.GnssSample()
    n.heading_deg = s.heading_deg
    n.heading_variance_deg2 = s.heading_variance_deg2
    n.timestamp_ms = s.timestamp_ms
    n.valid = s.valid
    return n


def _to_native_config(c: EkfConfig) -> _native.Config:
    n = _native.Config()
    n.q_heading_deg2 = c.q_heading_deg2
    n.q_bias_deg2_s2 = c.q_bias_deg2_s2
    n.p0_heading_deg2 = c.p0_heading_deg2
    n.p0_bias_deg2_s2 = c.p0_bias_deg2_s2
    return n


def _from_native_output(o: _native.FusionOutput) -> FusionOutput:
    return FusionOutput(
        heading_deg=o.heading_deg,
        heading_variance_deg2=o.heading_variance_deg2,
        timestamp_ms=o.timestamp_ms,
    )


class TinyEkfFilter:
    """Two-state EKF over heading and gyro_z bias; see lib/fusion/ekf_filter.h."""

    __slots__ = ("_inner",)

    def __init__(self, cfg: EkfConfig) -> None:
        self._inner = _native.TinyEkfFilter(_to_native_config(cfg))

    def predict(self, imu: ImuSample) -> None:
        self._inner.predict(_to_native_imu(imu))

    def update(self, gnss: GnssSample) -> None:
        self._inner.update(_to_native_gnss(gnss))

    def output(self) -> FusionOutput:
        return _from_native_output(self._inner.output())
