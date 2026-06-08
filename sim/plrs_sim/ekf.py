"""Python wrapper over the bound C++ EKF.

Converts dataclasses to/from the FFI structs at the call boundary so the
public surface stays Rust-shaped (frozen value types in, value type out)
even though the filter itself is C++.
"""

from __future__ import annotations

from . import _native
from .types import EkfConfig, FusionOutput, GnssSample, ImuSample, Quaternion, Vec3


def _to_native_vec3(v: Vec3) -> _native.Vec3:
    return _native.Vec3(v.x, v.y, v.z)


def _to_native_quaternion(q: Quaternion) -> _native.Quaternion:
    return _native.Quaternion(q.w, q.x, q.y, q.z)


def _to_native_unit_quaternion(q: Quaternion) -> _native.UnitQuaternion:
    return _native.UnitQuaternion.from_raw(_to_native_quaternion(q))


def _to_native_imu(s: ImuSample) -> _native.ImuSample:
    n = _native.ImuSample()
    n.angular_velocity_rad_s = _to_native_vec3(s.angular_velocity_rad_s)
    n.accel_ms2 = _to_native_vec3(s.accel_ms2)
    n.orientation = _to_native_unit_quaternion(s.orientation)
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
    n.q_roll_deg2 = c.q_roll_deg2
    n.q_pitch_deg2 = c.q_pitch_deg2
    n.q_bias_deg2_s2 = c.q_bias_deg2_s2
    n.p0_heading_deg2 = c.p0_heading_deg2
    n.p0_roll_deg2 = c.p0_roll_deg2
    n.p0_pitch_deg2 = c.p0_pitch_deg2
    n.p0_bias_deg2_s2 = c.p0_bias_deg2_s2
    n.mti_roll_variance_deg2 = c.mti_roll_variance_deg2
    n.mti_pitch_variance_deg2 = c.mti_pitch_variance_deg2
    return n


def _from_native_output(o: _native.FusionOutput) -> FusionOutput:
    return FusionOutput(
        heading_deg=o.heading_deg,
        heading_variance_deg2=o.heading_variance_deg2,
        roll_deg=o.roll_deg,
        roll_variance_deg2=o.roll_variance_deg2,
        pitch_deg=o.pitch_deg,
        pitch_variance_deg2=o.pitch_variance_deg2,
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
