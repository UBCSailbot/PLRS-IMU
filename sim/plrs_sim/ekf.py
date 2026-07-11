"""Python wrapper over the bound C++ EKF.

Converts dataclasses to/from the FFI structs at the call boundary so the
public surface stays Rust-shaped (frozen value types in, value type out)
even though the filter itself is C++.
"""

from __future__ import annotations

from . import _native
from .attitude import euler_to_quaternion
from .types import (
    EkfConfig,
    EkfDebug,
    FusionOutput,
    GnssAttitudeMount,
    GnssSample,
    ImuSample,
    Quaternion,
    Vec3,
)

# AttEuler mode codes: 0 is no attitude solution, 2 is heading + pitch from a
# fixed-ambiguity two-antenna baseline (see lib/septentrio_gnss/sbf_blocks.h).
_ATT_MODE_NO_ATTITUDE = 0
_ATT_MODE_HEADING_PITCH = 2


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
    if c.mti_yaw is not None:
        yaw = _native.MtiYawConfig()
        yaw.variance_deg2 = c.mti_yaw.variance_deg2
        yaw.q_offset_deg2 = c.mti_yaw.q_offset_deg2
        yaw.p0_offset_deg2 = c.mti_yaw.p0_offset_deg2
        n.mti_yaw = yaw
    mount = _native.MountRotation()
    mount.boat_to_imu = _to_native_unit_quaternion(
        euler_to_quaternion(c.mount_roll_deg, c.mount_pitch_deg, c.mount_yaw_deg)
    )
    n.mount = mount
    return n


def _to_native_mount(m: GnssAttitudeMount) -> _native.GnssAttitudeMount:
    n = _native.GnssAttitudeMount()
    n.baseline_offset_deg = m.baseline_offset_deg
    n.fallback_heading_variance_deg2 = m.fallback_heading_variance_deg2
    return n


def gnss_sample_from_attitude(
    *,
    heading_deg: float,
    heading_variance_deg2: float,
    valid: bool,
    tow_ms: int,
    mount: GnssAttitudeMount,
) -> GnssSample:
    """Run an antenna-baseline heading through the real C++ bridge.

    heading_deg is in the antenna-baseline frame; the bridge subtracts the
    mount offset and wraps to +-180. valid=False emits a no-attitude block
    so the bridge marks the result invalid, mirroring a GNSS dropout.
    """
    att = _native.AttEuler()
    att.tow = tow_ms
    att.mode = _ATT_MODE_HEADING_PITCH if valid else _ATT_MODE_NO_ATTITUDE
    att.heading = heading_deg
    cov = _native.AttCovEuler()
    cov.cov_headhead = heading_variance_deg2
    s = _native.att_euler_to_gnss_sample(
        att=att, cov=cov, mount=_to_native_mount(mount)
    )
    return GnssSample(
        heading_deg=s.heading_deg,
        heading_variance_deg2=s.heading_variance_deg2,
        timestamp_ms=s.timestamp_ms,
        valid=s.valid,
    )


def _from_native_output(o: _native.FusionOutput) -> FusionOutput:
    return FusionOutput(
        heading_deg=o.heading_deg,
        heading_variance_deg2=o.heading_variance_deg2,
        roll_deg=o.roll_deg,
        roll_variance_deg2=o.roll_variance_deg2,
        pitch_deg=o.pitch_deg,
        pitch_variance_deg2=o.pitch_variance_deg2,
        timestamp_ms=o.timestamp_ms,
        yaw_rate_dps=o.yaw_rate_dps,
    )


class TinyEkfFilter:
    """Five-state EKF over heading, roll, pitch, gyro_z bias, and mag
    offset; see lib/fusion/ekf_filter.h."""

    __slots__ = ("_inner",)

    def __init__(self, cfg: EkfConfig) -> None:
        self._inner = _native.TinyEkfFilter(_to_native_config(cfg))

    def predict(self, imu: ImuSample) -> None:
        self._inner.predict(_to_native_imu(imu))

    def update(self, gnss: GnssSample) -> None:
        self._inner.update(_to_native_gnss(gnss))

    def output(self) -> FusionOutput:
        return _from_native_output(self._inner.output())

    def debug(self) -> EkfDebug:
        d = self._inner.debug()
        return EkfDebug(
            gyro_bias_dps=d.gyro_bias_dps,
            gyro_bias_variance_deg2_s2=d.gyro_bias_variance_deg2_s2,
            mag_offset_deg=d.mag_offset_deg,
            mag_offset_variance_deg2=d.mag_offset_variance_deg2,
            gate_rejects=d.gate_rejects,
            mag_gate_rejects=d.mag_gate_rejects,
        )
