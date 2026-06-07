"""Attitude math, mirroring lib/fusion/attitude.h.

Pure Python so the sim can compute heel projections and Euler angles
without crossing the FFI on the hot path. The formulas match
lib/fusion/attitude.h verbatim so a regression on either side is caught
by parity tests.
"""

from __future__ import annotations

import math

from .types import Quaternion, Vec3


def conjugate(q: Quaternion) -> Quaternion:
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def from_axis_angle(axis: Vec3, angle_rad: float) -> Quaternion:
    """Unit quaternion encoding a rotation `angle_rad` about `axis`.

    The axis is assumed unit-length; pass (1,0,0), (0,1,0), or (0,0,1)
    for principal axes. No internal normalization.
    """
    half = 0.5 * angle_rad
    s = math.sin(half)
    return Quaternion(w=math.cos(half), x=axis.x * s, y=axis.y * s, z=axis.z * s)


def rotate(q: Quaternion, v: Vec3) -> Vec3:
    """Rotate v by unit quaternion q. Same formula as fusion::rotate."""
    tx = 2.0 * (q.y * v.z - q.z * v.y)
    ty = 2.0 * (q.z * v.x - q.x * v.z)
    tz = 2.0 * (q.x * v.y - q.y * v.x)
    return Vec3(
        x=v.x + q.w * tx + (q.y * tz - q.z * ty),
        y=v.y + q.w * ty + (q.z * tx - q.x * tz),
        z=v.z + q.w * tz + (q.x * ty - q.y * tx),
    )


def world_to_body(orientation: Quaternion, v_world: Vec3) -> Vec3:
    """Rotate a world-frame vector into the body frame.

    orientation is the world<-body quaternion; its conjugate rotates
    world vectors into body. Used to project the world-frame yaw rate
    onto the heeled boat's body axes.
    """
    return rotate(conjugate(orientation), v_world)


def quaternion_to_euler_zyx(q: Quaternion) -> tuple[float, float, float]:
    """ZYX intrinsic Euler decomposition. Returns (roll_deg, pitch_deg, yaw_deg)."""
    sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    rad_to_deg = 180.0 / math.pi
    return roll * rad_to_deg, pitch * rad_to_deg, yaw * rad_to_deg
