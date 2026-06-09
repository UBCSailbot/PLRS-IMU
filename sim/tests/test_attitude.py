"""Tests for plrs_sim.attitude (mirror of lib/fusion/attitude.h)."""

from __future__ import annotations

import math

import pytest

from plrs_sim import Quaternion, Vec3
from plrs_sim.attitude import (
    conjugate,
    euler_to_quaternion,
    from_axis_angle,
    multiply,
    quaternion_to_euler_zyx,
    rotate,
    world_to_body,
)

DEG_TO_RAD = math.pi / 180.0


def _approx_vec(v: Vec3, x: float, y: float, z: float, tol: float = 1e-6) -> None:
    assert v.x == pytest.approx(x, abs=tol)
    assert v.y == pytest.approx(y, abs=tol)
    assert v.z == pytest.approx(z, abs=tol)


def test_conjugate_negates_imaginary_part() -> None:
    q = Quaternion(w=0.5, x=0.5, y=-0.5, z=0.5)
    c = conjugate(q)
    assert (c.w, c.x, c.y, c.z) == (0.5, -0.5, 0.5, -0.5)


def test_multiply_identity_is_left_and_right_identity() -> None:
    q = from_axis_angle(Vec3(x=0.0, y=0.0, z=1.0), 1.234)
    i = Quaternion.identity()
    a = multiply(q, i)
    b = multiply(i, q)
    for got in (a, b):
        assert (got.w, got.x, got.y, got.z) == pytest.approx(
            (q.w, q.x, q.y, q.z), abs=1e-12
        )


def test_from_axis_angle_zero_is_identity() -> None:
    q = from_axis_angle(Vec3(x=1.0, y=0.0, z=0.0), 0.0)
    assert (q.w, q.x, q.y, q.z) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_rotate_identity_passes_through() -> None:
    v = Vec3(x=1.0, y=2.0, z=3.0)
    r = rotate(Quaternion.identity(), v)
    _approx_vec(r, 1.0, 2.0, 3.0)


def test_rotate_yaw_90_maps_east_to_north() -> None:
    q = from_axis_angle(Vec3(x=0.0, y=0.0, z=1.0), 90.0 * DEG_TO_RAD)
    r = rotate(q, Vec3(x=1.0, y=0.0, z=0.0))
    _approx_vec(r, 0.0, 1.0, 0.0)


def test_world_to_body_under_heel_splits_world_z() -> None:
    # Boat heeled 30 deg to starboard (positive roll about body X). World
    # yaw rate is purely along world Z; body Y tilts up to starboard so
    # it sees +sin(heel) of the world Z rate, and body Z gets cos(heel).
    heel_rad = 30.0 * DEG_TO_RAD
    omega = 1.0
    q = from_axis_angle(Vec3(x=1.0, y=0.0, z=0.0), heel_rad)
    body = world_to_body(q, Vec3(x=0.0, y=0.0, z=omega))
    _approx_vec(body, 0.0, omega * math.sin(heel_rad), omega * math.cos(heel_rad))


def test_world_to_body_identity_passes_through() -> None:
    body = world_to_body(Quaternion.identity(), Vec3(x=1.0, y=2.0, z=3.0))
    _approx_vec(body, 1.0, 2.0, 3.0)


def test_euler_identity_is_zero() -> None:
    roll, pitch, yaw = quaternion_to_euler_zyx(Quaternion.identity())
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_euler_pure_yaw_recovers_yaw() -> None:
    q = from_axis_angle(Vec3(x=0.0, y=0.0, z=1.0), 30.0 * DEG_TO_RAD)
    roll, pitch, yaw = quaternion_to_euler_zyx(q)
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 30.0), abs=1e-6)


def test_euler_pure_roll_recovers_roll() -> None:
    q = from_axis_angle(Vec3(x=1.0, y=0.0, z=0.0), 30.0 * DEG_TO_RAD)
    roll, pitch, yaw = quaternion_to_euler_zyx(q)
    assert (roll, pitch, yaw) == pytest.approx((30.0, 0.0, 0.0), abs=1e-6)


def test_euler_pure_pitch_recovers_pitch() -> None:
    q = from_axis_angle(Vec3(x=0.0, y=1.0, z=0.0), 30.0 * DEG_TO_RAD)
    roll, pitch, yaw = quaternion_to_euler_zyx(q)
    assert (roll, pitch, yaw) == pytest.approx((0.0, 30.0, 0.0), abs=1e-6)


def test_euler_to_quaternion_inverts_decomposition() -> None:
    for roll, pitch, yaw in [
        (0.0, 0.0, 0.0),
        (15.0, -8.0, 40.0),
        (-30.0, 20.0, -120.0),
    ]:
        q = euler_to_quaternion(roll, pitch, yaw)
        assert quaternion_to_euler_zyx(q) == pytest.approx((roll, pitch, yaw), abs=1e-6)
