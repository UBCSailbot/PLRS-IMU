"""Tests for the ground-truth samplers.

Each variant has a closed-form expected value at known t; tests check the
implementation matches it. The match-dispatch in truth.py is exhaustively
exercised by visiting every variant.
"""

from __future__ import annotations

import math

import pytest

from plrs_sim import (
    ConstantTrim,
    ConstantTurn,
    LevelAttitude,
    Quaternion,
    Sinusoidal,
    Static,
    StepTurns,
    WaveMotion,
)
from plrs_sim.attitude import conjugate, multiply, quaternion_to_euler_zyx
from plrs_sim.truth import DEG_TO_RAD, sample_attitude, sample_heading


def _heading(profile, t_ms: int) -> float:
    return sample_heading(profile, t_ms)[0]


def _gyro_z(profile, t_ms: int) -> float:
    return sample_heading(profile, t_ms)[1]


def test_constant_turn_heading_is_linear() -> None:
    t = ConstantTurn(rate_deg_s=10.0, heading0_deg=5.0)
    assert _heading(t, 0) == pytest.approx(5.0)
    assert _heading(t, 1000) == pytest.approx(15.0)
    assert _heading(t, 3000) == pytest.approx(35.0)


def test_constant_turn_gyro_is_constant() -> None:
    t = ConstantTurn(rate_deg_s=18.0 / math.pi)
    expected = 18.0 / math.pi * DEG_TO_RAD
    assert _gyro_z(t, 0) == pytest.approx(expected)
    assert _gyro_z(t, 9999) == pytest.approx(expected)


def test_sinusoidal_heading_zero_at_t0() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0, heading0_deg=45.0)
    assert _heading(s, 0) == pytest.approx(45.0)


def test_sinusoidal_heading_peak_at_quarter_period() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0)
    assert _heading(s, 2500) == pytest.approx(30.0)
    assert _heading(s, 7500) == pytest.approx(-30.0)


def test_sinusoidal_gyro_is_derivative_of_heading() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0)
    omega = 2.0 * math.pi / 10.0
    expected_at_0 = 30.0 * omega * DEG_TO_RAD
    assert _gyro_z(s, 0) == pytest.approx(expected_at_0)
    assert _gyro_z(s, 2500) == pytest.approx(0.0, abs=1e-12)


def test_step_turns_integrates_rate_within_leg() -> None:
    st = StepTurns(legs=((10.0, 0.0), (2.0, 45.0), (10.0, 0.0)))
    assert _heading(st, 0) == pytest.approx(0.0)
    assert _heading(st, 5000) == pytest.approx(0.0)
    assert _heading(st, 11000) == pytest.approx(45.0)
    assert _heading(st, 12000) == pytest.approx(90.0)
    assert _heading(st, 20000) == pytest.approx(90.0)


def test_step_turns_respects_heading0() -> None:
    st = StepTurns(legs=((1.0, 0.0),), heading0_deg=37.0)
    assert _heading(st, 0) == pytest.approx(37.0)
    assert _heading(st, 1000) == pytest.approx(37.0)


def test_step_turns_clamps_past_final_leg() -> None:
    st = StepTurns(legs=((2.0, 30.0),))
    assert _heading(st, 5000) == pytest.approx(60.0)


def test_step_turns_empty_returns_heading0() -> None:
    assert _heading(StepTurns(legs=(), heading0_deg=12.5), 1234) == pytest.approx(12.5)


def test_step_turns_gyro_is_leg_rate() -> None:
    st = StepTurns(legs=((1.0, 0.0), (1.0, 45.0), (1.0, -90.0)))
    assert _gyro_z(st, 500) == pytest.approx(0.0)
    assert _gyro_z(st, 1500) == pytest.approx(45.0 * DEG_TO_RAD)
    assert _gyro_z(st, 2500) == pytest.approx(-90.0 * DEG_TO_RAD)
    assert _gyro_z(st, 9999) == pytest.approx(0.0)


def test_static_heading_is_constant() -> None:
    s = Static(heading_deg=42.0)
    assert _heading(s, 0) == 42.0
    assert _heading(s, 999999) == 42.0


def test_static_gyro_is_zero() -> None:
    assert _gyro_z(Static(heading_deg=42.0), 12345) == 0.0


def test_level_attitude_is_identity_at_all_times() -> None:
    q, omega = sample_attitude(LevelAttitude(), 0)
    assert q == Quaternion.identity()
    assert omega.x == 0.0 and omega.y == 0.0 and omega.z == 0.0
    q2, omega2 = sample_attitude(LevelAttitude(), 999999)
    assert q2 == Quaternion.identity()
    assert omega2.x == 0.0 and omega2.y == 0.0 and omega2.z == 0.0


def test_constant_trim_is_fixed_pitch_with_zero_rate() -> None:
    # ConstantTrim holds a fixed pitch (rotation about body Y) with no roll,
    # no yaw, and zero attitude-motion body rate, at any time.
    for angle in (30.0, 60.0, -45.0):
        q, omega = sample_attitude(ConstantTrim(angle_deg=angle), 7777)
        roll, pitch, yaw = quaternion_to_euler_zyx(q)
        assert pitch == pytest.approx(angle, abs=1e-6)
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert yaw == pytest.approx(0.0, abs=1e-6)
        assert omega.x == 0.0 and omega.y == 0.0 and omega.z == 0.0


def test_wave_motion_amplitudes_and_phase() -> None:
    # Roll peaks a quarter period in; pitch is independent.
    w = WaveMotion(
        roll_amplitude_deg=15.0,
        roll_period_s=4.0,
        pitch_amplitude_deg=5.0,
        pitch_period_s=3.0,
    )
    q0, omega0 = sample_attitude(w, 0)
    assert q0 == Quaternion.identity()  # both sines zero at t=0
    # Roll rate is maximal at t=0, pitch rate too; both positive.
    assert omega0.x == pytest.approx(15.0 * (2 * math.pi / 4.0) * DEG_TO_RAD)
    assert omega0.y == pytest.approx(5.0 * (2 * math.pi / 3.0) * DEG_TO_RAD)
    assert omega0.z == pytest.approx(0.0, abs=1e-12)


def test_wave_motion_body_rate_matches_orientation_derivative() -> None:
    # The returned omega must be the true body angular velocity of the
    # returned orientation: recover it by central-differencing q(t) and
    # using dq/dt = 0.5 * q (x) (0, omega).
    w = WaveMotion(
        roll_amplitude_deg=15.0,
        roll_period_s=4.0,
        pitch_amplitude_deg=5.0,
        pitch_period_s=3.0,
    )
    h_ms = 1
    h_s = h_ms / 1000.0
    for t_ms in (250, 800, 1500, 2300):
        q, omega = sample_attitude(w, t_ms)
        q_plus, _ = sample_attitude(w, t_ms + h_ms)
        q_minus, _ = sample_attitude(w, t_ms - h_ms)
        dqdt = Quaternion(
            w=(q_plus.w - q_minus.w) / (2 * h_s),
            x=(q_plus.x - q_minus.x) / (2 * h_s),
            y=(q_plus.y - q_minus.y) / (2 * h_s),
            z=(q_plus.z - q_minus.z) / (2 * h_s),
        )
        recovered = multiply(conjugate(q), dqdt)
        assert 2.0 * recovered.x == pytest.approx(omega.x, abs=1e-3)
        assert 2.0 * recovered.y == pytest.approx(omega.y, abs=1e-3)
        assert 2.0 * recovered.z == pytest.approx(omega.z, abs=1e-3)
