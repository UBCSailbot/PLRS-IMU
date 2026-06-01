"""Tests for ground-truth trajectories.

Each variant has a closed-form expected value at known t; tests check the
implementation matches it. The match-dispatch in truth.py is exhaustively
exercised by visiting every variant in both functions.
"""

from __future__ import annotations

import math

import pytest

from plrs_sim import ConstantTurn, Sinusoidal, Static, StepTurns
from plrs_sim.truth import DEG_TO_RAD, gyro_z_at, heading_at


def test_constant_turn_heading_is_linear() -> None:
    t = ConstantTurn(rate_deg_s=10.0, heading0_deg=5.0)
    assert heading_at(t, 0) == pytest.approx(5.0)
    assert heading_at(t, 1000) == pytest.approx(15.0)
    assert heading_at(t, 3000) == pytest.approx(35.0)


def test_constant_turn_gyro_is_constant() -> None:
    t = ConstantTurn(rate_deg_s=18.0 / math.pi)
    expected = 18.0 / math.pi * DEG_TO_RAD
    assert gyro_z_at(t, 0) == pytest.approx(expected)
    assert gyro_z_at(t, 9999) == pytest.approx(expected)


def test_sinusoidal_heading_zero_at_t0() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0, heading0_deg=45.0)
    assert heading_at(s, 0) == pytest.approx(45.0)


def test_sinusoidal_heading_peak_at_quarter_period() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0)
    assert heading_at(s, 2500) == pytest.approx(30.0)
    assert heading_at(s, 7500) == pytest.approx(-30.0)


def test_sinusoidal_gyro_is_derivative_of_heading() -> None:
    s = Sinusoidal(amplitude_deg=30.0, period_s=10.0)
    omega = 2.0 * math.pi / 10.0
    expected_at_0 = 30.0 * omega * DEG_TO_RAD
    assert gyro_z_at(s, 0) == pytest.approx(expected_at_0)
    assert gyro_z_at(s, 2500) == pytest.approx(0.0, abs=1e-12)


def test_step_turns_integrates_rate_within_leg() -> None:
    st = StepTurns(legs=((10.0, 0.0), (2.0, 45.0), (10.0, 0.0)))
    assert heading_at(st, 0) == pytest.approx(0.0)
    assert heading_at(st, 5000) == pytest.approx(0.0)
    assert heading_at(st, 11000) == pytest.approx(45.0)
    assert heading_at(st, 12000) == pytest.approx(90.0)
    assert heading_at(st, 20000) == pytest.approx(90.0)


def test_step_turns_respects_heading0() -> None:
    st = StepTurns(legs=((1.0, 0.0),), heading0_deg=37.0)
    assert heading_at(st, 0) == pytest.approx(37.0)
    assert heading_at(st, 1000) == pytest.approx(37.0)


def test_step_turns_clamps_past_final_leg() -> None:
    st = StepTurns(legs=((2.0, 30.0),))
    assert heading_at(st, 5000) == pytest.approx(60.0)


def test_step_turns_empty_returns_heading0() -> None:
    assert heading_at(StepTurns(legs=(), heading0_deg=12.5), 1234) == pytest.approx(
        12.5
    )


def test_step_turns_gyro_is_leg_rate() -> None:
    st = StepTurns(legs=((1.0, 0.0), (1.0, 45.0), (1.0, -90.0)))
    assert gyro_z_at(st, 500) == pytest.approx(0.0)
    assert gyro_z_at(st, 1500) == pytest.approx(45.0 * DEG_TO_RAD)
    assert gyro_z_at(st, 2500) == pytest.approx(-90.0 * DEG_TO_RAD)
    assert gyro_z_at(st, 9999) == pytest.approx(0.0)


def test_static_heading_is_constant() -> None:
    s = Static(heading_deg=42.0)
    assert heading_at(s, 0) == 42.0
    assert heading_at(s, 999999) == 42.0


def test_static_gyro_is_zero() -> None:
    assert gyro_z_at(Static(heading_deg=42.0), 12345) == 0.0
