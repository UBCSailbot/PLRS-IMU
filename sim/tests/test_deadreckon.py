"""Tests for the shared heading dead-reckoner."""

from __future__ import annotations

import math

import pytest

from plrs_sim.deadreckon import HeadingDeadReckoner


def test_step_returns_none_until_anchored() -> None:
    dr = HeadingDeadReckoner()
    assert dr.step(100, 5.0) is None
    assert not dr.anchored


def test_step_integrates_the_negated_yaw_rate() -> None:
    # Compass heading falls as the ENU yaw rate rises: a +1 rad/s gyro over 1 s
    # subtracts 1 rad (~57.3 deg).
    dr = HeadingDeadReckoner()
    dr.anchor(0.0)
    dr.step(0, 0.0)  # establishes the previous timestamp
    assert dr.step(1000, 1.0) == pytest.approx(-math.degrees(1.0))


def test_step_wraps_across_the_seam() -> None:
    dr = HeadingDeadReckoner()
    dr.anchor(170.0)
    dr.step(0, 0.0)
    # A -20 deg/s ENU rate raises the heading to 190, which wraps to -170.
    assert dr.step(1000, -math.radians(20.0)) == pytest.approx(-170.0)


def test_first_step_after_anchor_spans_the_real_interval() -> None:
    # The un-anchored step still records its timestamp, so the first integrated
    # step covers the gap since then rather than starting from zero dt.
    dr = HeadingDeadReckoner()
    dr.step(1000, 9.9)  # not anchored: no integration, but prev-ms is recorded
    dr.anchor(0.0)
    assert dr.step(2000, 1.0) == pytest.approx(-math.degrees(1.0))


def test_anchor_overrides_the_current_heading() -> None:
    dr = HeadingDeadReckoner()
    dr.anchor(0.0)
    dr.step(0, 0.0)
    dr.step(1000, 1.0)
    dr.anchor(45.0)
    assert dr.step(2000, 0.0) == pytest.approx(45.0)
