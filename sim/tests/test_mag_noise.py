"""The sim's hard/soft iron shows up as 1/rev and 2/rev, per the analyzer.

Ties noise.py (which injects the disturbance) to magcal.py (which fits it),
so the sim can stand in for a real soft-iron boat before hardware exists.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim.angles import wrap180
from plrs_sim.attitude import from_axis_angle, quaternion_to_euler_zyx
from plrs_sim.magcal import fit_heading_harmonics
from plrs_sim.noise import ImuNoise
from plrs_sim.types import ImuNoiseModel, ImuSample, MagNoiseModel, Vec3

_Z = Vec3(x=0.0, y=0.0, z=1.0)


def _corrupted_heading_error(model: ImuNoiseModel, headings: np.ndarray) -> np.ndarray:
    """MTi-yaw-minus-truth error at each true heading, mag disturbance only."""
    noise = ImuNoise(model, np.random.default_rng(0))
    errors = []
    for heading in headings:
        clean = ImuSample(
            angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=0.0),
            accel_ms2=Vec3(x=0.0, y=0.0, z=9.81),
            orientation=from_axis_angle(_Z, math.radians(-heading)),  # ENU yaw
            timestamp_ms=0,
        )
        corrupted = noise.corrupt(clean, dt_s=0.01)
        _, _, yaw_deg = quaternion_to_euler_zyx(corrupted.orientation)
        errors.append(float(wrap180(-yaw_deg - heading)))  # compass = -ENU yaw
    return np.array(errors)


def test_soft_iron_is_two_per_rev() -> None:
    headings = np.arange(0.0, 360.0, 5.0)
    err = _corrupted_heading_error(
        ImuNoiseModel(mag=MagNoiseModel(soft_iron_deg=6.0)), headings
    )
    fit = fit_heading_harmonics(headings, err)
    assert fit.soft_iron_amp_deg == pytest.approx(6.0, abs=0.3)
    assert fit.hard_iron_amp_deg < 0.5


def test_hard_iron_is_one_per_rev() -> None:
    headings = np.arange(0.0, 360.0, 5.0)
    err = _corrupted_heading_error(
        ImuNoiseModel(mag=MagNoiseModel(iron_deg=6.0)), headings
    )
    fit = fit_heading_harmonics(headings, err)
    assert fit.hard_iron_amp_deg == pytest.approx(6.0, abs=0.3)
    assert fit.soft_iron_amp_deg < 0.5
