"""Mag characterization: recover injected hard/soft iron from a capture."""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim.angles import wrap180
from plrs_sim.attitude import from_axis_angle
from plrs_sim.magcal import HarmonicFit, analyze_capture, fit_heading_harmonics
from plrs_sim.telemetry import GnssRecord, ImuRecord, format_record
from plrs_sim.types import Vec3

_WORLD_Z = Vec3(x=0.0, y=0.0, z=1.0)
_STILL = Vec3(x=0.0, y=0.0, z=0.0)
_GRAVITY = Vec3(x=0.0, y=0.0, z=9.81)


def test_fit_separates_hard_and_soft_iron() -> None:
    heading = np.arange(0.0, 360.0, 1.0)
    h = np.radians(heading)
    # constant 4, hard iron (1/rev) amp 5, soft iron (2/rev) amp 3.
    error = 4.0 + 5.0 * np.cos(h) + 3.0 * np.sin(2 * h)
    fit = fit_heading_harmonics(heading, error)
    assert fit.constant_deg == pytest.approx(4.0, abs=0.05)
    assert fit.hard_iron_amp_deg == pytest.approx(5.0, abs=0.05)
    assert fit.soft_iron_amp_deg == pytest.approx(3.0, abs=0.05)
    assert fit.residual_rms_deg < 0.05
    assert fit.bins_covered == 12
    assert fit.coverage_ok


def test_fit_survives_constant_near_180_seam() -> None:
    # A large constant offset (declination + frame) parked on the +-180 seam
    # would split raw least-squares across the wrap; the circular-mean centering
    # must still recover the constant and the iron amplitudes.
    heading = np.arange(0.0, 360.0, 1.0)
    h = np.radians(heading)
    error = wrap180(179.0 + 5.0 * np.cos(h) + 3.0 * np.sin(2 * h))
    fit = fit_heading_harmonics(heading, error)
    assert abs(wrap180(fit.constant_deg - 179.0)) < 0.2
    assert fit.hard_iron_amp_deg == pytest.approx(5.0, abs=0.1)
    assert fit.soft_iron_amp_deg == pytest.approx(3.0, abs=0.1)
    assert fit.residual_rms_deg < 0.1


def _capture_over_circle(iron_deg, headings) -> list[str]:
    """I/G lines around a full circle; the MTi yaw carries iron(heading)."""
    lines: list[str] = []
    for i, heading in enumerate(headings):
        yaw_enu = -(heading + iron_deg(heading))  # compass = heading + iron
        lines.append(
            format_record(
                ImuRecord(
                    timestamp_ms=i * 100,
                    orientation=from_axis_angle(_WORLD_Z, math.radians(yaw_enu)),
                    angular_velocity_rad_s=_STILL,
                    accel_ms2=_GRAVITY,
                )
            )
        )
        lines.append(
            format_record(
                GnssRecord(
                    timestamp_ms=i * 100,
                    heading_deg=heading,
                    heading_sigma_deg=0.3,
                    valid=True,
                )
            )
        )
    return lines


def test_analyze_capture_recovers_soft_iron() -> None:
    lines = _capture_over_circle(
        lambda h: 4.0 * math.cos(2 * math.radians(h)), np.arange(0.0, 360.0, 5.0)
    )
    fit = analyze_capture(lines)
    assert fit.soft_iron_amp_deg == pytest.approx(4.0, abs=0.3)
    assert fit.hard_iron_amp_deg < 0.5
    assert fit.residual_rms_deg < 0.3
    assert fit.coverage_ok


def test_low_coverage_is_flagged() -> None:
    # Only ~40 deg of heading visited: the fit cannot be trusted.
    lines = _capture_over_circle(lambda h: 0.0, np.arange(0.0, 40.0, 2.0))
    fit = analyze_capture(lines)
    assert not fit.coverage_ok


def test_empty_capture_raises() -> None:
    with pytest.raises(ValueError):
        analyze_capture(["# just a boot message", "garbage"])


def _fit(**overrides) -> HarmonicFit:
    base = dict(
        n_samples=120,
        bins_covered=12,
        constant_deg=5.0,
        hard_iron_amp_deg=0.5,
        soft_iron_amp_deg=0.5,
        residual_rms_deg=0.5,
        uncorrected_rms_deg=0.5,
    )
    base.update(overrides)
    return HarmonicFit(**base)


def test_verdict_low_coverage() -> None:
    v = _fit(bins_covered=4).verdict()
    assert "not enough heading coverage" in v.lower()


def test_verdict_clean_mag_recommends_pin() -> None:
    v = _fit(uncorrected_rms_deg=1.0).verdict()
    assert "SAFE" in v and "q_offset_outage_deg2" in v


def test_verdict_correctable_iron() -> None:
    v = _fit(
        soft_iron_amp_deg=8.0, uncorrected_rms_deg=6.0, residual_rms_deg=1.0
    ).verdict()
    assert "correctable" in v.lower() and "soft iron" in v.lower()


def test_verdict_wander_leaves_pin_off() -> None:
    v = _fit(uncorrected_rms_deg=8.0, residual_rms_deg=5.0).verdict()
    assert "OFF" in v and "fail" in v.lower()
