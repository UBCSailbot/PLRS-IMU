"""Characterize the magnetometer against GNSS heading, as the filter sees it.

The filter aids heading off the MTi's fused (mag-referenced) yaw, absorbing a
single constant offset (declination + frame + a fixed hard-iron mean) into the
`mag_offset` state. Anything the offset cannot absorb is a heading-DEPENDENT
error, and that is what iron produces:

- 0/rev (constant): declination + frame + mean offset. The `mag_offset` state
  handles this; benign.
- 1/rev (once per revolution): hard iron. A constant field vector shifts the
  field circle off-center, so the heading error swings once per turn.
- 2/rev (twice per revolution): soft iron. Permeable metal stretches the circle
  into an ellipse (symmetric under 180 deg), so the error repeats twice per turn.
- residual (after removing 0/1/2): noise, wander, and dynamic iron. This is the
  part nothing can correct, so it is the number that decides whether the outage
  pin is safe (see docs/tuning.md).

`analyze_capture` reads a recorded telemetry stream (the `I` line's orientation
for the MTi yaw, `G` for GNSS truth; see telemetry.py) and fits those harmonics.
It characterizes the mag through the MTi's own fusion, which is the quantity the
filter actually consumes; a raw-`M` ellipse fit (to build a correction we apply
ourselves) would be a separate tool.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from .angles import wrap180
from .attitude import quaternion_to_euler_zyx
from .telemetry import GnssRecord, ImuRecord, parse_line

# Heading coverage is binned into 30 deg sectors; a fit needs most of the
# circle to separate 1/rev from 2/rev, so a capture that visits few sectors is
# flagged unreliable rather than trusted.
_COVERAGE_BINS = 12


@dataclass(frozen=True, slots=True, kw_only=True)
class HarmonicFit:
    """The 0/1/2-per-rev decomposition of mag-yaw-minus-GNSS heading error."""

    n_samples: int
    bins_covered: int  # of _COVERAGE_BINS 30 deg heading sectors visited
    constant_deg: float  # 0/rev: declination + frame + mean offset (benign)
    hard_iron_amp_deg: float  # 1/rev amplitude
    soft_iron_amp_deg: float  # 2/rev amplitude
    residual_rms_deg: float  # after removing 0/1/2: noise, wander, dynamic iron
    # Error left after removing only the constant, i.e. everything the filter's
    # offset state cannot absorb. This is what the mag contributes to heading
    # error during an outage if the 1/2-rev iron is NOT corrected.
    uncorrected_rms_deg: float

    @property
    def coverage_ok(self) -> bool:
        return self.bins_covered >= _COVERAGE_BINS - 2

    def summary(self) -> str:
        cov = (
            f"{self.bins_covered}/{_COVERAGE_BINS} sectors"
            f"{'' if self.coverage_ok else ' (LOW: cover more headings)'}"
        )
        const = f"{self.constant_deg:+.2f}"
        hard = f"{self.hard_iron_amp_deg:.2f}"
        soft = f"{self.soft_iron_amp_deg:.2f}"
        resid = f"{self.residual_rms_deg:.2f}"
        unc = f"{self.uncorrected_rms_deg:.2f}"
        return (
            f"mag vs GNSS over {self.n_samples} fixes, {cov}\n"
            f"  constant (0/rev, offset absorbs): {const} deg\n"
            f"  hard iron (1/rev):                {hard} deg\n"
            f"  soft iron (2/rev):                {soft} deg\n"
            f"  residual after 0/1/2:  {resid} deg rms (uncorrectable floor)\n"
            f"  offset removed only:   {unc} deg rms (iron left in)"
        )


def fit_heading_harmonics(
    heading_deg: np.ndarray, error_deg: np.ndarray
) -> HarmonicFit:
    """Least-squares fit of heading error to a 0/1/2-per-rev harmonic model.

    Assumes errors small enough not to wrap the +-180 seam (true for a mag past
    the MTi's own fusion); residuals are wrapped for safety.
    """
    h = np.radians(np.asarray(heading_deg, dtype=float))
    e = np.asarray(error_deg, dtype=float)
    design = np.column_stack(
        [np.ones_like(h), np.cos(h), np.sin(h), np.cos(2 * h), np.sin(2 * h)]
    )
    coeffs, *_ = np.linalg.lstsq(design, e, rcond=None)
    c0, a1, b1, a2, b2 = coeffs
    resid = wrap180(e - design @ coeffs)

    sectors = np.unique(
        (np.mod(np.degrees(h), 360.0) // (360.0 / _COVERAGE_BINS)).astype(int)
    )
    return HarmonicFit(
        n_samples=int(e.size),
        bins_covered=int(sectors.size),
        constant_deg=float(c0),
        hard_iron_amp_deg=float(math.hypot(a1, b1)),
        soft_iron_amp_deg=float(math.hypot(a2, b2)),
        residual_rms_deg=float(np.sqrt(np.mean(resid**2))),
        uncorrected_rms_deg=float(np.sqrt(np.mean(wrap180(e - c0) ** 2))),
    )


def heading_error_samples(
    lines: Iterable[str],
) -> tuple[np.ndarray, np.ndarray]:
    """Extract (GNSS heading, MTi-yaw-minus-GNSS error) pairs from a capture.

    One sample per valid `G` line, using the most recent `I`-line orientation
    for the MTi yaw. The MTi reports ENU yaw (CCW); compass heading is its
    negation, matching the filter's mag update. Both in degrees.
    """
    headings: list[float] = []
    errors: list[float] = []
    mti_compass_deg: float | None = None
    for line in lines:
        rec = parse_line(line)
        if isinstance(rec, ImuRecord):
            _, _, yaw_deg = quaternion_to_euler_zyx(rec.orientation)
            mti_compass_deg = -yaw_deg
        elif isinstance(rec, GnssRecord) and rec.valid and mti_compass_deg is not None:
            headings.append(rec.heading_deg)
            errors.append(float(wrap180(mti_compass_deg - rec.heading_deg)))
    return np.array(headings, dtype=float), np.array(errors, dtype=float)


def analyze_capture(lines: Iterable[str]) -> HarmonicFit:
    """Characterize the mag against GNSS from a recorded telemetry stream."""
    headings, errors = heading_error_samples(lines)
    if errors.size == 0:
        raise ValueError("no paired MTi/GNSS samples in capture")
    return fit_heading_harmonics(headings, errors)
