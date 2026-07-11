"""Tests for the shared angle-wrapping helpers."""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim.angles import seam_broken, wrap180


def test_wrap180_maps_into_range() -> None:
    got = wrap180(np.array([0.0, 179.0, -179.0, 190.0, -190.0, 358.0, -360.0]))
    assert got == pytest.approx([0.0, 179.0, -179.0, -170.0, 170.0, -2.0, 0.0])


def test_wrap180_handles_a_difference_across_the_seam() -> None:
    # 179 and -179 are 2 deg apart, not 358.
    assert wrap180(np.array([-179.0 - 179.0])) == pytest.approx([2.0])


def test_wrap180_resolves_the_seam_to_plus_180() -> None:
    # The deliberate convention: the principal +-180 seam lands on +180, unlike
    # the modulo form (deg + 180) % 360 - 180, which lands on -180. Matches
    # fusion::wrap180 (std::remainder). The half-to-even quotient rounding then
    # flips the sign at successive odd multiples (540 -> -180, 900 -> +180),
    # exactly as std::remainder does.
    assert float(wrap180(180.0)) == pytest.approx(180.0)
    assert float(wrap180(-180.0)) == pytest.approx(-180.0)
    assert float(wrap180(540.0)) == pytest.approx(-180.0)
    assert float(wrap180(900.0)) == pytest.approx(180.0)


def test_wrap180_scalar_matches_math_remainder() -> None:
    for deg in (0.0, 45.0, 190.0, -190.0, 359.9, -720.5, 12345.6):
        assert float(wrap180(deg)) == pytest.approx(math.remainder(deg, 360.0))


def test_seam_broken_inserts_nan_at_crossing() -> None:
    out = seam_broken(np.array([170.0, 179.0, 190.0, 200.0]))  # crosses +180
    assert not np.isnan(out[1])
    assert np.isnan(out[2])  # the sample just past the seam breaks the line
    assert out[3] == pytest.approx(-160.0)


def test_seam_broken_leaves_a_clean_series_untouched() -> None:
    out = seam_broken(np.array([-10.0, 0.0, 10.0, 20.0]))
    assert out == pytest.approx([-10.0, 0.0, 10.0, 20.0])
