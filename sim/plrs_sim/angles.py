"""Angle-wrapping helpers shared across the sim.

The wrapping convention matches the firmware's fusion::wrap180
(lib/fusion/attitude.h): std::remainder rounds the quotient half-to-even, so
angles map into (-180, 180] with the +-180 seam resolving to +180. np.round
shares that half-to-even rounding, so this holds for scalars and ndarrays alike.
"""

from __future__ import annotations

import numpy as np


def wrap180(deg):
    """Wrap an angle (or angle difference) in degrees into (-180, 180].

    Accepts a scalar or an ndarray. Scalar callers get an np.float64 back; wrap
    in float() where a plain float is required.
    """
    return deg - 360.0 * np.round(deg / 360.0)


def seam_broken(deg: np.ndarray) -> np.ndarray:
    """Wrap to +-180 and NaN out the sample after each seam crossing so a line
    plot does not draw a vertical connector across the +-180 jump."""
    wrapped = wrap180(np.asarray(deg, dtype=float))
    out = wrapped.copy()
    jumped = np.abs(np.diff(wrapped)) > 180.0
    out[1:][jumped] = np.nan
    return out
