"""Open-loop heading dead-reckoning, shared by the runner and the live monitor.

Both integrate gyro-z into a compass heading the same way; only the anchor
policy (when to anchor, and to what reference) differs, so that stays in the
callers. The compass heading integrates the *negated* ENU yaw rate, and the
result is wrapped to (-180, 180] to match fusion::wrap180 on the firmware.
"""

from __future__ import annotations

import math

from .angles import wrap180

_RAD_TO_DEG = 180.0 / math.pi


class HeadingDeadReckoner:
    """Heading state plus the previous timestamp, advanced by gyro integration.

    Not anchored until the caller supplies a reference via anchor(); step()
    returns None until then. prev-timestamp bookkeeping happens on every step,
    anchored or not, so the first integration after anchoring spans the real
    interval since the last sample.
    """

    def __init__(self) -> None:
        self._heading: float | None = None
        self._prev_ms: int | None = None

    @property
    def anchored(self) -> bool:
        return self._heading is not None

    def anchor(self, heading_deg: float) -> None:
        """Set the heading to an absolute reference (wrapped to (-180, 180])."""
        self._heading = float(wrap180(heading_deg))

    def step(self, t_ms: int, gyro_z_rad_s: float) -> float | None:
        """Advance the heading over the interval since the last step and return
        it wrapped, or None if not yet anchored."""
        if self._heading is not None and self._prev_ms is not None:
            dt_s = (t_ms - self._prev_ms) / 1000.0
            self._heading = float(
                wrap180(self._heading - gyro_z_rad_s * _RAD_TO_DEG * dt_s)
            )
        self._prev_ms = t_ms
        return self._heading
