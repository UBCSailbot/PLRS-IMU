"""Replay a captured telemetry log through the real filter.

`ReplaySource` turns a recorded telemetry stream (the `I`/`G` lines defined in
`telemetry.py`, which is the wire schema) back into `Tick`s, so a real hardware
capture drives the exact same runner + C++ filter as the sim. This is the
"tune what ships" guarantee applied to recorded data: the Q you pick replaying a
session is the Q the firmware runs.

Two things the capture does not carry:

- Ground truth. The `truth_*` fields are NaN; on the water the dual-antenna
  GNSS heading is the reference, and it rides through as the measurement series.
- Full-rate IMU, unless the firmware was in raw-log mode. The filter integrates
  at whatever `dt` the `I` timestamps give, so a 10 Hz throttled telemetry
  capture does NOT reproduce the 100 Hz firmware filter. Tuning captures must
  log the `I` line at the predict rate; see the raw-log follow-up.

Pairing: each `I` line drives one `Tick` (it carries the orientation the filter
needs). The most recent `G` line since the previous `Tick` rides along as that
`Tick`'s GNSS, so the runner's predict-then-update order matches the firmware.
If two `G` lines fall between the same pair of `I` lines only the last is kept
(the firmware would update on each), but with the IMU faster than GNSS in any
real capture there is always an `I` between fixes, so none are dropped. The `M`
(raw mag) and `F` (firmware fused output) lines are ignored here; the mag
belongs to the offline mag analyzer and `F` to a firmware-vs-replay check.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass

from .telemetry import GnssRecord, ImuRecord, parse_line
from .types import GnssSample, ImuSample, Tick


def _imu_sample(rec: ImuRecord) -> ImuSample:
    return ImuSample(
        angular_velocity_rad_s=rec.angular_velocity_rad_s,
        accel_ms2=rec.accel_ms2,
        orientation=rec.orientation,
        timestamp_ms=rec.timestamp_ms,
    )


def _gnss_sample(rec: GnssRecord) -> GnssSample:
    # The G line carries the heading sigma; the filter wants variance.
    return GnssSample(
        heading_deg=rec.heading_deg,
        heading_variance_deg2=rec.heading_sigma_deg * rec.heading_sigma_deg,
        timestamp_ms=rec.timestamp_ms,
        valid=rec.valid,
    )


@dataclass(frozen=True, slots=True, kw_only=True)
class ReplaySource:
    """A recorded telemetry stream as a source of `Tick`s.

    `lines` is any iterable of telemetry lines (e.g. `monitor.replay_file`). It
    is consumed once, so pass a fresh iterable per run.
    """

    lines: Iterable[str]

    def __iter__(self) -> Iterator[Tick]:
        pending_gnss: GnssSample | None = None
        for line in self.lines:
            rec = parse_line(line)
            if isinstance(rec, GnssRecord):
                pending_gnss = _gnss_sample(rec)
            elif isinstance(rec, ImuRecord):
                yield Tick(
                    timestamp_ms=rec.timestamp_ms,
                    truth_heading_deg=math.nan,
                    truth_roll_deg=math.nan,
                    truth_pitch_deg=math.nan,
                    imu=_imu_sample(rec),
                    gnss=pending_gnss,
                )
                pending_gnss = None
