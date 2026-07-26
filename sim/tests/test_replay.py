"""ReplaySource: a captured I/G telemetry stream drives the real filter.

Round-trips through the wire codec (format_record -> parse_line) so the test
exercises the same path a hardware capture would take.
"""

from __future__ import annotations

import math

import numpy as np

from plrs_sim import ReplaySource, Vec3
from plrs_sim.angles import wrap180
from plrs_sim.runner import run
from plrs_sim.telemetry import GnssRecord, ImuRecord, format_record
from plrs_sim.tuning import load_tuning
from plrs_sim.types import Quaternion

_HEADING = 30.0
_LEVEL = Quaternion.identity()
_STILL = Vec3(x=0.0, y=0.0, z=0.0)
_GRAVITY = Vec3(x=0.0, y=0.0, z=9.81)


def _capture(duration_ms: int, imu_dt_ms: int, gnss_dt_ms: int) -> list[str]:
    """A synthetic capture: level, still, static GNSS heading."""
    lines: list[str] = []
    for t in range(0, duration_ms + 1, imu_dt_ms):
        lines.append(
            format_record(
                ImuRecord(
                    timestamp_ms=t,
                    orientation=_LEVEL,
                    angular_velocity_rad_s=_STILL,
                    accel_ms2=_GRAVITY,
                )
            )
        )
        if t % gnss_dt_ms == 0:
            lines.append(
                format_record(
                    GnssRecord(
                        timestamp_ms=t,
                        heading_deg=_HEADING,
                        heading_sigma_deg=0.3,
                        valid=True,
                    )
                )
            )
    return lines


def test_replay_drives_filter_toward_gnss_heading() -> None:
    trace = run(ReplaySource(lines=iter(_capture(5000, 10, 1000))), load_tuning())
    heading = trace.channels["heading"]
    # One Tick per I line, plus the GNSS reference series populated.
    assert len(trace.t_ms) == 501
    assert heading.measurement is not None and heading.measurement.size > 0
    # The filter converges on the replayed GNSS heading.
    assert abs(wrap180(float(heading.estimate[-1]) - _HEADING)) < 2.0


def test_replay_truth_is_nan() -> None:
    # A real capture has no ground truth; the runner records it as NaN.
    trace = run(ReplaySource(lines=iter(_capture(1000, 10, 1000))), load_tuning())
    assert np.all(np.isnan(trace.channels["heading"].truth))


def test_replay_ignores_unknown_lines() -> None:
    # Firmware F/M lines and diagnostics must not break replay or spawn Ticks.
    lines = [
        "# booting",
        "M,0,0.0,0.0,9.81,0.0,0.0,0.0,0.1,0.2,0.3",
        format_record(
            ImuRecord(
                timestamp_ms=0,
                orientation=_LEVEL,
                angular_velocity_rad_s=_STILL,
                accel_ms2=_GRAVITY,
            )
        ),
        "garbage,not,a,record",
    ]
    trace = run(ReplaySource(lines=iter(lines)), load_tuning())
    assert len(trace.t_ms) == 1
    assert math.isfinite(float(trace.channels["heading"].estimate[-1]))
