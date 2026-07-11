"""Monitor state and the line sources that feed it.

`MonitorState` accumulates the parsed telemetry stream into rolling attitude
histories plus latest scalars, thread-safely, so a GUI can snapshot it while a
reader thread appends. The line sources (serial, replay, pacing) and the
headless summary path live here too; the interactive views are in views.py.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable, Generator, Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

from .angles import wrap180
from .attitude import quaternion_to_euler_zyx
from .deadreckon import HeadingDeadReckoner
from .telemetry import (
    DiagRecord,
    FusionRecord,
    GnssRecord,
    ImuRecord,
    Record,
    parse_line,
)

# Default rolling-buffer depth: 2000 samples at 10 Hz is ~200 s of history.
_DEFAULT_HISTORY = 2000


def heading_offset_deg(gnss_deg: float, imu_deg: float) -> float:
    """GNSS-minus-IMU heading residual, mapped into (-180, 180]."""
    return float(wrap180(gnss_deg - imu_deg))


@dataclass(slots=True)
class Series:
    """A rolling (time, heading, roll, pitch) history for one attitude source."""

    t_ms: deque[int] = field(default_factory=lambda: deque(maxlen=_DEFAULT_HISTORY))
    heading: deque[float] = field(
        default_factory=lambda: deque(maxlen=_DEFAULT_HISTORY)
    )
    roll: deque[float] = field(default_factory=lambda: deque(maxlen=_DEFAULT_HISTORY))
    pitch: deque[float] = field(default_factory=lambda: deque(maxlen=_DEFAULT_HISTORY))

    def append(self, t_ms: int, heading: float, roll: float, pitch: float) -> None:
        self.t_ms.append(t_ms)
        self.heading.append(heading)
        self.roll.append(roll)
        self.pitch.append(pitch)


@dataclass(slots=True)
class MonitorState:
    """Live view of the stream: rolling attitude histories plus latest scalars.

    `fused` is the EKF estimate (`F` lines); `openloop` is the IMU-only
    dead-reckon -- roll/pitch from the `I`-line quaternion, heading seeded from
    the first fused heading (so the two tracks start identical and any later
    separation is pure gyro drift) and then propagated by integrating gyro-z. A
    valid GNSS fix re-anchors that heading to the absolute reference once it
    arrives; without GNSS it still tracks (relative) so the bench is usable. The
    two tracks diverge as the gyro drifts and the EKF holds, which is the value
    of the EKF made visible. `last_gnss` / `last_diag` hold the most recent of
    each.
    """

    fused: Series = field(default_factory=Series)
    openloop: Series = field(default_factory=Series)
    last_gnss: GnssRecord | None = None
    last_diag: str | None = None
    latest_t_ms: int | None = None
    lines_seen: int = 0
    lines_parsed: int = 0
    last_error: str | None = None
    # Guards the series against concurrent append (drain thread) and read (GUI
    # thread); without it a snapshot can catch a half-appended sample.
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _dead_reckoner: HeadingDeadReckoner = field(
        default_factory=HeadingDeadReckoner, repr=False
    )
    _dr_anchored_to_gnss: bool = False

    def update(self, rec: Record) -> None:
        with self._lock:
            self._update(rec)

    def _update(self, rec: Record) -> None:
        if isinstance(rec, FusionRecord):
            self.fused.append(
                rec.timestamp_ms, rec.heading_deg, rec.roll_deg, rec.pitch_deg
            )
            self.latest_t_ms = rec.timestamp_ms
        elif isinstance(rec, ImuRecord):
            roll, pitch, yaw = quaternion_to_euler_zyx(rec.orientation)
            # Seed the dead-reckon heading from the first fused heading so the
            # two tracks start on the same absolute reference; fall back to the
            # boot yaw if no fused sample has arrived yet. A GNSS fix, if any,
            # re-anchors it later.
            if not self._dead_reckoner.anchored and not self._dr_anchored_to_gnss:
                self._dead_reckoner.anchor(
                    self.fused.heading[-1] if self.fused.t_ms else yaw
                )
            heading = self._dead_reckoner.step(
                rec.timestamp_ms, rec.angular_velocity_rad_s.z
            )
            assert heading is not None  # anchored just above, or already by GNSS
            self.openloop.append(rec.timestamp_ms, heading, roll, pitch)
            self.latest_t_ms = rec.timestamp_ms
        elif isinstance(rec, GnssRecord):
            self.last_gnss = rec
            # Re-anchor the relative dead-reckon to the absolute heading once.
            if rec.valid and not self._dr_anchored_to_gnss:
                self._dead_reckoner.anchor(rec.heading_deg)
                self._dr_anchored_to_gnss = True
        elif isinstance(rec, DiagRecord):
            self.last_diag = rec.text

    def snapshot(self, which: str):
        """Atomically copy the fused and open-loop (time, `which`) arrays plus the
        latest timestamp, consistent against the drain thread's appends.

        Returns `(t_now_s, [(t_s, values), ...])` with fused first, then
        open-loop. Taken under the lock so a series can never be read mid-append.
        """
        import numpy as np

        with self._lock:
            t_now = (self.latest_t_ms or 0) / 1000.0
            arrays = [
                (
                    np.fromiter(series.t_ms, dtype=float) / 1000.0,
                    np.fromiter(getattr(series, which), dtype=float),
                )
                for series in (self.fused, self.openloop)
            ]
        return t_now, arrays

    def status_line(self) -> str:
        """Drain health for the viewer banner: distinguishes a dead port (rx 0)
        from a parse failure (rx high, parsed 0) from a working stream."""
        age = (
            f"t={self.latest_t_ms / 1000:.1f}s"
            if self.latest_t_ms is not None
            else "no data"
        )
        status = f"rx {self.lines_seen} | parsed {self.lines_parsed} | {age}"
        if self.last_error is not None:
            status += f" | ERROR: {self.last_error}"
        return status

    def summary_line(self) -> str:
        """One-line snapshot for headless monitoring."""
        t_s = (self.latest_t_ms or 0) / 1000.0
        fused = (
            f"fused hdg/roll/pitch={self.fused.heading[-1]:7.2f}/"
            f"{self.fused.roll[-1]:7.2f}/{self.fused.pitch[-1]:7.2f}"
            if self.fused.t_ms
            else "fused --"
        )
        oloop = (
            f"o-loop {self.openloop.heading[-1]:7.2f}/{self.openloop.roll[-1]:7.2f}/"
            f"{self.openloop.pitch[-1]:7.2f}"
            if self.openloop.t_ms
            else "o-loop --"
        )
        gnss = (
            f"gnss hdg={self.last_gnss.heading_deg:7.2f}"
            if self.last_gnss and self.last_gnss.valid
            else "gnss --"
        )
        return f"t={t_s:8.2f}s  {fused}  {oloop}  {gnss}"

    def alignment_summary(self) -> str:
        """Sensor-alignment residuals: IMU roll/pitch and the heading offset.

        Roll and pitch are absolute from the IMU; the heading offset is
        GNSS-minus-IMU, shown only while the fix is valid.
        """
        if not self.openloop.t_ms:
            return "waiting for IMU..."
        roll, pitch = self.openloop.roll[-1], self.openloop.pitch[-1]
        if self.last_gnss is not None and self.last_gnss.valid:
            off = heading_offset_deg(
                self.last_gnss.heading_deg, self.openloop.heading[-1]
            )
            offset = f"heading offset (GNSS-IMU)={off:+6.2f}"
        else:
            offset = "heading offset --"
        return f"IMU roll={roll:+6.2f}  pitch={pitch:+6.2f}  |  {offset}"


def replay_file(path: Path) -> Iterator[str]:
    """Yield lines from a previously captured telemetry file."""
    with path.open() as f:
        yield from f


def serial_lines(port: str, baud: int = 115200) -> Iterator[str]:
    """Yield decoded lines from a serial port.

    DTR is asserted: the RP2040's USB CDC gates transmission on the host being
    connected, so it stays silent with DTR low (see CLAUDE.md Workarounds). A
    steady DTR at the data baud rate does not reset the board -- only the
    1200-bps bootloader touch does, which is never issued here.
    """
    import serial

    ser = serial.serial_for_url(port, baudrate=baud, do_not_open=True)
    ser.dtr = True
    ser.rts = True
    ser.timeout = 1.0
    ser.open()
    try:
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("ascii", errors="replace")
    finally:
        ser.close()


def _tee(lines: Iterable[str], sink: TextIO | None) -> Generator[str, None, None]:
    """Write each raw line to sink (one normalized line) and pass it through."""
    for line in lines:
        if sink is not None:
            sink.write(line.rstrip("\r\n") + "\n")
            sink.flush()
        yield line


def _run_headless(
    lines: Iterable[str], state: MonitorState, summary_interval_ms: int
) -> None:
    """Consume the stream, printing diagnostics live and a throttled summary."""
    last_summary: int | None = None
    for line in lines:
        state.lines_seen += 1
        rec = parse_line(line)
        if rec is None:
            continue
        state.lines_parsed += 1
        state.update(rec)
        if isinstance(rec, DiagRecord):
            print(f"# {rec.text}")
            continue
        t = state.latest_t_ms
        if t is not None and (
            last_summary is None or t - last_summary >= summary_interval_ms
        ):
            print(state.summary_line())
            last_summary = t


def pace(
    lines: Iterable[str],
    *,
    speed: float = 1.0,
    sleep: Callable[[float], None] = time.sleep,
) -> Iterator[str]:
    """Re-time a capture to its own timestamps, for real-time replay.

    Sleeps so line N emerges `(ts_N - ts_0) / speed` seconds after the first
    timestamped line. Untimestamped lines (diagnostics) pass through with no
    delay. `speed > 1` replays faster than real time; `sleep` is injectable
    for tests.
    """
    start_wall: float | None = None
    first_ts: int | None = None
    for line in lines:
        rec = parse_line(line)
        ts = getattr(rec, "timestamp_ms", None)
        if ts is not None:
            if first_ts is None:
                first_ts, start_wall = ts, time.monotonic()
            else:
                assert start_wall is not None
                target = (ts - first_ts) / 1000.0 / speed
                elapsed = time.monotonic() - start_wall
                if target > elapsed:
                    sleep(target - elapsed)
        yield line
