"""On-target telemetry monitor: parse, record, and visualize the RP2040 stream.

The firmware emits one tagged, comma-separated line per record over USB CDC
(see fusion_task.cpp). This module mirrors that wire format:

    F,ts_ms,heading,roll,pitch,hdg_sigma,roll_sigma,pitch_sigma   fused estimate
    I,ts_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az                         raw IMU
    G,ts_ms,heading,hdg_sigma,valid                               raw GNSS attitude
    # text                                                        human diagnostic

parse_line is tolerant: a malformed or partial line (a glitched byte, an
interleaved diagnostic) yields None rather than raising, so the monitor can
ride out a noisy link.
"""

from __future__ import annotations

import contextlib
import math
import time
from collections import deque
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

from .attitude import quaternion_to_euler_zyx
from .types import Quaternion, Vec3


@dataclass(frozen=True, slots=True, kw_only=True)
class FusionRecord:
    timestamp_ms: int
    heading_deg: float
    roll_deg: float
    pitch_deg: float
    heading_sigma_deg: float
    roll_sigma_deg: float
    pitch_sigma_deg: float


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuRecord:
    timestamp_ms: int
    orientation: Quaternion
    angular_velocity_rad_s: Vec3
    accel_ms2: Vec3


@dataclass(frozen=True, slots=True, kw_only=True)
class GnssRecord:
    timestamp_ms: int
    heading_deg: float
    heading_sigma_deg: float
    valid: bool


@dataclass(frozen=True, slots=True, kw_only=True)
class DiagRecord:
    text: str


Record = FusionRecord | ImuRecord | GnssRecord | DiagRecord

# Token counts including the leading tag.
_FIELD_COUNTS = {"F": 8, "I": 12, "G": 5}


def parse_line(line: str) -> Record | None:
    """Parse one telemetry line into a record, or None if it is not valid.

    A `#`-prefixed line is a diagnostic; anything else is dispatched on its
    tag. Wrong field counts and unparseable numbers return None.
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        return DiagRecord(text=line[1:].strip())

    fields = line.split(",")
    tag = fields[0]
    if len(fields) != _FIELD_COUNTS.get(tag, -1):
        return None

    try:
        if tag == "F":
            return FusionRecord(
                timestamp_ms=int(fields[1]),
                heading_deg=float(fields[2]),
                roll_deg=float(fields[3]),
                pitch_deg=float(fields[4]),
                heading_sigma_deg=float(fields[5]),
                roll_sigma_deg=float(fields[6]),
                pitch_sigma_deg=float(fields[7]),
            )
        if tag == "I":
            return ImuRecord(
                timestamp_ms=int(fields[1]),
                orientation=Quaternion(
                    w=float(fields[2]),
                    x=float(fields[3]),
                    y=float(fields[4]),
                    z=float(fields[5]),
                ),
                angular_velocity_rad_s=Vec3(
                    x=float(fields[6]), y=float(fields[7]), z=float(fields[8])
                ),
                accel_ms2=Vec3(
                    x=float(fields[9]), y=float(fields[10]), z=float(fields[11])
                ),
            )
        if tag == "G":
            return GnssRecord(
                timestamp_ms=int(fields[1]),
                heading_deg=float(fields[2]),
                heading_sigma_deg=float(fields[3]),
                valid=fields[4] == "1",
            )
    except ValueError:
        return None
    return None


def format_fusion(r: FusionRecord) -> str:
    return (
        f"F,{r.timestamp_ms},{r.heading_deg:.3f},{r.roll_deg:.3f},"
        f"{r.pitch_deg:.3f},{r.heading_sigma_deg:.3f},{r.roll_sigma_deg:.3f},"
        f"{r.pitch_sigma_deg:.3f}"
    )


def format_imu(r: ImuRecord) -> str:
    q, g, a = r.orientation, r.angular_velocity_rad_s, r.accel_ms2
    return (
        f"I,{r.timestamp_ms},{q.w:.5f},{q.x:.5f},{q.y:.5f},{q.z:.5f},"
        f"{g.x:.5f},{g.y:.5f},{g.z:.5f},{a.x:.4f},{a.y:.4f},{a.z:.4f}"
    )


def format_gnss(r: GnssRecord) -> str:
    return (
        f"G,{r.timestamp_ms},{r.heading_deg:.3f},{r.heading_sigma_deg:.3f},"
        f"{1 if r.valid else 0}"
    )


# Default rolling-buffer depth: 2000 samples at 10 Hz is ~200 s of history.
_DEFAULT_HISTORY = 2000

_RAD_TO_DEG = 180.0 / math.pi


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
    dead-reckon -- roll/pitch from the `I`-line quaternion, heading from
    integrating gyro-z off the first GNSS fix (same construction as
    runner.py's open-loop track). The two diverge as the gyro drifts and the
    EKF holds, which is the value of GNSS made visible. `last_gnss` /
    `last_diag` hold the most recent of each.
    """

    fused: Series = field(default_factory=Series)
    openloop: Series = field(default_factory=Series)
    last_gnss: GnssRecord | None = None
    last_diag: str | None = None
    latest_t_ms: int | None = None
    _dr_heading: float = 0.0
    _dr_seeded: bool = False
    _dr_prev_ms: int | None = None

    def update(self, rec: Record) -> None:
        if isinstance(rec, FusionRecord):
            self.fused.append(
                rec.timestamp_ms, rec.heading_deg, rec.roll_deg, rec.pitch_deg
            )
            self.latest_t_ms = rec.timestamp_ms
        elif isinstance(rec, ImuRecord):
            roll, pitch, _yaw = quaternion_to_euler_zyx(rec.orientation)
            # Dead-reckon heading by integrating gyro-z, but only once seeded by
            # a GNSS fix -- before that the absolute heading is unknown.
            if self._dr_seeded and self._dr_prev_ms is not None:
                dt_s = (rec.timestamp_ms - self._dr_prev_ms) / 1000.0
                self._dr_heading += rec.angular_velocity_rad_s.z * _RAD_TO_DEG * dt_s
            self._dr_prev_ms = rec.timestamp_ms
            self.openloop.append(rec.timestamp_ms, self._dr_heading, roll, pitch)
            self.latest_t_ms = rec.timestamp_ms
        elif isinstance(rec, GnssRecord):
            self.last_gnss = rec
            if rec.valid and not self._dr_seeded:
                self._dr_heading = rec.heading_deg
                self._dr_seeded = True
        elif isinstance(rec, DiagRecord):
            self.last_diag = rec.text

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


def replay_file(path: Path) -> Iterator[str]:
    """Yield lines from a previously captured telemetry file."""
    with path.open() as f:
        yield from f


def serial_lines(port: str, baud: int = 115200) -> Iterator[str]:
    """Yield decoded lines from a serial port, opened passively.

    DTR/RTS are deasserted before open and no 1200-bps touch is issued, so the
    RP2040's USB CDC is not reset/re-enumerated (see CLAUDE.md Workarounds).
    The production firmware emits unconditionally, so passive reading suffices.
    """
    import serial

    ser = serial.serial_for_url(port, baudrate=baud, do_not_open=True)
    ser.dtr = False
    ser.rts = False
    ser.timeout = 1.0
    ser.open()
    try:
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("ascii", errors="replace")
    finally:
        ser.close()


def _tee(lines: Iterable[str], sink: TextIO | None) -> Iterator[str]:
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
        rec = parse_line(line)
        if rec is None:
            continue
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


def monitor(
    lines: Iterable[str],
    *,
    record: Path | None = None,
    show: bool = True,
    summary_interval_ms: int = 1000,
) -> MonitorState:
    """Drive the telemetry stream: record losslessly, then view or summarize.

    `lines` is any iterable of raw telemetry lines -- a live serial reader, a
    replay file, or a test list -- so every entry point shares this code path.
    Returns the accumulated state for inspection.
    """
    state = MonitorState()
    sink_cm: contextlib.AbstractContextManager[TextIO | None]
    if record is not None:
        record.parent.mkdir(parents=True, exist_ok=True)
        sink_cm = record.open("w")
    else:
        sink_cm = contextlib.nullcontext(None)

    with sink_cm as sink:
        teed = _tee(lines, sink)
        if show:
            _run_live(teed, state)
        else:
            _run_headless(teed, state, summary_interval_ms)
    return state


def _drain(lines: Iterable[str], state: MonitorState) -> None:
    """Consume the line stream into state; runs on the reader thread."""
    for line in lines:
        rec = parse_line(line)
        if rec is not None:
            state.update(rec)


def _draw_boat_panel(ax, state: MonitorState) -> None:
    from .boat3d import draw_boat, set_equal_3d

    ax.cla()
    if state.openloop.t_ms:
        draw_boat(
            ax,
            state.openloop.roll[-1],
            state.openloop.pitch[-1],
            state.openloop.heading[-1],
            color="tab:green",
            alpha=0.5,
            label="open-loop",
        )
    if state.fused.t_ms:
        draw_boat(
            ax,
            state.fused.roll[-1],
            state.fused.pitch[-1],
            state.fused.heading[-1],
            color="tab:orange",
            label="fused",
        )
    set_equal_3d(ax)
    ax.legend(loc="upper left", fontsize=8)


def _draw_scroll(ax, state: MonitorState, which: str, window_s: float) -> None:
    import numpy as np

    ax.cla()
    t_max = (state.latest_t_ms or 0) / 1000.0
    for series, color, label in (
        (state.fused, "tab:orange", "fused"),
        (state.openloop, "tab:green", "open-loop"),
    ):
        if not series.t_ms:
            continue
        t = np.array(series.t_ms, dtype=float) / 1000.0
        ax.plot(
            t, np.array(getattr(series, which), dtype=float), color=color, label=label
        )
    ax.set_xlim(max(0.0, t_max - window_s), max(window_s, t_max))
    ax.set_ylabel(f"{which} [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=7)


def _run_live(
    lines: Iterable[str],
    state: MonitorState,
    *,
    redraw_interval_ms: int = 80,
    window_s: float = 20.0,
) -> None:
    """Open a Qt window: 3D boat (raw vs fused) plus scrolling angle plots.

    The line stream is drained on a background thread so a blocking serial
    read or a paced replay never stalls the GUI; the animation timer redraws
    from `state` at its own rate. QtAgg is forced because the devshell pins
    MPLBACKEND to the kitty backend, which cannot host a live figure.
    """
    import threading

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    try:
        plt.switch_backend("QtAgg")
    except Exception as e:  # surface any GUI-backend failure plainly
        raise RuntimeError(
            "live view needs a GUI backend (QtAgg); none available. "
            "Run outside a headless shell, or use --no-show to record only."
        ) from e

    reader = threading.Thread(target=_drain, args=(lines, state), daemon=True)
    reader.start()

    fig = plt.figure(figsize=(11, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_h = fig.add_subplot(3, 2, 2)
    ax_r = fig.add_subplot(3, 2, 4)
    ax_p = fig.add_subplot(3, 2, 6)

    def update(_frame: int) -> None:
        _draw_boat_panel(ax3d, state)
        _draw_scroll(ax_h, state, "heading", window_s)
        _draw_scroll(ax_r, state, "roll", window_s)
        _draw_scroll(ax_p, state, "pitch", window_s)
        if state.last_diag is not None:
            fig.suptitle(f"# {state.last_diag}", fontsize=9)

    # Held in a local so the animation is not garbage-collected before show().
    anim = FuncAnimation(  # noqa: F841
        fig, update, interval=redraw_interval_ms, cache_frame_data=False
    )
    plt.show()


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
