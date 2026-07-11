"""On-target telemetry monitor: parse, record, and visualize the RP2040 stream.

The firmware emits one tagged, comma-separated line per record over USB CDC
(see fusion_task.cpp). This module mirrors that wire format:

    F,ts_ms,heading,roll,pitch,hdg_sigma,roll_sigma,pitch_sigma   fused estimate
    I,ts_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az                         raw IMU
    M,ts_ms,ax,ay,az,gx,gy,gz,mx,my,mz                            raw MEMS triad
    G,ts_ms,heading,hdg_sigma,valid,mode,error                    raw GNSS attitude
    # text                                                        human diagnostic

parse_line is tolerant: a malformed or partial line (a glitched byte, an
interleaved diagnostic) yields None rather than raising, so the monitor can
ride out a noisy link.
"""

from __future__ import annotations

import contextlib
import math
import threading
import time
from collections import deque
from collections.abc import Callable, Generator, Iterable, Iterator, Sequence
from dataclasses import MISSING, dataclass, field, fields
from pathlib import Path
from typing import Annotated, TextIO, get_args, get_origin, get_type_hints

from .attitude import quaternion_to_euler_zyx
from .types import Quaternion, Vec3

# The record dataclasses ARE the wire schema: field order is token order,
# Vec3/Quaternion fields flatten to their components, the Annotated metadata
# is the wire precision, and a trailing run of defaulted fields is a
# format-version boundary (all present or all absent). parse_line,
# format_record, and the accepted field counts all derive from them, so the
# three cannot drift apart.

_Deg = Annotated[float, 3]
_Quat = Annotated[Quaternion, 5]
_Gyro = Annotated[Vec3, 5]
_Accel = Annotated[Vec3, 4]
_Mag = Annotated[Vec3, 5]


@dataclass(frozen=True, slots=True, kw_only=True)
class FusionRecord:
    timestamp_ms: int
    heading_deg: _Deg
    roll_deg: _Deg
    pitch_deg: _Deg
    heading_sigma_deg: _Deg
    roll_sigma_deg: _Deg
    pitch_sigma_deg: _Deg


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuRecord:
    timestamp_ms: int
    orientation: _Quat
    angular_velocity_rad_s: _Gyro
    accel_ms2: _Accel


@dataclass(frozen=True, slots=True, kw_only=True)
class MemsRecord:
    timestamp_ms: int
    accel_ms2: _Accel
    angular_velocity_rad_s: _Gyro
    magnetic_field_au: _Mag


@dataclass(frozen=True, slots=True, kw_only=True)
class GnssRecord:
    timestamp_ms: int
    heading_deg: _Deg
    heading_sigma_deg: _Deg
    valid: bool
    # Raw AttEuler mode/error, for diagnosing why valid is false (float
    # ambiguity vs no-attitude vs a flagged baseline). Default to the
    # no-solution codes so older logs without these fields still parse.
    mode: int = 0
    error: int = 0


@dataclass(frozen=True, slots=True, kw_only=True)
class DiagRecord:
    text: str


Record = FusionRecord | ImuRecord | MemsRecord | GnssRecord | DiagRecord

_NESTED_COMPONENTS = {Vec3: ("x", "y", "z"), Quaternion: ("w", "x", "y", "z")}


def _parse_float(token: str) -> float:
    # The firmware prints out-of-range floats as "ovf" (Arduino Print).
    return math.inf if token == "ovf" else float(token)


def _format_float(v: float, prec: int) -> str:
    return "ovf" if not math.isfinite(v) else f"{v:.{prec}f}"


@dataclass(frozen=True, slots=True)
class _FieldCodec:
    """Wire layout of one record field: `width` consecutive tokens."""

    name: str
    width: int
    parse: Callable[[Sequence[str]], object]
    unparse: Callable[[object], list[str]]
    optional: bool


def _field_codec(name: str, hint: type, optional: bool) -> _FieldCodec:
    t, prec = (
        (get_args(hint)[0], get_args(hint)[1])
        if get_origin(hint) is Annotated
        else (hint, None)
    )
    if t in _NESTED_COMPONENTS:
        comps = _NESTED_COMPONENTS[t]
        return _FieldCodec(
            name,
            len(comps),
            lambda toks: t(
                **{c: _parse_float(k) for c, k in zip(comps, toks, strict=True)}
            ),
            lambda v: [_format_float(getattr(v, c), prec) for c in comps],
            optional,
        )
    if t is float:
        return _FieldCodec(
            name,
            1,
            lambda toks: _parse_float(toks[0]),
            lambda v: [_format_float(v, prec)],
            optional,
        )
    if t is bool:
        return _FieldCodec(
            name,
            1,
            lambda toks: toks[0] == "1",
            lambda v: ["1" if v else "0"],
            optional,
        )
    if t is int:
        return _FieldCodec(
            name, 1, lambda toks: int(toks[0]), lambda v: [str(v)], optional
        )
    raise TypeError(f"unsupported wire field type: {t!r}")


@dataclass(frozen=True, slots=True)
class _RecordCodec:
    cls: type
    fields: tuple[_FieldCodec, ...]
    widths: tuple[int, ...]  # accepted token counts, including the tag


def _record_codec(cls: type) -> _RecordCodec:
    hints = get_type_hints(cls, include_extras=True)
    codecs = tuple(
        _field_codec(f.name, hints[f.name], f.default is not MISSING)
        for f in fields(cls)
    )
    full = 1 + sum(f.width for f in codecs)
    required = full - sum(f.width for f in codecs if f.optional)
    widths = (required,) if required == full else (required, full)
    return _RecordCodec(cls, codecs, widths)


_CODECS = {
    "F": _record_codec(FusionRecord),
    "I": _record_codec(ImuRecord),
    "M": _record_codec(MemsRecord),
    "G": _record_codec(GnssRecord),
}
_TAGS = {codec.cls: tag for tag, codec in _CODECS.items()}


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

    tokens = line.split(",")
    codec = _CODECS.get(tokens[0])
    if codec is None or len(tokens) not in codec.widths:
        return None

    kwargs = {}
    at = 1
    try:
        for fc in codec.fields:
            if at >= len(tokens):
                break  # absent optional tail keeps its defaults
            kwargs[fc.name] = fc.parse(tokens[at : at + fc.width])
            at += fc.width
        return codec.cls(**kwargs)
    except ValueError:
        return None


def format_record(r: FusionRecord | ImuRecord | MemsRecord | GnssRecord) -> str:
    """Render a record as its wire line, the inverse of parse_line."""
    tag = _TAGS[type(r)]
    tokens = [tag]
    for fc in _CODECS[tag].fields:
        tokens += fc.unparse(getattr(r, fc.name))
    return ",".join(tokens)


# Default rolling-buffer depth: 2000 samples at 10 Hz is ~200 s of history.
_DEFAULT_HISTORY = 2000

_RAD_TO_DEG = 180.0 / math.pi


def wrap180(deg: float) -> float:
    """Wrap an angle in degrees into (-180, 180], matching the firmware."""
    return math.remainder(deg, 360.0)


def heading_offset_deg(gnss_deg: float, imu_deg: float) -> float:
    """GNSS-minus-IMU heading residual, mapped into (-180, 180]."""
    return wrap180(gnss_deg - imu_deg)


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
    arrives;
    without GNSS it still tracks (relative) so the bench is usable. The two
    tracks diverge as the gyro drifts and the EKF holds, which is the value of
    the EKF made visible. `last_gnss` / `last_diag` hold the most recent of each.
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
    _dr_heading: float = 0.0
    _dr_prev_ms: int | None = None
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
            # two tracks start on the same absolute reference, then propagate it
            # by integrating gyro-z. Fall back to the boot yaw if no fused sample
            # has arrived yet; a GNSS fix, if any, re-anchors it later.
            if self._dr_prev_ms is None:
                if not self._dr_anchored_to_gnss:
                    self._dr_heading = (
                        self.fused.heading[-1] if self.fused.t_ms else yaw
                    )
            else:
                dt_s = (rec.timestamp_ms - self._dr_prev_ms) / 1000.0
                # Wrap to (-180, 180] like the firmware so the open-loop track
                # crosses the seam together with the fused one, not past it.
                self._dr_heading = wrap180(
                    self._dr_heading - rec.angular_velocity_rad_s.z * _RAD_TO_DEG * dt_s
                )
            self._dr_prev_ms = rec.timestamp_ms
            self.openloop.append(rec.timestamp_ms, self._dr_heading, roll, pitch)
            self.latest_t_ms = rec.timestamp_ms
        elif isinstance(rec, GnssRecord):
            self.last_gnss = rec
            # Re-anchor the relative dead-reckon to the absolute heading once.
            if rec.valid and not self._dr_anchored_to_gnss:
                self._dr_heading = rec.heading_deg
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


def monitor(
    lines: Iterable[str],
    *,
    record: Path | None = None,
    show: bool = True,
    align: bool = False,
    summary_interval_ms: int = 1000,
) -> MonitorState:
    """Drive the telemetry stream: record losslessly, then view or summarize.

    `lines` is any iterable of raw telemetry lines -- a live serial reader, a
    replay file, or a test list -- so every entry point shares this code path.
    `align` swaps the drift view for the sensor-alignment view. Returns the
    accumulated state for inspection.
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
            (_run_align if align else _run_live)(teed, state)
        else:
            _run_headless(teed, state, summary_interval_ms)
    return state


def _drain(lines: Iterable[str], state: MonitorState) -> None:
    """Consume the line stream into state; runs on the reader thread.

    A reader-thread exception would otherwise vanish silently, leaving the
    viewer frozen with no clue; record it on the shared state so the banner
    shows it, and dump the traceback for the full detail.
    """
    try:
        for line in lines:
            state.lines_seen += 1
            rec = parse_line(line)
            if rec is not None:
                state.lines_parsed += 1
                state.update(rec)
    except Exception as exc:
        import traceback

        traceback.print_exc()
        state.last_error = f"drain: {exc}"


# Initial y-ranges per scroll panel. Heading spans the full circle and never
# grows; roll/pitch start tight and the panel widens them as the data demands.
_SCROLL_INITIAL_YLIM = {
    "heading": (0.0, 360.0),
    "roll": (-15.0, 15.0),
    "pitch": (-15.0, 15.0),
}


class _ScrollPanel:
    """One scrolling angle panel, built for blitting.

    The x-axis is *relative* time ("seconds ago", fixed [-window, 0]) so the
    ticks never move and the axis frame can stay a cached background while only
    the lines are blitted. The lines are `animated` so the blit machinery
    excludes them from that background. y-limits only ever grow, so they settle
    quickly and rarely force a frame redraw.
    """

    def __init__(self, ax, which: str, window_s: float) -> None:
        self.ax = ax
        self.which = which
        self.window_s = window_s
        (self.fused_line,) = ax.plot(
            [], [], color="tab:orange", label="fused", animated=True
        )
        (self.open_line,) = ax.plot(
            [], [], color="tab:green", label="open-loop", animated=True
        )
        ax.set_xlim(-window_s, 0.0)
        ax.set_ylim(*_SCROLL_INITIAL_YLIM[which])
        ax.set_xlabel("seconds ago")
        ax.set_ylabel(f"{which} [deg]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)

    @property
    def lines(self) -> tuple:
        return (self.fused_line, self.open_line)

    def update(self, state: MonitorState) -> bool:
        """Set both lines to the visible window. Returns True if the y-limits had
        to grow (so the caller must redraw this axis frame, not just blit)."""
        t_now, arrays = state.snapshot(self.which)
        ymin = ymax = None
        for (t, values), line in zip(arrays, self.lines, strict=True):
            if t.size == 0:
                continue
            rel = t - t_now
            mask = rel >= -self.window_s
            y = values[mask]
            line.set_data(rel[mask], y)
            if y.size:
                ymin = y.min() if ymin is None else min(ymin, y.min())
                ymax = y.max() if ymax is None else max(ymax, y.max())
        if ymin is None or ymax is None:
            return False
        lo, hi = self.ax.get_ylim()
        if ymin < lo or ymax > hi:
            pad = max(5.0, 0.1 * (ymax - ymin))
            self.ax.set_ylim(min(lo, ymin - pad), max(hi, ymax + pad))
            return True
        return False


def _draw_align_panel(ax, state: MonitorState) -> None:
    from .boat3d import draw_boat, draw_heading_arrow, draw_triad, set_equal_3d

    ax.cla()
    # Level reference hull: the boat frame both sensors are aligned against.
    draw_boat(ax, 0.0, 0.0, 0.0, color="0.6", label="boat (level)")
    if state.openloop.t_ms:
        draw_triad(
            ax,
            state.openloop.roll[-1],
            state.openloop.pitch[-1],
            state.openloop.heading[-1],
            labels=("IMU X", "IMU Y", "IMU Z"),
        )
    if state.last_gnss is not None:
        # Grey a stale fix so a frozen heading is not read as live agreement.
        valid = state.last_gnss.valid
        draw_heading_arrow(
            ax,
            state.last_gnss.heading_deg,
            color="tab:purple" if valid else "0.7",
            alpha=1.0 if valid else 0.4,
            label="GNSS heading",
        )
    set_equal_3d(ax)
    ax.legend(loc="upper left", fontsize=8)


def _ensure_qt_backend() -> None:
    """Force QtAgg: the devshell pins MPLBACKEND to the kitty backend, which
    cannot host a live (continuously redrawn) figure."""
    import matplotlib.pyplot as plt

    try:
        plt.switch_backend("QtAgg")
    except Exception as e:  # surface any GUI-backend failure plainly
        raise RuntimeError(
            "live view needs a GUI backend (QtAgg); none available. "
            "Run outside a headless shell, or use --no-show to record only."
        ) from e


@contextlib.contextmanager
def _drain_thread(lines: Iterable[str], state: MonitorState):
    """Drain the stream on a daemon thread for the duration of the block.

    A blocking serial read or a paced replay never stalls the GUI this way. On
    exit the source is closed so the generator chain (including any C++ objects)
    is freed before interpreter shutdown.
    """
    import threading

    reader = threading.Thread(target=_drain, args=(lines, state), daemon=True)
    reader.start()
    try:
        yield
    finally:
        with contextlib.suppress(Exception):
            lines.close()  # type: ignore[attr-defined]
        reader.join(timeout=1.0)


def _animate(
    lines: Iterable[str],
    state: MonitorState,
    build_figure: Callable[[], tuple],
    *,
    redraw_interval_ms: int = 200,
) -> None:
    """Open a Qt window and animate from `state` while a thread drains `lines`.

    `build_figure` is called once the GUI backend is live and returns
    `(figure, update)`, where `update(frame)` redraws from the shared state.
    Used by the all-3D align view, which cannot be blitted; the live view drives
    its own blit loop instead (see _run_live).
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    _ensure_qt_backend()
    with _drain_thread(lines, state):
        fig, update = build_figure()
        # Held in a local so the animation is not garbage-collected before show().
        anim = FuncAnimation(  # noqa: F841
            fig, update, interval=redraw_interval_ms, cache_frame_data=False
        )
        plt.show()


class _LiveBlitView:
    """Drive the live figure by blitting the scroll panels every tick.

    Blitting (restore a cached axis background, draw just the moving line, push
    the pixels) costs ~7 ms for all three panels versus ~230 ms for a full
    figure redraw, so the view stays smooth without pegging the event loop. A
    panel whose y-limits had to grow needs one real redraw to refresh its ticks;
    that also re-captures the backgrounds via the draw_event handler.
    """

    def __init__(self, fig, scrolls: list[_ScrollPanel], state: MonitorState) -> None:
        self.fig = fig
        self.canvas = fig.canvas
        self.scrolls = scrolls
        self.state = state
        self._bgs: dict = {}
        self.canvas.mpl_connect("draw_event", self._capture_backgrounds)

    def _capture_backgrounds(self, _event=None) -> None:
        self._bgs = {p.ax: self.canvas.copy_from_bbox(p.ax.bbox) for p in self.scrolls}

    def tick(self) -> None:
        try:
            self._tick()
        except Exception as exc:
            import traceback

            traceback.print_exc()
            self.state.last_error = f"draw: {exc}"

    def _tick(self) -> None:
        if not self._bgs:
            self.canvas.draw()  # first paint -> draw_event captures backgrounds
            return

        if any([p.update(self.state) for p in self.scrolls]):
            # A y-limit expanded: a real redraw refreshes the ticks (and, via the
            # draw_event handler, the cached backgrounds).
            self.canvas.draw()
        else:
            for p in self.scrolls:
                self.canvas.restore_region(self._bgs[p.ax])
                for line in p.lines:
                    p.ax.draw_artist(line)
                self.canvas.blit(p.ax.bbox)

        self.canvas.flush_events()
        self._set_status_title()

    def _set_status_title(self) -> None:
        # The status banner goes in the window title: it is always current and
        # costs nothing, where an on-canvas banner would need its own blit.
        manager = getattr(self.canvas, "manager", None)
        if manager is None:
            return
        title = self.state.status_line()
        if self.state.last_diag is not None:
            title += f"  # {self.state.last_diag}"
        manager.set_window_title(title)


def _run_live(
    lines: Iterable[str],
    state: MonitorState,
    *,
    scroll_interval_ms: int = 33,
    window_s: float = 20.0,
) -> None:
    """Open a Qt window with the heading/roll/pitch scrolling angle plots."""
    import matplotlib.pyplot as plt

    _ensure_qt_backend()
    with _drain_thread(lines, state):
        fig = plt.figure(figsize=(9, 7))
        scrolls = [
            _ScrollPanel(fig.add_subplot(3, 1, 1), "heading", window_s),
            _ScrollPanel(fig.add_subplot(3, 1, 2), "roll", window_s),
            _ScrollPanel(fig.add_subplot(3, 1, 3), "pitch", window_s),
        ]
        view = _LiveBlitView(fig, scrolls, state)

        timer = fig.canvas.new_timer(interval=scroll_interval_ms)
        timer.add_callback(view.tick)
        timer.start()
        plt.show()
        timer.stop()


def _run_align(
    lines: Iterable[str], state: MonitorState, *, redraw_interval_ms: int = 80
) -> None:
    """Open a Qt window for physical sensor alignment.

    A level reference hull carries the live IMU axis triad and the GNSS
    heading arrow; the readout shows the residuals (IMU roll/pitch, which are
    absolute, and the GNSS-minus-IMU heading offset) to drive toward zero by
    physically adjusting the mounts.
    """
    import matplotlib.pyplot as plt

    def build() -> tuple:
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(projection="3d")

        def update(_frame: int) -> None:
            try:
                _draw_align_panel(ax, state)
            except Exception as exc:
                import traceback

                traceback.print_exc()
                state.last_error = f"draw: {exc}"
            fig.suptitle(
                f"{state.alignment_summary()}\n{state.status_line()}", fontsize=10
            )

        return fig, update

    _animate(lines, state, build, redraw_interval_ms=redraw_interval_ms)


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
