"""Interactive matplotlib/Qt views over a MonitorState.

Two live views: the scrolling heading/roll/pitch angle plots (`_run_live`,
blitted for smoothness) and the 3D sensor-alignment view (`_run_align`). A
daemon thread drains the line stream into the shared state while the GUI reads
snapshots. See CLAUDE.md Workarounds for why the angle view is blitted and the
3D view is not.
"""

from __future__ import annotations

import contextlib
from collections.abc import Callable, Iterable

from .monitor import MonitorState
from .telemetry import parse_line


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
