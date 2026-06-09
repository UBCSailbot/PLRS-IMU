"""Trace plotting.

For each channel asked for, two stacked subplots:
- Top: trajectory view. Truth, open-loop integration, measurements, and
  EKF estimate, all overlaid.
- Bottom: residual view. EKF estimate error against truth with the
  filter's claimed +/-1 sigma band centered at zero, plus measurement
  error scattered. The band is the load-bearing element here -- residuals
  staying inside the band mean the filter is consistent with its claimed
  uncertainty; residuals escaping mean overconfidence.

Headless callers should set MPLBACKEND=Agg in the environment before
importing this module.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .types import Channel, Trace

_MEASUREMENT_SCATTER_ALPHA = 0.4


def _wrap180(deg: np.ndarray) -> np.ndarray:
    """Map angles (or angle differences) into (-180, 180]."""
    return (np.asarray(deg, dtype=float) + 180.0) % 360.0 - 180.0


def _seam_broken(deg: np.ndarray) -> np.ndarray:
    """Wrap to +-180 and NaN out the sample after each seam crossing so the
    line plot does not draw a vertical connector across the +-180 jump."""
    wrapped = _wrap180(deg)
    out = wrapped.copy()
    jumped = np.abs(np.diff(wrapped)) > 180.0
    out[1:][jumped] = np.nan
    return out


def plot_trace(
    trace: Trace,
    *,
    channels: list[str] | None = None,
    show: bool = True,
    save: Path | None = None,
    title: str | None = None,
) -> None:
    names = channels if channels is not None else list(trace.channels.keys())
    n = len(names)
    fig, axes = plt.subplots(
        2 * n,
        1,
        figsize=(10, 3.5 * n + 0.5),
        sharex=True,
        gridspec_kw={"height_ratios": [2, 1] * n},
    )
    axes = np.atleast_1d(axes)
    t_s = trace.t_ms / 1000.0

    for i, name in enumerate(names):
        ax_traj = axes[2 * i]
        ax_res = axes[2 * i + 1]
        _plot_channel(ax_traj, ax_res, t_s, trace.t_ms, trace.channels[name])
        if i == 0 and title is not None:
            ax_traj.set_title(title)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)


def _plot_channel(ax_traj, ax_res, t_s, t_ms, ch: Channel) -> None:
    # For a circular channel, display every series in the same +-180 frame and
    # break the lines at the seam; otherwise plot the raw values.
    line = _seam_broken if ch.wrap else (lambda x: x)
    ax_traj.plot(t_s, line(ch.truth), label="truth", linewidth=2.0, color="black")
    if ch.openloop is not None:
        ax_traj.plot(
            t_s,
            line(ch.openloop),
            label="open-loop",
            linewidth=1.0,
            linestyle="--",
            color="tab:gray",
        )
    has_measurements = (
        ch.measurement_t_ms is not None
        and ch.measurement is not None
        and len(ch.measurement_t_ms) > 0
    )
    if has_measurements:
        assert ch.measurement_t_ms is not None and ch.measurement is not None
        meas_t_s = ch.measurement_t_ms / 1000.0
        ax_traj.scatter(
            meas_t_s,
            _wrap180(ch.measurement) if ch.wrap else ch.measurement,
            label="measurement",
            s=10,
            color="tab:orange",
            alpha=_MEASUREMENT_SCATTER_ALPHA,
            zorder=2,
        )
    ax_traj.plot(
        t_s,
        line(ch.estimate),
        label="EKF estimate",
        linewidth=1.5,
        color="tab:blue",
        zorder=3,
    )
    ax_traj.set_ylabel(f"{ch.name} ({ch.unit})")
    ax_traj.legend(loc="best")
    ax_traj.grid(True, alpha=0.3)

    residual = ch.estimate - ch.truth
    if ch.wrap:
        residual = _wrap180(residual)

    if ch.estimate_std is not None:
        ax_res.fill_between(
            t_s,
            -ch.estimate_std,
            ch.estimate_std,
            alpha=0.2,
            color="tab:blue",
            label="EKF +/-1 sigma",
        )
    ax_res.axhline(0, color="black", linewidth=0.8, alpha=0.7)
    if has_measurements:
        assert ch.measurement is not None and ch.measurement_t_ms is not None
        meas_t_s = ch.measurement_t_ms / 1000.0
        meas_truth = np.interp(ch.measurement_t_ms, t_ms, ch.truth)
        meas_error = ch.measurement - meas_truth
        if ch.wrap:
            meas_error = _wrap180(meas_error)
        ax_res.scatter(
            meas_t_s,
            meas_error,
            label="measurement error",
            s=10,
            color="tab:orange",
            alpha=_MEASUREMENT_SCATTER_ALPHA,
            zorder=2,
        )
    ax_res.plot(
        t_s,
        residual,
        label="EKF error",
        linewidth=1.5,
        color="tab:blue",
        zorder=3,
    )

    if ch.estimate_std is not None:
        # Bound y so the wide initial P0 sigma band doesn't squash the
        # steady-state detail. Skip the first 5% (the P0 transient).
        warmup = max(1, len(ch.estimate_std) // 20)
        steady_std = ch.estimate_std[warmup:]
        steady_std = steady_std[np.isfinite(steady_std)]
        if len(steady_std) > 0:
            steady_res = residual[warmup:]
            steady_res = steady_res[np.isfinite(steady_res)]
            bound = 3.0 * max(
                float(steady_std.max()),
                float(np.abs(steady_res).max()) if len(steady_res) > 0 else 0.0,
            )
            if bound > 0.0:
                ax_res.set_ylim(-bound, bound)

    ax_res.set_ylabel(f"{ch.name} error ({ch.unit})")
    ax_res.legend(loc="best")
    ax_res.grid(True, alpha=0.3)
