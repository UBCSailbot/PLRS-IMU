"""Heading-overlay plot for a Trace.

Single axes, time on the x-axis, heading on the y-axis. Truth, open-loop
gyro integration, GNSS samples, and EKF estimate with a +/-1 sigma band, all
overlaid. Headless callers should set MPLBACKEND=Agg in the environment
before importing this module; the module itself doesn't change the
backend so it stays well-behaved for interactive use.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

from .types import Trace


def plot_heading(
    trace: Trace,
    *,
    show: bool = True,
    save: Path | None = None,
    title: str | None = None,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    t_s = trace.t_ms / 1000.0

    ax.plot(
        t_s,
        trace.truth_deg,
        label="truth",
        linewidth=2.0,
        color="black",
    )
    ax.plot(
        t_s,
        trace.openloop_deg,
        label="open-loop gyro",
        linewidth=1.0,
        linestyle="--",
        color="tab:gray",
    )
    if len(trace.gnss_t_ms) > 0:
        ax.scatter(
            trace.gnss_t_ms / 1000.0,
            trace.gnss_deg,
            label="GNSS",
            s=10,
            color="tab:orange",
            alpha=0.6,
            zorder=3,
        )
    ax.plot(
        t_s,
        trace.est_deg,
        label="EKF estimate",
        linewidth=1.5,
        color="tab:blue",
    )
    ax.fill_between(
        t_s,
        trace.est_deg - trace.est_std_deg,
        trace.est_deg + trace.est_std_deg,
        alpha=0.2,
        color="tab:blue",
        label="EKF +/-1 sigma",
    )

    ax.set_xlabel("time (s)")
    ax.set_ylabel("heading (deg)")
    if title is not None:
        ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    if save is not None:
        fig.savefig(save, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
