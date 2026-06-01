"""Heading-overlay plot for a Trace.

Two stacked subplots sharing an x-axis:
- Top: trajectory view. Truth, open-loop gyro integration, GNSS samples,
  and EKF estimate, all overlaid.
- Bottom: residual view. EKF estimate error against truth with the
  filter's claimed +/-1 sigma band centered at zero, plus GNSS error
  scattered. The band is the load-bearing element here -- residuals
  staying inside the band mean the filter is consistent with its claimed
  uncertainty; residuals escaping mean overconfidence.

Headless callers should set MPLBACKEND=Agg in the environment before
importing this module.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .types import Trace

_GNSS_SCATTER_ALPHA = 0.4


def plot_heading(
    trace: Trace,
    *,
    show: bool = True,
    save: Path | None = None,
    title: str | None = None,
) -> None:
    fig, (ax_traj, ax_res) = plt.subplots(
        2,
        1,
        figsize=(10, 7),
        sharex=True,
        gridspec_kw={"height_ratios": [2, 1]},
    )

    t_s = trace.t_ms / 1000.0
    gnss_t_s = trace.gnss_t_ms / 1000.0

    ax_traj.plot(t_s, trace.truth_deg, label="truth", linewidth=2.0, color="black")
    ax_traj.plot(
        t_s,
        trace.openloop_deg,
        label="open-loop gyro",
        linewidth=1.0,
        linestyle="--",
        color="tab:gray",
    )
    if len(gnss_t_s) > 0:
        ax_traj.scatter(
            gnss_t_s,
            trace.gnss_deg,
            label="GNSS",
            s=10,
            color="tab:orange",
            alpha=_GNSS_SCATTER_ALPHA,
            zorder=2,
        )
    ax_traj.plot(
        t_s,
        trace.est_deg,
        label="EKF estimate",
        linewidth=1.5,
        color="tab:blue",
        zorder=3,
    )
    ax_traj.set_ylabel("heading (deg)")
    if title is not None:
        ax_traj.set_title(title)
    ax_traj.legend(loc="best")
    ax_traj.grid(True, alpha=0.3)

    ekf_residual = trace.est_deg - trace.truth_deg

    ax_res.fill_between(
        t_s,
        -trace.est_std_deg,
        trace.est_std_deg,
        alpha=0.2,
        color="tab:blue",
        label="EKF +/-1 sigma",
    )
    ax_res.axhline(0, color="black", linewidth=0.8, alpha=0.7)
    if len(gnss_t_s) > 0:
        gnss_truth = np.interp(trace.gnss_t_ms, trace.t_ms, trace.truth_deg)
        ax_res.scatter(
            gnss_t_s,
            trace.gnss_deg - gnss_truth,
            label="GNSS error",
            s=10,
            color="tab:orange",
            alpha=_GNSS_SCATTER_ALPHA,
            zorder=2,
        )
    ax_res.plot(
        t_s,
        ekf_residual,
        label="EKF error",
        linewidth=1.5,
        color="tab:blue",
        zorder=3,
    )

    # Bound the y-axis so the wide initial P0 sigma band doesn't squash
    # the steady-state detail. Skip the first 5% of samples for the
    # bound calculation; that's the transient where p0 dominates.
    warmup = max(1, len(trace.est_std_deg) // 20)
    steady_std = trace.est_std_deg[warmup:]
    steady_std = steady_std[np.isfinite(steady_std)]
    if len(steady_std) > 0:
        steady_res = ekf_residual[warmup:]
        steady_res = steady_res[np.isfinite(steady_res)]
        bound = 3.0 * max(
            float(steady_std.max()),
            float(np.abs(steady_res).max()) if len(steady_res) > 0 else 0.0,
        )
        if bound > 0.0:
            ax_res.set_ylim(-bound, bound)

    ax_res.set_xlabel("time (s)")
    ax_res.set_ylabel("error (deg)")
    ax_res.legend(loc="best")
    ax_res.grid(True, alpha=0.3)

    fig.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
