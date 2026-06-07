"""Smoke tests for plot_trace.

Uses the Agg backend (no display required). Asserts the function builds
a figure without raising and that --save writes a real file.
"""

from __future__ import annotations

import os

os.environ.setdefault("MPLBACKEND", "Agg")

from pathlib import Path

from plrs_sim import (
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
)
from plrs_sim.plot import plot_trace
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

CFG = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)


def _trace():
    src = SimulatedSource(
        trajectory=ConstantTurn(rate_deg_s=5.0),
        imu_noise=ImuNoiseModel(gyro_white_std_rad_s=0.01),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
        duration_s=2.0,
        seed=0,
    )
    return run(src, CFG)


def test_plot_trace_runs_headless() -> None:
    plot_trace(_trace(), show=False)


def test_plot_trace_writes_save_path(tmp_path: Path) -> None:
    out = tmp_path / "heading.png"
    plot_trace(_trace(), show=False, save=out)
    assert out.exists()
    assert out.stat().st_size > 0


def test_plot_trace_accepts_title() -> None:
    plot_trace(_trace(), show=False, title="test scenario")
