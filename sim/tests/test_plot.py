"""Smoke tests for plot_trace.

Uses the Agg backend (no display required). Asserts the function builds
a figure without raising and that --save writes a real file.
"""

from __future__ import annotations

import os

os.environ.setdefault("MPLBACKEND", "Agg")

from pathlib import Path

import numpy as np
import pytest

from plrs_sim import (
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
)
from plrs_sim.plot import _seam_broken, _wrap180, plot_animate, plot_pose, plot_trace
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
        scenario=Scenario(yaw=ConstantTurn(rate_deg_s=5.0)),
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


def test_plot_pose_runs_headless() -> None:
    plot_pose(_trace(), frames=3, show=False)


def test_plot_animate_runs_headless() -> None:
    plot_animate(_trace(), show=False)


def test_plot_pose_writes_save_path(tmp_path: Path) -> None:
    out = tmp_path / "pose.png"
    plot_pose(_trace(), frames=3, show=False, save=out)
    assert out.exists()
    assert out.stat().st_size > 0


def test_wrap180_maps_into_range() -> None:
    got = _wrap180(np.array([0.0, 179.0, -179.0, 190.0, -190.0, 358.0, -360.0]))
    assert got == pytest.approx([0.0, 179.0, -179.0, -170.0, 170.0, -2.0, 0.0])


def test_wrap180_handles_a_difference_across_the_seam() -> None:
    # 179 and -179 are 2 deg apart, not 358.
    assert _wrap180(np.array([-179.0 - 179.0])) == pytest.approx([2.0])


def test_seam_broken_inserts_nan_at_crossing() -> None:
    out = _seam_broken(np.array([170.0, 179.0, 190.0, 200.0]))  # crosses +180
    assert not np.isnan(out[1])
    assert np.isnan(out[2])  # the sample just past the seam breaks the line
    assert out[3] == pytest.approx(-160.0)


def test_seam_broken_leaves_a_clean_series_untouched() -> None:
    out = _seam_broken(np.array([-10.0, 0.0, 10.0, 20.0]))
    assert out == pytest.approx([-10.0, 0.0, 10.0, 20.0])
