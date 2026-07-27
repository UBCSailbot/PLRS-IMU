"""Regression for post-movement heading drift (see docs/tuning.md, mti_yaw).

The failure mode: with GNSS out, the MTi's mag-referenced yaw is the only
heading authority, so when it snaps and re-converges after a disturbance the
fused heading follows at deg/s scale. The truth here is a static heading and
a truthful gyro; any fused heading motion in the quiet tail is the filter
following the corrupted mag, not the boat.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from plrs_sim import (
    GnssNoiseModel,
    ImuNoiseModel,
    MagNoiseModel,
    Scenario,
    Static,
)
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource
from plrs_sim.tuning import load_tuning

# Aggressive but realistic indoor mag: a snap roughly every 30 s, up to
# 40 deg, re-converging over 20 s, on top of a 20 deg iron lobe.
_MAG = MagNoiseModel(
    iron_deg=20.0, snap_deg=40.0, snap_interval_s=30.0, snap_tau_s=20.0
)

_CUT_MS = 30_000
_END_S = 150.0
_TAIL_MS = 60_000


def _source(seed: int) -> SimulatedSource:
    return SimulatedSource(
        scenario=Scenario(heading=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(mag=_MAG),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
        duration_s=_END_S,
        seed=seed,
        gnss_rate_hz=1.0,
    )


def _tail_heading_range(source, cut_ms: int | None, cfg=None) -> float:
    ticks = (
        replace(t, gnss=None) if cut_ms is not None and t.timestamp_ms > cut_ms else t
        for t in source
    )
    trace = run(ticks, cfg if cfg is not None else load_tuning())
    heading = trace.channels["heading"]
    tail = trace.t_ms >= _TAIL_MS
    est = np.degrees(np.unwrap(np.radians(heading.estimate[tail])))
    return float(est.max() - est.min())


def test_clean_mag_stays_bounded() -> None:
    # Control: with a truthful mag the filter holds a static heading through
    # the same run, anchored or not. The anchored bound is set by the 1 deg
    # GNSS noise this scenario feeds (the real receiver reports ~0.3 deg);
    # unanchored, heading rides the clean mag and stays tighter.
    clean = replace(_source(seed=7), imu_noise=ImuNoiseModel())
    assert _tail_heading_range(clean, cut_ms=None) < 6.0
    assert _tail_heading_range(clean, cut_ms=_CUT_MS) < 2.0


def test_mag_snaps_with_gnss_stay_bounded() -> None:
    # Anchored regime: 1 Hz fixes must keep the mag from steering heading.
    assert _tail_heading_range(_source(seed=7), cut_ms=None) < 5.0


def test_mag_snaps_during_outage_do_not_steer_heading() -> None:
    # The fail-safe (unpinned) opt-out: after the cut the truth is still static
    # and the gyro truthful, and the fused heading must not walk with the mag's
    # snap re-convergence. The MTi yaw gate blocks the snap steps and the loose
    # offset absorbs the rest, so heading stays with the gyro. The shipped
    # default now pins the offset (trusts a clean mag to hold heading through an
    # outage), which trades this away; the pin belongs to a characterized mag.
    unpinned = replace(
        load_tuning(),
        mti_yaw=replace(load_tuning().mti_yaw, q_offset_outage_deg2=None),
    )
    assert _tail_heading_range(_source(seed=7), cut_ms=_CUT_MS, cfg=unpinned) < 5.0
