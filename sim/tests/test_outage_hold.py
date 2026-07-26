"""Regression for the outage offset-pin (mti_yaw.q_offset_outage_deg2).

With the shipped loose offset the mag cannot hold heading through a GNSS outage:
the offset floats, heading coasts on the gyro, and its confidence decays past
the rudder's steer-on bound within seconds. Pinning the offset once GNSS has
been gone past the grace lets a clean mag hold heading and keep it confident
enough to steer on. Safe only with a genuinely clean mag; see docs/tuning.md.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from plrs_sim import (
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Static,
    Vec3,
)
from plrs_sim.angles import wrap180
from plrs_sim.noise import MTI3_GYRO_WHITE_STD_RAD_S
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource
from plrs_sim.tuning import load_tuning

_OUTAGE_S = 30.0
_END_S = 180.0
_STEER_ON_SIGMA = 5.0  # sqrt(rudder_task HEADING_VALID_MAX_VARIANCE_DEG2 = 25)
_DEG2RAD = math.pi / 180.0


def _source(seed: int) -> SimulatedSource:
    # Clean mag (mag=None) and a 0.2 deg/s turn-on Z bias, so there is real
    # heading drift for the mag to hold against once GNSS drops.
    return SimulatedSource(
        scenario=Scenario(heading=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=MTI3_GYRO_WHITE_STD_RAD_S,
            gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.2 * _DEG2RAD),
            mti_attitude_std_deg=0.5,
        ),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0, outage_start_s=_OUTAGE_S),
        duration_s=_END_S,
        seed=seed,
        gnss_rate_hz=1.0,
    )


def _outage_peak_and_final_sigma(trace) -> tuple[float, float]:
    ch = trace.channels["heading"]
    win = trace.t_ms >= _OUTAGE_S * 1000.0
    err = np.abs(wrap180(ch.estimate[win] - ch.truth[win]))
    return float(err.max()), float(ch.estimate_std[win][-1])


def _pinned_tuning():
    base = load_tuning()
    return replace(base, mti_yaw=replace(base.mti_yaw, q_offset_outage_deg2=1.0e-4))


def test_pin_holds_heading_and_confidence_through_outage() -> None:
    peak, final_sigma = _outage_peak_and_final_sigma(run(_source(7), _pinned_tuning()))
    # Clean mag + pinned offset: heading held tight for the whole 2.5 min outage
    # and still confident enough to steer on.
    assert peak < 2.0
    assert final_sigma < _STEER_ON_SIGMA


def test_default_offset_loses_confidence_through_outage() -> None:
    # Contrast: the shipped default (no outage pin) coasts and its reported
    # confidence decays past the steer-on bound, so the rudder correctly stops
    # trusting heading. This is the fail-safe the pin trades away for hold.
    _, final_sigma = _outage_peak_and_final_sigma(run(_source(7), load_tuning()))
    assert final_sigma > _STEER_ON_SIGMA
