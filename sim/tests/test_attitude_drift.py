"""Regression: heading drift vs boat attitude, including the Euler blind spot.

Static truth heading + a body-frame gyro turn-on bias + a sustained GNSS
outage. In the sailing envelope (roll heel, moderate trim) the 3-axis bias
state learns the bias before the outage and holds it, so heading does not
ramp. Near 90 deg of pitch the ZYX-Euler heading kinematics go singular
(sec(pitch) -> inf, NaN by ~89 deg) and heading is unreliable -- the regime
the drifty bench captures sat in (pitch to 112 deg), and a known limitation
captured here so a quaternion heading state would flip the xfail.

See examples/drift_sweep.py for the interactive version of this grid.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from plrs_sim import (
    ConstantHeel,
    ConstantTrim,
    GnssNoiseModel,
    ImuNoiseModel,
    LevelAttitude,
    MagNoiseModel,
    Scenario,
    Static,
    Vec3,
)
from plrs_sim.angles import wrap180
from plrs_sim.noise import MTI3_GYRO_WHITE_STD_RAD_S
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource
from plrs_sim.tuning import load_tuning

_DEG = math.pi / 180.0
_OUTAGE_S = 20.0
_DURATION = 90.0
# A z-axis turn-on bias at the MTi-3's ~0.2 deg/s repeatability.
_Z_BIAS = Vec3(x=0.0, y=0.0, z=0.2 * _DEG)
_INDOOR_MAG = MagNoiseModel(
    iron_deg=20.0, snap_deg=40.0, snap_interval_s=30.0, snap_tau_s=20.0
)


def _outage_drift(attitude, *, bias=_Z_BIAS, mag=None, seed=0):
    """Return (peak heading error deg, net drift rate deg/s) over the outage."""
    src = SimulatedSource(
        scenario=Scenario(heading=Static(heading_deg=0.0), attitude=attitude),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=MTI3_GYRO_WHITE_STD_RAD_S,
            gyro_constant_bias_rad_s=bias,
            mti_attitude_std_deg=0.5,
            mag=mag,
        ),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0, outage_start_s=_OUTAGE_S),
        duration_s=_DURATION,
        seed=seed,
        gnss_rate_hz=1.0,
    )
    tr = run(src, load_tuning())
    ch = tr.channels["heading"]
    win = tr.t_ms >= round(_OUTAGE_S * 1000)
    resid = wrap180(ch.estimate[win] - ch.truth[win])
    est = np.degrees(np.unwrap(np.radians(ch.estimate[win])))
    dt = float(tr.t_ms[win][-1] - tr.t_ms[win][0]) / 1000.0
    return float(np.max(np.abs(resid))), abs(est[-1] - est[0]) / dt


@pytest.mark.parametrize(
    "attitude",
    [
        LevelAttitude(),
        ConstantHeel(angle_deg=20.0),
        ConstantHeel(angle_deg=45.0),
        ConstantHeel(angle_deg=77.0),
        ConstantTrim(angle_deg=45.0),
        ConstantTrim(angle_deg=70.0),
        ConstantTrim(angle_deg=80.0),
    ],
)
def test_bias_bounded_across_sailing_envelope(attitude) -> None:
    # At any heel and any trim short of vertical the filter holds heading
    # through the outage: the bias is observed and removed, not ramped.
    peak, drift = _outage_drift(attitude)
    assert drift < 0.12, f"{attitude}: {drift:.3f} deg/s"
    assert peak < 12.0, f"{attitude}: {peak:.1f} deg"


def test_heeled_bias_with_indoor_mag_stays_bounded() -> None:
    # The heeled + indoor-mag case that motivated the drift-hardening arc.
    peak, drift = _outage_drift(ConstantHeel(angle_deg=45.0), mag=_INDOOR_MAG)
    assert drift < 0.15, f"{drift:.3f} deg/s"
    assert peak < 15.0, f"{peak:.1f} deg"


def test_yaxis_bias_at_heel_stays_bounded() -> None:
    # A body-Y bias at heel is the mechanism the 3-axis extension addressed:
    # it is observable through roll/pitch, so it is carried as a bias rather
    # than ramped into heading. It stays within the same envelope bound as the
    # z-axis case, not the multi-deg/s ramp an unmodelled Y bias would give.
    y_bias = Vec3(x=0.0, y=0.2 * _DEG, z=0.0)
    peak, drift = _outage_drift(ConstantHeel(angle_deg=77.0), bias=y_bias)
    assert drift < 0.12, f"{drift:.3f} deg/s"
    assert peak < 12.0, f"{peak:.1f} deg"


@pytest.mark.xfail(
    strict=True,
    reason="ZYX-Euler gimbal lock near 90 deg pitch; heading unreliable past "
    "~85 deg (NaN by ~89). A boat never trims here, but bench handling and "
    "knockdown can. Fix: a quaternion/rotation-vector heading state.",
)
def test_heading_survives_near_vertical_pitch() -> None:
    # Documents the boundary: at 88 deg trim the sec(pitch) term blows heading
    # up far past the sailing-envelope bounds. Flips to xpass once the state
    # parameterization no longer goes singular near vertical.
    peak, drift = _outage_drift(ConstantTrim(angle_deg=88.0))
    assert drift < 0.12
    assert peak < 12.0
