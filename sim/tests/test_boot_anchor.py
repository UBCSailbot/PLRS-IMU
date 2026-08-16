"""Regression for the boot-time mag heading anchor (mti_yaw.offset_seed_deg).

Without GNSS the EKF heading has no absolute authority: the mag yaw only
observes heading + offset, so a free offset lets the good mag heading leak away
and the reported heading coasts on gyro bias (bench: ~75 deg drift over ~2 min
on a clean, still BNO085). Seeding the offset and pinning it anchors heading to
the mag from the first sample, before any GNSS fix, so a clean mag holds heading
from boot. The shipped tuning enables this; safe only with a characterized mag.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from plrs_sim import EkfConfig, GnssNoiseModel, ImuNoiseModel, Scenario, Static, Vec3
from plrs_sim.angles import wrap180
from plrs_sim.attitude import euler_to_quaternion
from plrs_sim.noise import MTI3_GYRO_WHITE_STD_RAD_S
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource
from plrs_sim.tuning import load_tuning

_END_S = 120.0
_DEG2RAD = math.pi / 180.0


def _no_gnss_source(seed: int, cfg: EkfConfig) -> SimulatedSource:
    # No GNSS ever (outage from t=0), a clean mag, and a 0.2 deg/s turn-on Z
    # bias so there is real heading drift for the anchor to hold against.
    #
    # The synthesized IMU is tilted by the same mount the filter corrects for,
    # as __main__ does. Leaving it identity while the filter carries the boat's
    # measured mount would make the filter de-rotate a rotation the IMU never
    # had, and with no GNSS to pull heading back nothing would absorb the error.
    return SimulatedSource(
        imu_mount=euler_to_quaternion(
            cfg.mount_roll_deg, cfg.mount_pitch_deg, cfg.mount_yaw_deg
        ),
        scenario=Scenario(heading=Static(heading_deg=0.0)),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=MTI3_GYRO_WHITE_STD_RAD_S,
            gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.2 * _DEG2RAD),
            mti_attitude_std_deg=0.5,
        ),
        gnss_noise=GnssNoiseModel(heading_std_deg=1.0, outage_start_s=0.0),
        duration_s=_END_S,
        seed=seed,
        gnss_rate_hz=1.0,
    )


def _peak_heading_error(trace) -> float:
    ch = trace.channels["heading"]
    return float(np.abs(wrap180(ch.estimate - ch.truth)).max())


def test_seed_anchors_heading_from_boot_without_gnss() -> None:
    # Seeded + pinned: heading is anchored to the clean mag from the first
    # sample and holds near truth for the whole no-GNSS run. The synthetic mag
    # models no declination (magnetic == true) and a square IMU, so the
    # sim-correct seed is 0; the shipped seed is the real boat's declination
    # plus frame constant, which this scenario does not carry.
    base = load_tuning()
    seeded = replace(base, mti_yaw=replace(base.mti_yaw, offset_seed_deg=0.0))
    assert _peak_heading_error(run(_no_gnss_source(7, seeded), seeded)) < 3.0


def test_unseeded_heading_drifts_without_gnss() -> None:
    # Contrast: drop the seed and the pin, and with no GNSS the offset floats
    # while heading coasts on the gyro bias, drifting well past the anchored
    # bound. This is the behaviour the seed fixes.
    base = load_tuning()
    unseeded = replace(
        base,
        mti_yaw=replace(base.mti_yaw, offset_seed_deg=None, q_offset_outage_deg2=None),
    )
    assert _peak_heading_error(run(_no_gnss_source(7, unseeded), unseeded)) > 10.0
