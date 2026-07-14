"""Tacking pattern: sail straight, hard turn, sail straight, turn back.

Legs are (duration_s, turn_rate_deg_s). The 2 s legs at +/-45 deg/s
rotate the boat 90 deg each, alternating port and starboard tacks.
"""

from __future__ import annotations

from plrs_sim import (
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    StepTurns,
    Vec3,
)
from plrs_sim.plot import plot_trace
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

src = SimulatedSource(
    scenario=Scenario(
        heading=StepTurns(
            legs=(
                (15.0, 0.0),
                (2.0, 45.0),
                (15.0, 0.0),
                (2.0, -45.0),
                (15.0, 0.0),
                (2.0, 45.0),
                (15.0, 0.0),
            ),
        ),
    ),
    imu_noise=ImuNoiseModel(
        gyro_white_std_rad_s=0.02,
        gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.008),
    ),
    gnss_noise=GnssNoiseModel(heading_std_deg=1.5),
    duration_s=80.0,
    seed=2,
)
cfg = EkfConfig(
    q_heading_deg2=0.05,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)
plot_trace(run(src, cfg), title="Tacking: straight + alternating 90 deg turns")
