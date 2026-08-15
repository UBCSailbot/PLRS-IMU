"""Tack through waves: short leg, 30 deg turn, long leg, with roll and pitch.

Legs are (duration_s, turn_rate_deg_s), same as step_turns. On top of the
yaw profile, the boat rolls and pitches sinusoidally as in a seaway, so the
EKF has to track heading while the hull is also rocking.
"""

from __future__ import annotations

from plrs_sim import (
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    StepTurns,
    Vec3,
    WaveMotion,
)
from plrs_sim.plot import plot_trace
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

src = SimulatedSource(
    scenario=Scenario(
        heading=StepTurns(
            legs=(
                (5.0, 0.0),
                (3.0, 30.0),
                (15.0, 0.0),
            ),
        ),
        attitude=WaveMotion(
            roll_amplitude_deg=15.0,
            roll_period_s=4.0,
            pitch_amplitude_deg=4.0,
            pitch_period_s=3.0,
        ),
    ),
    imu_noise=ImuNoiseModel(
        gyro_white_std_rad_s=0.02,
        gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.008),
    ),
    gnss_noise=GnssNoiseModel(heading_std_deg=1.5),
    duration_s=40.0,
    seed=2,
)
cfg = EkfConfig(
    q_heading_deg2=0.05,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)
plot_trace(run(src, cfg), title="Wave tack: 30 deg turn under wave-driven roll/pitch")
