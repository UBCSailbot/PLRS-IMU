"""Sinusoidal heading: amplitude 45 deg, period 20 s.

Exercises a changing turn rate so the EKF and open-loop both react to
non-constant gyro_z input.
"""

from __future__ import annotations

from plrs_sim import (
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Sinusoidal,
    Vec3,
)
from plrs_sim.plot import plot_trace
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

src = SimulatedSource(
    scenario=Scenario(heading=Sinusoidal(amplitude_deg=45.0, period_s=20.0)),
    imu_noise=ImuNoiseModel(
        gyro_white_std_rad_s=0.015,
        gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.005),
    ),
    gnss_noise=GnssNoiseModel(heading_std_deg=2.0),
    duration_s=60.0,
    seed=1,
)
cfg = EkfConfig(
    q_heading_deg2=0.05,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)
plot_trace(run(src, cfg), title="Sinusoidal heading, A=45 deg, T=20 s")
