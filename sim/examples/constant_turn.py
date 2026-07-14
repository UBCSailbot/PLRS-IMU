"""Constant 10 deg/s turn with realistic noise.

Open-loop drifts visibly due to gyro_bias; EKF tracks the truth line.
"""

from __future__ import annotations

from plrs_sim import (
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Vec3,
)
from plrs_sim.plot import plot_trace
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

src = SimulatedSource(
    scenario=Scenario(heading=ConstantTurn(rate_deg_s=10.0)),
    imu_noise=ImuNoiseModel(
        gyro_white_std_rad_s=0.01,
        gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.01),
        gyro_bias_walk_std_rad_s_sqrt_s=0.0005,
    ),
    gnss_noise=GnssNoiseModel(heading_std_deg=1.5),
    duration_s=30.0,
    seed=0,
)
cfg = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)
plot_trace(run(src, cfg), title="Constant turn at 10 deg/s")
