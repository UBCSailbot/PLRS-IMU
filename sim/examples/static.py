"""Boat sitting still at heading 30 deg.

GNSS-only correction with bias estimation — the open-loop integrator
drifts at the bias rate while the EKF stays pinned by GNSS.
"""

from __future__ import annotations

from plrs_sim import EkfConfig, GnssNoiseModel, ImuNoiseModel, Static
from plrs_sim.plot import plot_heading
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource

src = SimulatedSource(
    trajectory=Static(heading_deg=30.0),
    imu_noise=ImuNoiseModel(
        gyro_white_std_rad_s=0.01,
        gyro_constant_bias_rad_s=0.015,
        gyro_bias_walk_std_rad_s_sqrt_s=0.001,
    ),
    gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
    duration_s=60.0,
    seed=3,
)
cfg = EkfConfig(
    q_heading_deg2=0.01,
    q_bias_deg2_s2=0.0001,
    p0_heading_deg2=1000.0,
    p0_bias_deg2_s2=1.0,
)
plot_heading(run(src, cfg), title="Static heading 30 deg, drifting gyro")
