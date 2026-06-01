"""Value types for the simulation harness.

Frozen, slotted, kw-only dataclasses mirror the C++ structs in fusion.h
and add the sim-only types (trajectories, noise models, traces).
Each crosses any boundary by copy; nothing is shared, nothing mutated.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuSample:
    rate_of_turn_x_rad_s: float
    rate_of_turn_y_rad_s: float
    rate_of_turn_z_rad_s: float
    accel_x_ms2: float
    accel_y_ms2: float
    accel_z_ms2: float
    timestamp_ms: int


@dataclass(frozen=True, slots=True, kw_only=True)
class GnssSample:
    heading_deg: float
    heading_variance_deg2: float
    timestamp_ms: int
    valid: bool


@dataclass(frozen=True, slots=True, kw_only=True)
class FusionOutput:
    heading_deg: float
    heading_variance_deg2: float
    timestamp_ms: int


@dataclass(frozen=True, slots=True, kw_only=True)
class EkfConfig:
    q_heading_deg2: float
    q_bias_deg2_s2: float
    p0_heading_deg2: float
    p0_bias_deg2_s2: float


@dataclass(frozen=True, slots=True, kw_only=True)
class Tick:
    """One step in a sample-source iteration.

    gnss is None on IMU ticks that do not coincide with a GNSS sample.
    truth_heading_deg is the noise-free heading from the trajectory at
    timestamp_ms; the runner records it so the plot can show ground truth.
    """

    timestamp_ms: int
    truth_heading_deg: float
    imu: ImuSample
    gnss: GnssSample | None


@dataclass(frozen=True, slots=True, kw_only=True)
class ConstantTurn:
    rate_deg_s: float
    heading0_deg: float = 0.0


@dataclass(frozen=True, slots=True, kw_only=True)
class Sinusoidal:
    amplitude_deg: float
    period_s: float
    heading0_deg: float = 0.0


@dataclass(frozen=True, slots=True, kw_only=True)
class StepTurns:
    """Piecewise-constant turn rate; each leg is (duration_s, rate_deg_s).

    Heading is the cumulative integral starting from heading0_deg, so a
    `(10.0, 0.0)` leg sails straight for 10 s and a `(2.0, 45.0)` leg
    turns 90 degrees over 2 s. Past the final leg, heading holds.
    """

    legs: tuple[tuple[float, float], ...]
    heading0_deg: float = 0.0


@dataclass(frozen=True, slots=True, kw_only=True)
class Static:
    heading_deg: float


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuNoiseModel:
    """Additive gyro_z effects. None disables that effect.

    gyro_bias_walk_std_rad_s_sqrt_s is the diffusion coefficient: the
    per-step delta is drawn from N(0, sigma * sqrt(dt)).
    """

    gyro_white_std_rad_s: float | None = None
    gyro_constant_bias_rad_s: float | None = None
    gyro_bias_walk_std_rad_s_sqrt_s: float | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class GnssNoiseModel:
    heading_std_deg: float | None = None
    dropout_prob: float | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class Trace:
    """Captured run output, one column per series, ready for plotting."""

    t_ms: np.ndarray
    truth_deg: np.ndarray
    est_deg: np.ndarray
    est_std_deg: np.ndarray
    openloop_deg: np.ndarray
    gnss_t_ms: np.ndarray
    gnss_deg: np.ndarray
