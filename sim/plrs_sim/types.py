"""Value types for the simulation harness.

Frozen, slotted, kw-only dataclasses mirror the C++ structs in fusion.h
and add the sim-only types (trajectories, noise models, traces).
Each crosses any boundary by copy; nothing is shared, nothing mutated.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRAVITY_MS2 = 9.81


@dataclass(frozen=True, slots=True, kw_only=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True, kw_only=True)
class Quaternion:
    """Mirror of plrs::Quaternion in lib/geometry, {w, x, y, z} order."""

    w: float
    x: float
    y: float
    z: float

    @staticmethod
    def identity() -> "Quaternion":
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuSample:
    angular_velocity_rad_s: Vec3
    accel_ms2: Vec3
    orientation: Quaternion = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
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
class LevelAttitude:
    """The boat sits upright with no roll or pitch motion.

    The placeholder attitude profile used when a scenario does not
    exercise heel or trim. sample_attitude returns identity for every t.
    """


@dataclass(frozen=True, slots=True, kw_only=True)
class ConstantHeel:
    """The boat sits at a fixed heel angle for the whole run.

    Positive angle is starboard heel (right-hand rotation about body X).
    No pitch, no time variation; the orientation quaternion is constant
    and the body-frame angular velocity from attitude motion is zero.
    """

    angle_deg: float


YawProfile = ConstantTurn | Sinusoidal | StepTurns | Static
AttitudeProfile = LevelAttitude | ConstantHeel


@dataclass(frozen=True, slots=True, kw_only=True)
class Scenario:
    """A composition of independent truth profiles, one per quantity.

    Each axis (yaw, attitude, eventually position/velocity) is sampled
    by its own pure function in truth.py. SimulatedSource composes the
    samplers into one ImuSample per tick.
    """

    yaw: YawProfile
    attitude: AttitudeProfile = LevelAttitude()


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
class Channel:
    """One scalar quantity tracked through a run, ready for plotting.

    Optional fields are None when the channel does not have that series:
    a freshly-added quantity may not yet have an open-loop integrator or
    a measurement source, and that should not force the plot to invent
    fake data.

    measurement_t_ms / measurement_deg travel together; the cadence is
    independent of t_ms because GNSS does not share the IMU rate.
    """

    name: str
    unit: str
    truth: np.ndarray
    estimate: np.ndarray
    estimate_std: np.ndarray | None = None
    openloop: np.ndarray | None = None
    measurement_t_ms: np.ndarray | None = None
    measurement: np.ndarray | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class Trace:
    """Captured run output: a shared time axis and a channel per quantity."""

    t_ms: np.ndarray
    channels: dict[str, Channel]
