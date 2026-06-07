"""Ground-truth profile samplers.

Each profile variant has a closed-form sampler — no internal state to
defend. Dispatch is a `match` statement; adding a variant is a type
error in its sampler until handled.

sample_yaw(profile, t) -> (heading_deg, gyro_z_rad_s) returns the truth
heading and its derivative.

sample_attitude(profile, t) -> (orientation, body_omega_rad_s) returns
the world<-body quaternion and the body-frame angular velocity
contributed by attitude motion alone (not yaw).
"""

from __future__ import annotations

import math

from .attitude import from_axis_angle
from .types import (
    AttitudeProfile,
    ConstantHeel,
    ConstantTurn,
    LevelAttitude,
    Quaternion,
    Sinusoidal,
    Static,
    StepTurns,
    Vec3,
    YawProfile,
)

DEG_TO_RAD = math.pi / 180.0
_BODY_X = Vec3(x=1.0, y=0.0, z=0.0)


def sample_yaw(profile: YawProfile, t_ms: int) -> tuple[float, float]:
    """Return (heading_deg, gyro_z_rad_s) at t."""
    t_s = t_ms / 1000.0
    match profile:
        case ConstantTurn(rate_deg_s=rate, heading0_deg=h0):
            return h0 + rate * t_s, rate * DEG_TO_RAD
        case Sinusoidal(amplitude_deg=a, period_s=period, heading0_deg=h0):
            omega = 2.0 * math.pi / period
            return (
                h0 + a * math.sin(omega * t_s),
                a * omega * math.cos(omega * t_s) * DEG_TO_RAD,
            )
        case StepTurns(legs=legs, heading0_deg=h0):
            return (
                _step_heading_at(legs, h0, t_s),
                _step_rate_at(legs, t_s) * DEG_TO_RAD,
            )
        case Static(heading_deg=h):
            return h, 0.0


def sample_attitude(profile: AttitudeProfile, t_ms: int) -> tuple[Quaternion, Vec3]:
    """Return (world<-body orientation, body-frame omega from attitude motion)."""
    match profile:
        case LevelAttitude():
            return Quaternion.identity(), Vec3(x=0.0, y=0.0, z=0.0)
        case ConstantHeel(angle_deg=angle):
            return (
                from_axis_angle(_BODY_X, angle * DEG_TO_RAD),
                Vec3(x=0.0, y=0.0, z=0.0),
            )


def _step_heading_at(
    legs: tuple[tuple[float, float], ...], heading0: float, t_s: float
) -> float:
    heading = heading0
    elapsed = 0.0
    for duration, rate in legs:
        if t_s < elapsed + duration:
            return heading + rate * (t_s - elapsed)
        heading += rate * duration
        elapsed += duration
    return heading


def _step_rate_at(legs: tuple[tuple[float, float], ...], t_s: float) -> float:
    elapsed = 0.0
    for duration, rate in legs:
        if t_s < elapsed + duration:
            return rate
        elapsed += duration
    return 0.0
