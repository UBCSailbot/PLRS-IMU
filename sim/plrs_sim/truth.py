"""Ground-truth trajectories.

A trajectory is one of four frozen dataclass variants. `heading_at` and
`gyro_z_at` are pure functions of (trajectory, time) — no internal state
to defend. Dispatch is a `match` statement with `assert_never` so adding
a fifth variant is a type error until both functions handle it.
"""

from __future__ import annotations

import math

from .types import ConstantTurn, Sinusoidal, Static, StepTurns

Trajectory = ConstantTurn | Sinusoidal | StepTurns | Static

DEG_TO_RAD = math.pi / 180.0


def heading_at(traj: Trajectory, t_ms: int) -> float:
    t_s = t_ms / 1000.0
    match traj:
        case ConstantTurn(rate_deg_s=rate, heading0_deg=h0):
            return h0 + rate * t_s
        case Sinusoidal(amplitude_deg=a, period_s=period, heading0_deg=h0):
            return h0 + a * math.sin(2.0 * math.pi * t_s / period)
        case StepTurns(legs=legs, heading0_deg=h0):
            return _step_heading_at(legs, h0, t_s)
        case Static(heading_deg=h):
            return h


def gyro_z_at(traj: Trajectory, t_ms: int) -> float:
    t_s = t_ms / 1000.0
    match traj:
        case ConstantTurn(rate_deg_s=rate):
            return rate * DEG_TO_RAD
        case Sinusoidal(amplitude_deg=a, period_s=period):
            omega = 2.0 * math.pi / period
            return a * omega * math.cos(omega * t_s) * DEG_TO_RAD
        case StepTurns(legs=legs):
            return _step_rate_at(legs, t_s) * DEG_TO_RAD
        case Static():
            return 0.0


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
