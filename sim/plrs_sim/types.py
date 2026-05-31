"""Value types for the simulation harness.

Frozen, slotted, kw-only dataclasses mirror the C++ structs in fusion.h.
Each crosses the FFI boundary by copy; nothing is shared, nothing mutated.
"""

from __future__ import annotations

from dataclasses import dataclass


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
