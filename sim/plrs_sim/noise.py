"""Sensor noise corruption.

`ImuNoiseModel` / `GnssNoiseModel` are frozen *configs*. `ImuNoise` /
`GnssNoise` are the stateful executors — each owns a config plus the
running state (random-walked gyro bias, the RNG) and exposes a single
`.corrupt(...)` method. The RNG is passed in by the caller so the source
controls reproducibility.

A `None` field on either model disables that effect; `0.0` is a valid
configured value that means "modeled, magnitude zero".
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from .attitude import from_axis_angle, multiply, quaternion_to_euler_zyx
from .types import (
    GnssNoiseModel,
    GnssSample,
    ImuNoiseModel,
    ImuSample,
    Quaternion,
    Vec3,
)

_WORLD_Z = Vec3(x=0.0, y=0.0, z=1.0)


class ImuNoise:
    def __init__(self, model: ImuNoiseModel, rng: np.random.Generator) -> None:
        self._model = model
        self._rng = rng
        self._bias = model.gyro_constant_bias_rad_s or 0.0
        self._mag_snap_err_deg = 0.0
        self._mag_iron_phase = rng.uniform(0.0, 2.0 * math.pi)

    def corrupt(self, clean: ImuSample, dt_s: float) -> ImuSample:
        walk_std = self._model.gyro_bias_walk_std_rad_s_sqrt_s
        if walk_std is not None and dt_s > 0.0:
            self._bias += self._rng.normal(0.0, walk_std * math.sqrt(dt_s))

        white_std = self._model.gyro_white_std_rad_s
        noise = self._rng.normal(0.0, white_std) if white_std is not None else 0.0

        gyro = clean.angular_velocity_rad_s
        return replace(
            clean,
            angular_velocity_rad_s=Vec3(
                x=gyro.x, y=gyro.y, z=gyro.z + self._bias + noise
            ),
            orientation=self._perturb(self._mag_disturb(clean.orientation, dt_s)),
        )

    def _mag_disturb(self, orientation: Quaternion, dt_s: float) -> Quaternion:
        mag = self._model.mag
        if mag is None:
            return orientation

        if mag.snap_deg > 0.0 and dt_s > 0.0:
            self._mag_snap_err_deg *= math.exp(-dt_s / mag.snap_tau_s)
            if self._rng.random() < dt_s / mag.snap_interval_s:
                self._mag_snap_err_deg += self._rng.uniform(
                    -mag.snap_deg, mag.snap_deg
                )

        _, _, yaw_deg = quaternion_to_euler_zyx(orientation)
        iron = mag.iron_deg * math.sin(math.radians(yaw_deg) + self._mag_iron_phase)
        err_rad = math.radians(iron + self._mag_snap_err_deg)
        if err_rad == 0.0:
            return orientation
        # A yaw-only error rotates about world Z, leaving roll/pitch intact.
        return multiply(from_axis_angle(_WORLD_Z, err_rad), orientation)

    def _perturb(self, orientation: Quaternion) -> Quaternion:
        att_std = self._model.mti_attitude_std_deg
        if att_std is None or att_std <= 0.0:
            return orientation
        std_rad = math.radians(att_std)
        delta = multiply(
            multiply(
                from_axis_angle(Vec3(x=1.0, y=0.0, z=0.0), self._draw(std_rad)),
                from_axis_angle(Vec3(x=0.0, y=1.0, z=0.0), self._draw(std_rad)),
            ),
            from_axis_angle(Vec3(x=0.0, y=0.0, z=1.0), self._draw(std_rad)),
        )
        return multiply(orientation, delta)

    def _draw(self, std_rad: float) -> float:
        return float(self._rng.normal(0.0, std_rad))

    @property
    def bias_rad_s(self) -> float:
        """Current accumulated gyro_z bias (constant + random walk so far)."""
        return self._bias


class GnssNoise:
    def __init__(self, model: GnssNoiseModel, rng: np.random.Generator) -> None:
        self._model = model
        self._rng = rng

    def corrupt(self, clean: GnssSample) -> GnssSample | None:
        dropout = self._model.dropout_prob
        if dropout is not None and self._rng.random() < dropout:
            return None

        std = self._model.heading_std_deg
        if std is None:
            return clean

        noise = self._rng.normal(0.0, std) if std > 0.0 else 0.0
        return replace(
            clean,
            heading_deg=clean.heading_deg + noise,
            heading_variance_deg2=std * std,
        )
