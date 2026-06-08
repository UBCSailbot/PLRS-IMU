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

from .attitude import from_axis_angle, multiply
from .types import (
    GnssNoiseModel,
    GnssSample,
    ImuNoiseModel,
    ImuSample,
    Quaternion,
    Vec3,
)


class ImuNoise:
    def __init__(self, model: ImuNoiseModel, rng: np.random.Generator) -> None:
        self._model = model
        self._rng = rng
        self._bias = model.gyro_constant_bias_rad_s or 0.0

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
            orientation=self._perturb(clean.orientation),
        )

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
