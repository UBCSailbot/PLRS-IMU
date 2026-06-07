"""Sample sources that drive the runner.

A source is anything that yields `Tick`s; the runner just iterates. The
named seam exists so a future `ReplaySource` reading a recorded log
drops into the runner unchanged.

`SimulatedSource` walks an IMU-rate timeline, scheduling GNSS samples on
their own cadence, and applies the noise corruptors to every emitted
sample. RNG is built fresh from `seed` on each iteration so iterating
twice produces identical output.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np

from .attitude import world_to_body
from .noise import GnssNoise, ImuNoise
from .truth import sample_attitude, sample_yaw
from .types import (
    GRAVITY_MS2,
    GnssNoiseModel,
    GnssSample,
    ImuNoiseModel,
    ImuSample,
    Scenario,
    Tick,
    Vec3,
)


@dataclass(frozen=True, slots=True, kw_only=True)
class SimulatedSource:
    scenario: Scenario
    imu_noise: ImuNoiseModel
    gnss_noise: GnssNoiseModel
    duration_s: float
    seed: int
    imu_rate_hz: float = 100.0
    gnss_rate_hz: float = 5.0

    def __iter__(self) -> Iterator[Tick]:
        rng = np.random.default_rng(self.seed)
        imu_noise = ImuNoise(self.imu_noise, rng)
        gnss_noise = GnssNoise(self.gnss_noise, rng)

        imu_dt_ms = round(1000.0 / self.imu_rate_hz)
        gnss_dt_ms = round(1000.0 / self.gnss_rate_hz)
        end_ms = round(self.duration_s * 1000.0)
        imu_dt_s = imu_dt_ms / 1000.0

        next_gnss_ms = 0
        for t_ms in range(0, end_ms + 1, imu_dt_ms):
            truth_h, yaw_omega = sample_yaw(self.scenario.yaw, t_ms)
            orientation, attitude_body_omega = sample_attitude(
                self.scenario.attitude, t_ms
            )

            # Yaw is a world-frame angular velocity about world Z. The
            # gyro sees it in the body frame; rotate through the inverse
            # orientation. Under level attitude this collapses to body Z;
            # under heel, world Z splits into body Y and Z components.
            yaw_body = world_to_body(orientation, Vec3(x=0.0, y=0.0, z=yaw_omega))

            clean_imu = ImuSample(
                angular_velocity_rad_s=Vec3(
                    x=attitude_body_omega.x + yaw_body.x,
                    y=attitude_body_omega.y + yaw_body.y,
                    z=attitude_body_omega.z + yaw_body.z,
                ),
                accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
                orientation=orientation,
                timestamp_ms=t_ms,
            )
            corrupted_imu = imu_noise.corrupt(clean_imu, dt_s=imu_dt_s)

            gnss: GnssSample | None = None
            if t_ms >= next_gnss_ms:
                clean_gnss = GnssSample(
                    heading_deg=truth_h,
                    heading_variance_deg2=0.0,
                    timestamp_ms=t_ms,
                    valid=True,
                )
                gnss = gnss_noise.corrupt(clean_gnss)
                next_gnss_ms += gnss_dt_ms

            yield Tick(
                timestamp_ms=t_ms,
                truth_heading_deg=truth_h,
                imu=corrupted_imu,
                gnss=gnss,
            )
