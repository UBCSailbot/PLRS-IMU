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

import math
from collections.abc import Iterator
from dataclasses import dataclass, field

import numpy as np

from .attitude import (
    conjugate,
    from_axis_angle,
    multiply,
    quaternion_to_euler_zyx,
    world_to_body,
)
from .ekf import gnss_sample_from_attitude
from .noise import GnssNoise, ImuNoise
from .truth import sample_attitude, sample_heading
from .types import (
    GRAVITY_MS2,
    GnssAttitudeMount,
    GnssNoiseModel,
    GnssSample,
    ImuNoiseModel,
    ImuSample,
    Quaternion,
    Scenario,
    Tick,
    Vec3,
)

_WORLD_Z = Vec3(x=0.0, y=0.0, z=1.0)


@dataclass(frozen=True, slots=True, kw_only=True)
class SimulatedSource:
    scenario: Scenario
    imu_noise: ImuNoiseModel
    gnss_noise: GnssNoiseModel
    duration_s: float
    seed: int
    mount: GnssAttitudeMount = field(default_factory=GnssAttitudeMount)
    # boat_to_imu rotation: the synthesized MTi orientation is the boat
    # attitude pre-rotated by this, so the filter (configured with the same
    # mount) recovers boat attitude. Identity leaves the IMU square.
    imu_mount: Quaternion = field(default_factory=Quaternion.identity)
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
            truth_heading, heading_dot = sample_heading(self.scenario.heading, t_ms)
            attitude_q, attitude_body_omega = sample_attitude(
                self.scenario.attitude, t_ms
            )

            # The MTi reports in ENU, where yaw is CCW-positive: the
            # orientation carries yaw = -heading and the world-Z yaw rate
            # is the negated heading rate. See docs/attitude.md.
            orientation = multiply(
                from_axis_angle(_WORLD_Z, math.radians(-truth_heading)), attitude_q
            )

            # Yaw is a world-frame angular velocity about world Z. The
            # gyro sees it in the body frame; rotate through the inverse
            # orientation. Under level attitude this collapses to body Z;
            # under heel, world Z splits into body Y and Z components.
            yaw_body = world_to_body(orientation, Vec3(x=0.0, y=0.0, z=-heading_dot))

            # The MTi sits in the IMU frame: report the boat attitude rotated
            # by the inverse mount. The gyro stays in boat axes, matching the
            # filter, which corrects the quaternion but not the rates.
            mti_orientation = multiply(orientation, conjugate(self.imu_mount))
            clean_imu = ImuSample(
                angular_velocity_rad_s=Vec3(
                    x=attitude_body_omega.x + yaw_body.x,
                    y=attitude_body_omega.y + yaw_body.y,
                    z=attitude_body_omega.z + yaw_body.z,
                ),
                accel_ms2=Vec3(x=0.0, y=0.0, z=GRAVITY_MS2),
                orientation=mti_orientation,
                timestamp_ms=t_ms,
            )
            corrupted_imu = imu_noise.corrupt(clean_imu, dt_s=imu_dt_s)

            gnss: GnssSample | None = None
            if t_ms >= next_gnss_ms:
                # Measure in the antenna-baseline frame, corrupt there, then
                # run through the real bridge so it undoes the offset and the
                # filter sees the same path as on hardware.
                baseline_heading = truth_heading + self.mount.baseline_offset_deg
                clean_gnss = GnssSample(
                    heading_deg=baseline_heading,
                    heading_variance_deg2=0.0,
                    timestamp_ms=t_ms,
                    valid=True,
                )
                noisy = gnss_noise.corrupt(clean_gnss)
                gnss = gnss_sample_from_attitude(
                    heading_deg=baseline_heading
                    if noisy is None
                    else noisy.heading_deg,
                    heading_variance_deg2=0.0
                    if noisy is None
                    else noisy.heading_variance_deg2,
                    valid=noisy is not None,
                    tow_ms=t_ms,
                    mount=self.mount,
                )
                next_gnss_ms += gnss_dt_ms

            truth_roll, truth_pitch, _ = quaternion_to_euler_zyx(orientation)
            yield Tick(
                timestamp_ms=t_ms,
                truth_heading_deg=truth_heading,
                truth_roll_deg=truth_roll,
                truth_pitch_deg=truth_pitch,
                imu=corrupted_imu,
                gnss=gnss,
            )
