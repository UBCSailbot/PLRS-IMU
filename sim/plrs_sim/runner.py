"""Drive a sample source through the EKF and collect a Trace."""

from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np

from .ekf import TinyEkfFilter
from .types import Channel, EkfConfig, Tick, Trace

_RAD_TO_DEG = 180.0 / math.pi

# Anything above this variance is uninitialized FLT_MAX; render as NaN
# so matplotlib's fill_between skips it instead of blowing up the y-axis.
_UNINITIALIZED_VAR_THRESHOLD = 1e10


def _std_or_nan(variance_deg2: float) -> float:
    if variance_deg2 >= _UNINITIALIZED_VAR_THRESHOLD:
        return math.nan
    return math.sqrt(max(variance_deg2, 0.0))


def run(source: Iterable[Tick], cfg: EkfConfig) -> Trace:
    ekf = TinyEkfFilter(cfg)

    t_ms: list[int] = []
    truth_heading_deg: list[float] = []
    est_heading_deg: list[float] = []
    est_heading_std_deg: list[float] = []
    openloop_heading_deg: list[float] = []
    gnss_t_ms: list[int] = []
    gnss_heading_deg: list[float] = []
    truth_roll_deg: list[float] = []
    est_roll_deg: list[float] = []
    est_roll_std_deg: list[float] = []
    truth_pitch_deg: list[float] = []
    est_pitch_deg: list[float] = []
    est_pitch_std_deg: list[float] = []

    openloop: float | None = None
    prev_t_ms: int | None = None

    for tick in source:
        ekf.predict(tick.imu)
        if tick.gnss is not None:
            # update() skips invalid samples; only plot the ones it accepts.
            ekf.update(tick.gnss)
            if tick.gnss.valid:
                gnss_t_ms.append(tick.gnss.timestamp_ms)
                gnss_heading_deg.append(tick.gnss.heading_deg)
                if openloop is None:
                    openloop = tick.gnss.heading_deg

        if openloop is not None and prev_t_ms is not None:
            dt_s = (tick.timestamp_ms - prev_t_ms) / 1000.0
            openloop += tick.imu.angular_velocity_rad_s.z * _RAD_TO_DEG * dt_s

        out = ekf.output()
        t_ms.append(tick.timestamp_ms)
        truth_heading_deg.append(tick.truth_heading_deg)
        est_heading_deg.append(out.heading_deg)
        est_heading_std_deg.append(_std_or_nan(out.heading_variance_deg2))
        openloop_heading_deg.append(openloop if openloop is not None else math.nan)
        truth_roll_deg.append(tick.truth_roll_deg)
        est_roll_deg.append(out.roll_deg)
        est_roll_std_deg.append(_std_or_nan(out.roll_variance_deg2))
        truth_pitch_deg.append(tick.truth_pitch_deg)
        est_pitch_deg.append(out.pitch_deg)
        est_pitch_std_deg.append(_std_or_nan(out.pitch_variance_deg2))

        prev_t_ms = tick.timestamp_ms

    heading = Channel(
        name="heading",
        unit="deg",
        truth=np.array(truth_heading_deg, dtype=np.float64),
        estimate=np.array(est_heading_deg, dtype=np.float64),
        estimate_std=np.array(est_heading_std_deg, dtype=np.float64),
        openloop=np.array(openloop_heading_deg, dtype=np.float64),
        measurement_t_ms=np.array(gnss_t_ms, dtype=np.int64),
        measurement=np.array(gnss_heading_deg, dtype=np.float64),
        wrap=True,
    )
    roll = Channel(
        name="roll",
        unit="deg",
        truth=np.array(truth_roll_deg, dtype=np.float64),
        estimate=np.array(est_roll_deg, dtype=np.float64),
        estimate_std=np.array(est_roll_std_deg, dtype=np.float64),
    )
    pitch = Channel(
        name="pitch",
        unit="deg",
        truth=np.array(truth_pitch_deg, dtype=np.float64),
        estimate=np.array(est_pitch_deg, dtype=np.float64),
        estimate_std=np.array(est_pitch_std_deg, dtype=np.float64),
    )
    return Trace(
        t_ms=np.array(t_ms, dtype=np.int64),
        channels={c.name: c for c in (heading, roll, pitch)},
    )
