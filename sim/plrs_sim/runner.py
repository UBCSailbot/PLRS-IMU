"""Drive a sample source through the EKF and collect a Trace."""

from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np

from .ekf import TinyEkfFilter
from .types import EkfConfig, Tick, Trace

_RAD_TO_DEG = 180.0 / math.pi

# Anything above this variance is uninitialized FLT_MAX; render as NaN
# so matplotlib's fill_between skips it instead of blowing up the y-axis.
_UNINITIALIZED_VAR_THRESHOLD = 1e10


def run(source: Iterable[Tick], cfg: EkfConfig) -> Trace:
    ekf = TinyEkfFilter(cfg)

    t_ms: list[int] = []
    truth_deg: list[float] = []
    est_deg: list[float] = []
    est_std_deg: list[float] = []
    openloop_deg: list[float] = []
    gnss_t_ms: list[int] = []
    gnss_deg: list[float] = []

    openloop: float | None = None
    prev_t_ms: int | None = None

    for tick in source:
        ekf.predict(tick.imu)
        if tick.gnss is not None:
            ekf.update(tick.gnss)
            gnss_t_ms.append(tick.gnss.timestamp_ms)
            gnss_deg.append(tick.gnss.heading_deg)
            if openloop is None:
                openloop = tick.gnss.heading_deg

        if openloop is not None and prev_t_ms is not None:
            dt_s = (tick.timestamp_ms - prev_t_ms) / 1000.0
            openloop += tick.imu.angular_velocity_rad_s.z * _RAD_TO_DEG * dt_s

        out = ekf.output()
        t_ms.append(tick.timestamp_ms)
        truth_deg.append(tick.truth_heading_deg)
        est_deg.append(out.heading_deg)
        var = out.heading_variance_deg2
        est_std_deg.append(
            math.sqrt(max(var, 0.0)) if var < _UNINITIALIZED_VAR_THRESHOLD else math.nan
        )
        openloop_deg.append(openloop if openloop is not None else math.nan)

        prev_t_ms = tick.timestamp_ms

    return Trace(
        t_ms=np.array(t_ms, dtype=np.int64),
        truth_deg=np.array(truth_deg, dtype=np.float64),
        est_deg=np.array(est_deg, dtype=np.float64),
        est_std_deg=np.array(est_std_deg, dtype=np.float64),
        openloop_deg=np.array(openloop_deg, dtype=np.float64),
        gnss_t_ms=np.array(gnss_t_ms, dtype=np.int64),
        gnss_deg=np.array(gnss_deg, dtype=np.float64),
    )
