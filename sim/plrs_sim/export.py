"""Render a simulated run as RP2040 telemetry lines.

Drives the same EKF the firmware runs over a Tick source and emits the exact
F/I/G wire format fusion_task.cpp produces. The point is verification without
hardware: a synthetic capture can be replayed through the live monitor to
prove the visualization works before a board is on the bench.

The emission cadence mirrors the firmware -- F and I paired at the telemetry
interval, G on each accepted GNSS sample -- so the replayed stream looks like
the real one.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator

from .ekf import TinyEkfFilter
from .live import FusionRecord, GnssRecord, ImuRecord, format_record
from .types import EkfConfig, Tick

_DEFAULT_TELEMETRY_INTERVAL_MS = 100


def _sigma(variance_deg2: float) -> float:
    return math.sqrt(max(variance_deg2, 0.0))


def export_telemetry(
    source: Iterable[Tick],
    cfg: EkfConfig,
    *,
    telemetry_interval_ms: int = _DEFAULT_TELEMETRY_INTERVAL_MS,
) -> Iterator[str]:
    """Yield telemetry lines for a run, mirroring the firmware's emit cadence."""
    ekf = TinyEkfFilter(cfg)
    next_emit_ms: int | None = None

    yield "# IMU: ready"
    yield "# GNSS: ready"

    for tick in source:
        ekf.predict(tick.imu)
        if tick.gnss is not None:
            ekf.update(tick.gnss)
            yield format_record(
                GnssRecord(
                    timestamp_ms=tick.gnss.timestamp_ms,
                    heading_deg=tick.gnss.heading_deg,
                    heading_sigma_deg=_sigma(tick.gnss.heading_variance_deg2),
                    valid=tick.gnss.valid,
                )
            )

        if next_emit_ms is None:
            next_emit_ms = tick.timestamp_ms
        if tick.timestamp_ms < next_emit_ms:
            continue
        next_emit_ms += telemetry_interval_ms

        out = ekf.output()
        dbg = ekf.debug()
        yield format_record(
            FusionRecord(
                timestamp_ms=out.timestamp_ms,
                heading_deg=out.heading_deg,
                roll_deg=out.roll_deg,
                pitch_deg=out.pitch_deg,
                heading_sigma_deg=_sigma(out.heading_variance_deg2),
                roll_sigma_deg=_sigma(out.roll_variance_deg2),
                pitch_sigma_deg=_sigma(out.pitch_variance_deg2),
                gyro_bias_dps=dbg.gyro_bias_dps,
                gyro_bias_sigma_dps=_sigma(dbg.gyro_bias_variance_deg2_s2),
                gyro_bias_x_dps=dbg.gyro_bias_x_dps,
                gyro_bias_x_sigma_dps=_sigma(dbg.gyro_bias_x_variance_deg2_s2),
                gyro_bias_y_dps=dbg.gyro_bias_y_dps,
                gyro_bias_y_sigma_dps=_sigma(dbg.gyro_bias_y_variance_deg2_s2),
                mag_offset_deg=dbg.mag_offset_deg,
                mag_offset_sigma_deg=_sigma(dbg.mag_offset_variance_deg2),
                gate_rejects=dbg.gate_rejects,
                mag_gate_rejects=dbg.mag_gate_rejects,
            )
        )
        yield format_record(
            ImuRecord(
                timestamp_ms=tick.imu.timestamp_ms,
                orientation=tick.imu.orientation,
                angular_velocity_rad_s=tick.imu.angular_velocity_rad_s,
                accel_ms2=tick.imu.accel_ms2,
            )
        )
