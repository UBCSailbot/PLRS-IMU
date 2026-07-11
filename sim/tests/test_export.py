"""Tests for plrs_sim.export (sim run -> telemetry wire format)."""

from __future__ import annotations

from itertools import pairwise

from plrs_sim import (
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Sinusoidal,
    load_tuning,
)
from plrs_sim.export import export_telemetry
from plrs_sim.live import (
    DiagRecord,
    FusionRecord,
    GnssRecord,
    ImuRecord,
    monitor,
    parse_line,
)
from plrs_sim.source import SimulatedSource


def _source(duration_s: float = 3.0) -> SimulatedSource:
    return SimulatedSource(
        scenario=Scenario(heading=Sinusoidal(amplitude_deg=20.0, period_s=5.0)),
        imu_noise=ImuNoiseModel(),
        gnss_noise=GnssNoiseModel(),
        duration_s=duration_s,
        seed=1,
    )


def test_export_emits_parseable_tagged_stream() -> None:
    lines = list(export_telemetry(_source(), load_tuning()))
    records = [parse_line(line) for line in lines]
    kinds = {type(r) for r in records}
    assert {DiagRecord, FusionRecord, ImuRecord, GnssRecord} <= kinds
    assert all(r is not None for r in records)


def test_export_throttles_fusion_to_telemetry_interval() -> None:
    lines = list(export_telemetry(_source(duration_s=1.0), load_tuning()))
    fused = [r for r in map(parse_line, lines) if isinstance(r, FusionRecord)]
    # 1 s at the default 100 ms interval is ~10 fused lines, not the 100 IMU
    # ticks; consecutive timestamps step by the interval.
    assert 8 <= len(fused) <= 12
    deltas = {b.timestamp_ms - a.timestamp_ms for a, b in pairwise(fused)}
    assert deltas == {100}


def test_exported_stream_replays_through_the_monitor() -> None:
    lines = list(export_telemetry(_source(), load_tuning()))
    state = monitor(lines, show=False, summary_interval_ms=0)
    # Both attitude sources populated and time advanced past the first tick.
    assert state.fused.t_ms and state.openloop.t_ms
    assert state.latest_t_ms is not None and state.latest_t_ms > 0
