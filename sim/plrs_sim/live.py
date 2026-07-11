"""On-target telemetry monitor: parse, record, and visualize the RP2040 stream.

This module wires the pieces together and is the public entry point. The parts
live alongside it:

- `telemetry`: the wire records and their parse/format codec.
- `monitor`: `MonitorState` plus the line sources (serial, replay, pacing) and
  the headless summary path.
- `views`: the interactive matplotlib/Qt views.

The names those modules own are re-exported here for back-compat, so
`from plrs_sim.live import ...` keeps working.
"""

from __future__ import annotations

import contextlib
from pathlib import Path
from typing import TextIO

from .monitor import (
    MonitorState,
    Series,
    _run_headless,
    _tee,
    heading_offset_deg,
    pace,
    replay_file,
    serial_lines,
)
from .telemetry import (
    DiagRecord,
    FusionRecord,
    GnssRecord,
    ImuRecord,
    MemsRecord,
    Record,
    format_record,
    parse_line,
)
from .views import _run_align, _run_live

__all__ = [
    "DiagRecord",
    "FusionRecord",
    "GnssRecord",
    "ImuRecord",
    "MemsRecord",
    "MonitorState",
    "Record",
    "Series",
    "format_record",
    "heading_offset_deg",
    "monitor",
    "pace",
    "parse_line",
    "replay_file",
    "serial_lines",
]


def monitor(
    lines,
    *,
    record: Path | None = None,
    show: bool = True,
    align: bool = False,
    summary_interval_ms: int = 1000,
) -> MonitorState:
    """Drive the telemetry stream: record losslessly, then view or summarize.

    `lines` is any iterable of raw telemetry lines -- a live serial reader, a
    replay file, or a test list -- so every entry point shares this code path.
    `align` swaps the drift view for the sensor-alignment view. Returns the
    accumulated state for inspection.
    """
    state = MonitorState()
    sink_cm: contextlib.AbstractContextManager[TextIO | None]
    if record is not None:
        record.parent.mkdir(parents=True, exist_ok=True)
        sink_cm = record.open("w")
    else:
        sink_cm = contextlib.nullcontext(None)

    with sink_cm as sink:
        teed = _tee(lines, sink)
        if show:
            (_run_align if align else _run_live)(teed, state)
        else:
            _run_headless(teed, state, summary_interval_ms)
    return state
