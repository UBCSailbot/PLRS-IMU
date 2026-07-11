"""Tests for plrs_sim.live telemetry parsing (mirror of fusion_task.cpp)."""

from __future__ import annotations

import math

import pytest

from plrs_sim import Quaternion, Vec3
from plrs_sim.live import (
    DiagRecord,
    FusionRecord,
    GnssRecord,
    ImuRecord,
    MemsRecord,
    MonitorState,
    format_record,
    heading_offset_deg,
    monitor,
    pace,
    parse_line,
    replay_file,
)

# A short captured stream: bring-up, then two telemetry ticks with a GNSS fix.
_SAMPLE_LINES = [
    "# IMU: ready",
    "F,1000,10.000,1.000,2.000,0.100,0.200,0.300",
    "I,1000,1.00000,0.00000,0.00000,0.00000,0.0,0.0,0.0,0.0,0.0,9.81",
    "G,1000,90.000,1.500,1",
    "garbled?!",
    "F,2000,11.000,1.500,2.500,0.090,0.190,0.290",
    "I,2000,1.00000,0.00000,0.00000,0.00000,0.0,0.0,0.0,0.0,0.0,9.81",
]


def test_parse_fusion_line() -> None:
    # Pre-debug capture format: the trailing EKF-debug run is absent and
    # keeps its defaults.
    rec = parse_line("F,1234,12.500,-3.250,1.000,0.100,0.200,0.300")
    assert isinstance(rec, FusionRecord)
    assert rec.heading_deg == 12.5
    assert math.isnan(rec.gyro_bias_dps)
    assert math.isnan(rec.mag_offset_deg)
    assert rec.gate_rejects == 0


def test_parse_fusion_line_with_debug_tail() -> None:
    rec = parse_line(
        "F,1234,12.500,-3.250,1.000,0.100,0.200,0.300,2.2000,0.2236,-33.000,10.000,3,7"
    )
    assert isinstance(rec, FusionRecord)
    assert rec.gyro_bias_dps == pytest.approx(2.2)
    assert rec.gyro_bias_sigma_dps == pytest.approx(0.2236)
    assert rec.mag_offset_deg == pytest.approx(-33.0)
    assert rec.mag_offset_sigma_deg == pytest.approx(10.0)
    assert rec.gate_rejects == 3
    assert rec.mag_gate_rejects == 7


def test_parse_imu_line() -> None:
    rec = parse_line("I,5000,1.00000,0.00000,0.00000,0.00000,0.1,-0.2,0.3,0.0,0.0,9.81")
    assert isinstance(rec, ImuRecord)
    assert rec.timestamp_ms == 5000
    assert (rec.orientation.w, rec.orientation.x) == (1.0, 0.0)
    assert rec.angular_velocity_rad_s.y == pytest.approx(-0.2)
    assert rec.accel_ms2.z == pytest.approx(9.81)


def test_parse_gnss_line_valid_and_invalid() -> None:
    ok = parse_line("G,42,90.000,1.500,1")
    assert ok == GnssRecord(
        timestamp_ms=42, heading_deg=90.0, heading_sigma_deg=1.5, valid=True
    )
    bad = parse_line("G,42,0.000,0.000,0")
    assert isinstance(bad, GnssRecord) and not bad.valid


def test_parse_gnss_line_with_mode_error() -> None:
    rec = parse_line("G,42,160.000,2.000,0,1,4")
    assert isinstance(rec, GnssRecord)
    assert (rec.valid, rec.mode, rec.error) == (False, 1, 4)


def test_parse_mems_line_roundtrips() -> None:
    rec = parse_line(
        "M,5000,0.0100,-0.0200,9.8100,0.10000,-0.20000,0.30000,0.40000,-0.50000,0.60000"
    )
    assert isinstance(rec, MemsRecord)
    assert rec.timestamp_ms == 5000
    assert rec.accel_ms2.z == pytest.approx(9.81)
    assert rec.angular_velocity_rad_s.x == pytest.approx(0.1)
    assert rec.magnetic_field_au.y == pytest.approx(-0.5)
    assert parse_line(format_record(rec)) == rec


def test_parse_diagnostic_strips_hash() -> None:
    assert parse_line("# IMU: ready") == DiagRecord(text="IMU: ready")
    assert parse_line("#GNSS: ready") == DiagRecord(text="GNSS: ready")


def test_blank_and_unknown_lines_are_none() -> None:
    assert parse_line("") is None
    assert parse_line("   ") is None
    assert parse_line("Z,1,2,3") is None


def test_wrong_field_count_is_none() -> None:
    assert parse_line("F,1,2,3") is None
    assert parse_line("F,1,2,3,4,5,6,7,8") is None


def test_unparseable_number_is_none() -> None:
    assert parse_line("F,abc,2,3,4,5,6,7") is None
    assert parse_line("I,5,x,0,0,0,0,0,0,0,0,9.8") is None


def test_strips_trailing_newline_and_whitespace() -> None:
    rec = parse_line("  F,1,2,3,4,5,6,7\r\n")
    assert isinstance(rec, FusionRecord) and rec.timestamp_ms == 1


def test_monitor_state_accumulates_both_attitude_sources() -> None:
    state = monitor(_SAMPLE_LINES, show=False, summary_interval_ms=0)
    assert list(state.fused.t_ms) == [1000, 2000]
    assert list(state.fused.heading) == [10.0, 11.0]
    # Identity quaternion -> level attitude on the open-loop series.
    assert list(state.openloop.t_ms) == [1000, 2000]
    assert state.openloop.roll[-1] == pytest.approx(0.0)
    # The first I (t=1000) seeds heading from the first fused heading (10);
    # the GNSS fix then re-anchors it to 90, and the zero gyro adds no drift.
    assert list(state.openloop.heading) == pytest.approx([10.0, 90.0])
    assert state.last_gnss is not None and state.last_gnss.heading_deg == 90.0
    assert state.last_diag == "IMU: ready"
    assert state.latest_t_ms == 2000


def test_open_loop_heading_integrates_gyro_after_seed() -> None:
    # Seed heading at 100 deg; compass convention, so a CCW 1 rad/s yaw rate
    # for 1 s drives it -57.3 deg.
    lines = [
        "G,0,100.000,1.000,1",
        "I,0,1,0,0,0,0,0,0,0,0,9.81",
        "I,1000,1,0,0,0,0,0,1.0,0,0,9.81",
    ]
    state = monitor(lines, show=False, summary_interval_ms=0)
    expected = [100.0, 100.0 - 180.0 / 3.14159]
    assert list(state.openloop.heading) == pytest.approx(expected, abs=0.1)


def test_open_loop_heading_wraps_at_seam() -> None:
    # Seed at -170, then a CCW gyro that walks heading below -180. Each step
    # wraps to (-180, 180] like the firmware-wrapped fused track, so the
    # open-loop line crosses the seam together with it rather than running past.
    lines = ["G,0,-170.000,1.000,1", "I,0,1,0,0,0,0,0,0,0,0,9.81"]
    # 0.5 rad/s for 1 s steps -> -28.6 deg/step; by 1 s it is below -180.
    lines += [f"I,{t},1,0,0,0,0,0,0.5,0,0,9.81" for t in (1000, 2000, 3000)]
    state = monitor(lines, show=False, summary_interval_ms=0)
    headings = list(state.openloop.heading)
    assert headings[1] == pytest.approx(161.35, abs=0.1)  # first step wrapped
    assert all(-180.0 < h <= 180.0 for h in headings)


def test_snapshot_consistent_under_concurrent_appends() -> None:
    # The GUI thread snapshots the series while the drain thread appends; the two
    # arrays per series must always be equal length (no half-appended sample).
    import threading

    state = MonitorState()
    stop = threading.Event()

    def writer() -> None:
        t = 0
        while not stop.is_set():
            t += 10
            state.update(
                ImuRecord(
                    timestamp_ms=t,
                    orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
                    angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=0.1),
                    accel_ms2=Vec3(x=0.0, y=0.0, z=9.81),
                )
            )

    thread = threading.Thread(target=writer)
    thread.start()
    try:
        for _ in range(3000):
            _t_now, arrays = state.snapshot("heading")
            for t, values in arrays:
                assert t.size == values.size
    finally:
        stop.set()
        thread.join()


def test_monitor_records_stream_verbatim(tmp_path) -> None:
    path = tmp_path / "nested" / "capture.log"
    monitor(_SAMPLE_LINES, record=path, show=False, summary_interval_ms=0)
    written = path.read_text().splitlines()
    # Every raw line is captured, including the garbled one, parent dir created.
    assert written == _SAMPLE_LINES


def test_replay_round_trips_a_capture(tmp_path) -> None:
    path = tmp_path / "capture.log"
    path.write_text("\n".join(_SAMPLE_LINES) + "\n")
    state = monitor(replay_file(path), show=False, summary_interval_ms=0)
    assert list(state.fused.heading) == [10.0, 11.0]


def test_headless_prints_summary_and_diagnostics(capsys) -> None:
    monitor(_SAMPLE_LINES, show=False, summary_interval_ms=0)
    out = capsys.readouterr().out
    assert "# IMU: ready" in out
    assert "fused hdg/roll/pitch" in out


def test_summary_line_handles_empty_state() -> None:
    # No records seen yet: must not raise on the empty deques.
    assert "fused --" in MonitorState().summary_line()


def test_wire_format_is_pinned() -> None:
    # Exact wire bytes per record type: tag, field order, precision, and
    # flag encoding. A diff here is a protocol change and must be made
    # deliberately, together with the emitters in fusion_task.cpp.
    fusion = FusionRecord(
        timestamp_ms=1234,
        heading_deg=12.5,
        roll_deg=-3.25,
        pitch_deg=1.0,
        heading_sigma_deg=0.1,
        roll_sigma_deg=0.2,
        pitch_sigma_deg=0.3,
        gyro_bias_dps=2.2,
        gyro_bias_sigma_dps=0.2236,
        mag_offset_deg=-33.0,
        mag_offset_sigma_deg=10.0,
        gate_rejects=3,
        mag_gate_rejects=7,
    )
    assert format_record(fusion) == (
        "F,1234,12.500,-3.250,1.000,0.100,0.200,0.300,2.2000,0.2236,-33.000,10.000,3,7"
    )
    imu = ImuRecord(
        timestamp_ms=5000,
        orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        angular_velocity_rad_s=Vec3(x=0.1, y=-0.2, z=0.3),
        accel_ms2=Vec3(x=0.0, y=0.0, z=9.81),
    )
    assert format_record(imu) == (
        "I,5000,1.00000,0.00000,0.00000,0.00000,"
        "0.10000,-0.20000,0.30000,0.0000,0.0000,9.8100"
    )
    mems = MemsRecord(
        timestamp_ms=5000,
        accel_ms2=Vec3(x=0.01, y=-0.02, z=9.81),
        angular_velocity_rad_s=Vec3(x=0.1, y=-0.2, z=0.3),
        magnetic_field_au=Vec3(x=0.4, y=-0.5, z=0.6),
    )
    assert format_record(mems) == (
        "M,5000,0.0100,-0.0200,9.8100,0.10000,-0.20000,0.30000,0.40000,-0.50000,0.60000"
    )
    gnss = GnssRecord(
        timestamp_ms=42,
        heading_deg=90.0,
        heading_sigma_deg=1.5,
        valid=True,
        mode=2,
        error=0,
    )
    assert format_record(gnss) == "G,42,90.000,1.500,1,2,0"


def test_overflowed_sigma_round_trips_as_ovf() -> None:
    # Arduino Print renders out-of-range floats as "ovf"; the codec must
    # emit and parse the same token.
    rec = FusionRecord(
        timestamp_ms=1,
        heading_deg=0.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        heading_sigma_deg=math.inf,
        roll_sigma_deg=0.2,
        pitch_sigma_deg=0.3,
        gyro_bias_dps=0.0,
        gyro_bias_sigma_dps=0.1,
        mag_offset_deg=0.0,
        mag_offset_sigma_deg=0.1,
    )
    line = format_record(rec)
    assert ",ovf," in line
    assert parse_line(line) == rec


def test_nan_debug_tail_round_trips_as_nan() -> None:
    # A NaN'd filter state renders as "nan" (Arduino Print) and must stay
    # parseable; NaN breaks == so it is checked field-wise.
    rec = FusionRecord(
        timestamp_ms=1,
        heading_deg=0.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        heading_sigma_deg=0.1,
        roll_sigma_deg=0.2,
        pitch_sigma_deg=0.3,
    )
    line = format_record(rec)
    assert ",nan," in line
    back = parse_line(line)
    assert isinstance(back, FusionRecord)
    assert math.isnan(back.gyro_bias_dps)
    assert math.isnan(back.mag_offset_sigma_deg)


def test_gnss_partial_optional_tail_is_rejected() -> None:
    # mode/error are a format-version boundary: present together or not at
    # all. Six tokens is a corrupt line, not an old format.
    assert parse_line("G,42,90.000,1.500,1,2") is None


def test_format_parse_round_trips() -> None:
    fusion = FusionRecord(
        timestamp_ms=1000,
        heading_deg=12.5,
        roll_deg=-3.25,
        pitch_deg=1.0,
        heading_sigma_deg=0.1,
        roll_sigma_deg=0.2,
        pitch_sigma_deg=0.3,
        gyro_bias_dps=0.01,
        gyro_bias_sigma_dps=0.22,
        mag_offset_deg=-33.0,
        mag_offset_sigma_deg=10.0,
        gate_rejects=1,
    )
    imu = ImuRecord(
        timestamp_ms=2000,
        orientation=Quaternion(w=0.70711, x=0.0, y=0.70711, z=0.0),
        angular_velocity_rad_s=Vec3(x=0.1, y=-0.2, z=0.3),
        accel_ms2=Vec3(x=0.0, y=0.0, z=9.81),
    )
    gnss = GnssRecord(
        timestamp_ms=42, heading_deg=90.0, heading_sigma_deg=1.5, valid=True
    )
    assert parse_line(format_record(fusion)) == fusion
    assert parse_line(format_record(gnss)) == gnss
    back = parse_line(format_record(imu))
    assert isinstance(back, ImuRecord)
    assert back.orientation.w == pytest.approx(0.70711)
    assert back.accel_ms2.z == pytest.approx(9.81)


def test_live_draw_panels_render_headless() -> None:
    # Prove the scroll-panel drawing code runs (under Agg, no window): a bug
    # in the live panels surfaces here instead of only on hardware night.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from plrs_sim.views import _ScrollPanel

    state = monitor(_SAMPLE_LINES, show=False, summary_interval_ms=0)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    panel = _ScrollPanel(ax, "heading", window_s=20.0)
    panel.update(state)
    assert any(line.get_xdata().size for line in ax.lines)  # a series got data
    plt.close(fig)


def test_heading_offset_takes_the_short_way_around() -> None:
    assert heading_offset_deg(90.0, 0.0) == pytest.approx(90.0)
    assert heading_offset_deg(0.0, 90.0) == pytest.approx(-90.0)
    # Across the +-180 seam: 10 vs 350 is +20, not -340.
    assert heading_offset_deg(10.0, 350.0) == pytest.approx(20.0)
    # +180 and -180 are the same heading -> zero residual.
    assert heading_offset_deg(180.0, -180.0) == pytest.approx(0.0)


def test_alignment_summary_states() -> None:
    assert MonitorState().alignment_summary() == "waiting for IMU..."

    # IMU only (no GNSS yet): roll/pitch shown, offset withheld.
    imu_only = monitor(
        ["I,0,1,0,0,0,0,0,0,0,0,9.81"], show=False, summary_interval_ms=0
    )
    summary = imu_only.alignment_summary()
    assert "roll=" in summary and "heading offset --" in summary

    # With a valid fix and an identity quaternion seeded to 90, the offset is 0.
    aligned = monitor(_SAMPLE_LINES, show=False, summary_interval_ms=0)
    assert "heading offset (GNSS-IMU)=" in aligned.alignment_summary()
    assert heading_offset_deg(90.0, 90.0) == pytest.approx(0.0)


def test_align_panel_renders_headless() -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from plrs_sim.views import _draw_align_panel

    state = monitor(_SAMPLE_LINES, show=False, summary_interval_ms=0)
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    _draw_align_panel(ax, state)
    assert ax.lines  # the reference hull drew at least one segment
    plt.close(fig)


def test_pace_sleeps_only_between_timestamps() -> None:
    slept: list[float] = []
    lines = [
        "# start",
        "F,1000,0,0,0,0,0,0",
        "I,1000,1,0,0,0,0,0,0,0,0,9.81",
        "F,1100,0,0,0,0,0,0",
    ]
    out = list(pace(lines, speed=1.0, sleep=slept.append))
    assert out == lines  # every line passes through, in order
    # Only the 100 ms jump between the two F lines induces a wait (~0.1 s).
    assert len(slept) == 1
    assert slept[0] == pytest.approx(0.1, abs=0.03)
