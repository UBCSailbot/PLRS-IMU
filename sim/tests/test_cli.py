"""CLI smoke tests: argparse dispatch + end-to-end run on each scenario."""

from __future__ import annotations

import os

os.environ.setdefault("MPLBACKEND", "Agg")

from pathlib import Path

import pytest

from plrs_sim.__main__ import SCENARIOS, main


@pytest.mark.parametrize("scenario", sorted(SCENARIOS.keys()))
def test_cli_runs_each_scenario(tmp_path: Path, scenario: str) -> None:
    out = tmp_path / f"{scenario}.png"
    main(
        [
            "sim",
            scenario,
            "--duration",
            "1.0",
            "--no-show",
            "--save",
            str(out),
        ]
    )
    assert out.exists()
    assert out.stat().st_size > 0


def test_cli_zero_flag_disables_noise(tmp_path: Path) -> None:
    out = tmp_path / "clean.png"
    main(
        [
            "sim",
            "static",
            "--duration",
            "1.0",
            "--gyro-white",
            "0",
            "--gyro-bias",
            "0",
            "--gyro-walk",
            "0",
            "--gnss-std",
            "0",
            "--no-show",
            "--save",
            str(out),
        ]
    )
    assert out.exists()


def test_cli_missing_scenario_errors() -> None:
    with pytest.raises(SystemExit):
        main(["sim"])


def test_cli_analyze_prints_iron_summary(tmp_path: Path, capsys) -> None:
    import math

    from plrs_sim.attitude import from_axis_angle
    from plrs_sim.telemetry import GnssRecord, ImuRecord, format_record
    from plrs_sim.types import Vec3

    z = Vec3(x=0.0, y=0.0, z=1.0)
    lines = []
    for i, heading in enumerate(range(0, 360, 5)):
        yaw = -(heading + 5.0 * math.cos(2 * math.radians(heading)))  # soft iron
        lines.append(
            format_record(
                ImuRecord(
                    timestamp_ms=i * 100,
                    orientation=from_axis_angle(z, math.radians(yaw)),
                    angular_velocity_rad_s=Vec3(x=0.0, y=0.0, z=0.0),
                    accel_ms2=Vec3(x=0.0, y=0.0, z=9.81),
                )
            )
        )
        lines.append(
            format_record(
                GnssRecord(
                    timestamp_ms=i * 100,
                    heading_deg=float(heading),
                    heading_sigma_deg=0.3,
                    valid=True,
                )
            )
        )
    capture = tmp_path / "swing.log"
    capture.write_text("\n".join(lines) + "\n")

    main(["analyze", str(capture)])
    out = capsys.readouterr().out
    assert "soft iron (2/rev)" in out
    assert "12/12 sectors" in out


@pytest.mark.parametrize("view", ["mounting", "pose"])
def test_cli_alternate_views(tmp_path: Path, view: str) -> None:
    out = tmp_path / f"{view}.png"
    main(
        [
            "sim",
            "wave_tack",
            "--duration",
            "2.0",
            "--view",
            view,
            "--no-show",
            "--save",
            str(out),
        ]
    )
    assert out.exists()
    assert out.stat().st_size > 0
