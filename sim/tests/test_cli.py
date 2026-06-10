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
