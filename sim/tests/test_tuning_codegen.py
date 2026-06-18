"""Guards scripts/gen_tuning.py against drift from the sim.

The firmware compiles include/tuning.h, generated from tuning.toml by
gen_tuning.py; the sim reads the same file via plrs_sim. These tests pin the
emitted numbers to what the sim loads, and pin gen_tuning's euler_to_quaternion
port to plrs_sim's, so "the value you tune is the value that ships" cannot
silently break.
"""

from __future__ import annotations

import importlib.util
import re
import tomllib
from pathlib import Path

import pytest

from plrs_sim import load_mount, load_tuning
from plrs_sim.attitude import euler_to_quaternion as sim_euler_to_quaternion
from plrs_sim.tuning import DEFAULT_PATH

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_gen_tuning():
    spec = importlib.util.spec_from_file_location(
        "gen_tuning", _REPO_ROOT / "scripts" / "gen_tuning.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen_tuning = _load_gen_tuning()

# A designated scalar initializer: `.name = <number>f`. The mount line's value
# is a function call, not a number, so it is skipped.
_SCALAR = re.compile(r"\.(\w+)\s*=\s*(-?\d[\d.eE+-]*)f")
# The four float literals inside from_unit_unchecked({...}).
_QUAT = re.compile(r"from_unit_unchecked\(\s*\{([^}]*)\}")

_CONFIG_FIELDS = (
    "q_heading_deg2",
    "q_roll_deg2",
    "q_pitch_deg2",
    "q_bias_deg2_s2",
    "p0_heading_deg2",
    "p0_roll_deg2",
    "p0_pitch_deg2",
    "p0_bias_deg2_s2",
    "mti_roll_variance_deg2",
    "mti_pitch_variance_deg2",
)


def _default_tuning() -> dict:
    with DEFAULT_PATH.open("rb") as f:
        return tomllib.load(f)


def _quat(text: str) -> tuple[float, ...]:
    inside = _QUAT.search(text).group(1)
    return tuple(float(tok.strip().rstrip("f")) for tok in inside.split(","))


def test_generated_scalars_match_sim() -> None:
    fields = {
        name: float(value)
        for name, value in _SCALAR.findall(gen_tuning.render(_default_tuning()))
    }
    cfg = load_tuning()
    mount = load_mount()
    for name in _CONFIG_FIELDS:
        assert fields[name] == pytest.approx(getattr(cfg, name))
    assert fields["baseline_offset_deg"] == pytest.approx(mount.baseline_offset_deg)
    assert fields["fallback_heading_variance_deg2"] == pytest.approx(
        mount.fallback_heading_variance_deg2
    )


@pytest.mark.parametrize(
    ("roll", "pitch", "yaw"),
    [(0.0, 0.0, 0.0), (3.0, -2.0, 5.0), (90.0, 0.0, 0.0), (10.0, 20.0, 30.0)],
)
def test_euler_to_quaternion_matches_sim(roll: float, pitch: float, yaw: float) -> None:
    gen = gen_tuning.euler_to_quaternion(roll, pitch, yaw)
    sim = sim_euler_to_quaternion(roll, pitch, yaw)
    assert gen == pytest.approx((sim.w, sim.x, sim.y, sim.z))


def test_generated_mount_quaternion_matches_sim() -> None:
    tuning = _default_tuning()
    tuning["imu_mount"] = {
        "mount_roll_deg": 3.0,
        "mount_pitch_deg": -2.0,
        "mount_yaw_deg": 5.0,
    }
    quat = _quat(gen_tuning.render(tuning))
    sim = sim_euler_to_quaternion(3.0, -2.0, 5.0)
    assert quat == pytest.approx((sim.w, sim.x, sim.y, sim.z))
