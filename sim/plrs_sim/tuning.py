"""Load EKF tuning and GNSS calibration from tuning.toml."""

from __future__ import annotations

import tomllib
from pathlib import Path

from .types import EkfConfig, GnssAttitudeMount, MtiYawConfig

# tuning.toml lives at the repo root, shared with the (future) firmware build.
DEFAULT_PATH = Path(__file__).resolve().parents[2] / "tuning.toml"


def load_tuning(path: Path = DEFAULT_PATH) -> EkfConfig:
    with path.open("rb") as f:
        t = tomllib.load(f)
    proc, init, mti = t["process_noise"], t["initial_covariance"], t["mti_noise"]
    mount = t.get("imu_mount", {})
    yaw = t.get("mti_yaw")
    return EkfConfig(
        q_heading_deg2=proc["heading_deg2"],
        q_roll_deg2=proc["roll_deg2"],
        q_pitch_deg2=proc["pitch_deg2"],
        q_bias_deg2_s2=proc["gyro_bias_deg2_s2"],
        p0_heading_deg2=init["heading_deg2"],
        p0_roll_deg2=init["roll_deg2"],
        p0_pitch_deg2=init["pitch_deg2"],
        p0_bias_deg2_s2=init["gyro_bias_deg2_s2"],
        mti_roll_variance_deg2=mti["roll_deg2"],
        mti_pitch_variance_deg2=mti["pitch_deg2"],
        mti_yaw=(
            MtiYawConfig(
                variance_deg2=yaw["variance_deg2"],
                q_offset_deg2=yaw["q_offset_deg2"],
                p0_offset_deg2=yaw["p0_offset_deg2"],
            )
            if yaw is not None
            else None
        ),
        mount_roll_deg=mount.get("mount_roll_deg", 0.0),
        mount_pitch_deg=mount.get("mount_pitch_deg", 0.0),
        mount_yaw_deg=mount.get("mount_yaw_deg", 0.0),
    )


def load_mount(path: Path = DEFAULT_PATH) -> GnssAttitudeMount:
    with path.open("rb") as f:
        t = tomllib.load(f)
    gnss = t["gnss"]
    return GnssAttitudeMount(
        baseline_offset_deg=gnss["baseline_offset_deg"],
        fallback_heading_variance_deg2=gnss["fallback_heading_variance_deg2"],
    )
