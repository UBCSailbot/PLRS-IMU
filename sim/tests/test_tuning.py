"""Tests for loading EKF tuning from tuning.toml."""

from __future__ import annotations

from plrs_sim import load_mount, load_tuning
from plrs_sim.tuning import DEFAULT_PATH


def test_default_tuning_file_exists() -> None:
    assert DEFAULT_PATH.exists()


def test_default_tuning_loads_all_fields() -> None:
    cfg = load_tuning()
    assert cfg.q_heading_deg2 == 0.01
    assert cfg.q_roll_deg2 == 0.01
    assert cfg.q_pitch_deg2 == 0.01
    assert cfg.q_bias_deg2_s2 == 0.0001
    assert cfg.p0_heading_deg2 == 1000.0
    assert cfg.mti_roll_variance_deg2 == 1.0
    assert cfg.mti_pitch_variance_deg2 == 1.0


def test_load_tuning_reads_a_custom_file(tmp_path) -> None:
    path = tmp_path / "custom.toml"
    path.write_text(
        "[process_noise]\n"
        "heading_deg2 = 0.5\n"
        "roll_deg2 = 0.1\n"
        "pitch_deg2 = 0.2\n"
        "gyro_bias_deg2_s2 = 0.001\n"
        "[initial_covariance]\n"
        "heading_deg2 = 10.0\n"
        "roll_deg2 = 20.0\n"
        "pitch_deg2 = 30.0\n"
        "gyro_bias_deg2_s2 = 2.0\n"
        "[mti_noise]\n"
        "roll_deg2 = 3.0\n"
        "pitch_deg2 = 4.0\n"
    )
    cfg = load_tuning(path)
    assert cfg.q_heading_deg2 == 0.5
    assert cfg.p0_pitch_deg2 == 30.0
    assert cfg.mti_pitch_variance_deg2 == 4.0


def test_load_mount_reads_gnss_section() -> None:
    mount = load_mount()
    assert mount.baseline_offset_deg == 0.0
    assert mount.fallback_heading_variance_deg2 == 4.0


def test_load_mount_reads_a_custom_file(tmp_path) -> None:
    path = tmp_path / "custom.toml"
    path.write_text(
        "[gnss]\nbaseline_offset_deg = 17.5\nfallback_heading_variance_deg2 = 9.0\n"
    )
    mount = load_mount(path)
    assert mount.baseline_offset_deg == 17.5
    assert mount.fallback_heading_variance_deg2 == 9.0


def test_default_tuning_has_zero_imu_mount() -> None:
    cfg = load_tuning()
    assert (cfg.mount_roll_deg, cfg.mount_pitch_deg, cfg.mount_yaw_deg) == (
        0.0,
        0.0,
        0.0,
    )


def test_load_tuning_reads_imu_mount_section(tmp_path) -> None:
    path = tmp_path / "custom.toml"
    path.write_text(
        "[process_noise]\n"
        "heading_deg2 = 0.01\nroll_deg2 = 0.01\npitch_deg2 = 0.01\n"
        "gyro_bias_deg2_s2 = 0.0001\n"
        "[initial_covariance]\n"
        "heading_deg2 = 1000.0\nroll_deg2 = 1000.0\npitch_deg2 = 1000.0\n"
        "gyro_bias_deg2_s2 = 1.0\n"
        "[mti_noise]\nroll_deg2 = 1.0\npitch_deg2 = 1.0\n"
        "[imu_mount]\n"
        "mount_roll_deg = 3.0\nmount_pitch_deg = -2.0\nmount_yaw_deg = 5.0\n"
    )
    cfg = load_tuning(path)
    assert (cfg.mount_roll_deg, cfg.mount_pitch_deg, cfg.mount_yaw_deg) == (
        3.0,
        -2.0,
        5.0,
    )
