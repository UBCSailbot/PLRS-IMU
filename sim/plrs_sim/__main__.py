"""CLI entry: `python -m plrs_sim sim <scenario> [flags...]`.

Each scenario has a baked-in trajectory shape. Per-scenario tuning of the
trajectory itself (rate, period, leg pattern) lives in the example
scripts under sim/examples/; the CLI tunes noise and EKF parameters on
top of a canned scenario.

A flag value of 0.0 for any noise std/bias disables that effect entirely.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from .attitude import euler_to_quaternion
from .boat3d import plot_mounting
from .noise import MTI3_GYRO_WHITE_STD_RAD_S
from .plot import plot_animate, plot_pose, plot_trace
from .runner import run
from .source import SimulatedSource
from .tuning import load_mount, load_tuning
from .types import (
    ConstantHeel,
    ConstantTurn,
    GnssNoiseModel,
    ImuNoiseModel,
    Scenario,
    Sinusoidal,
    Static,
    StepTurns,
    Vec3,
    WaveMotion,
)

VIEWS = ("timeseries", "mounting", "simulate", "pose")

# Human-readable labels for the interactive picker, in display order.
# "pose" (filmstrip) is CLI-only; it does not appear here. "Monitor" is the
# live hardware view, not a sim view, and is dispatched separately.
_VIEW_LABELS = {
    "Config (tuning.toml)": "mounting",
    "Timeseries": "timeseries",
    "Simulate": "simulate",
    "Monitor (hardware)": "monitor",
}

SCENARIOS: dict[str, Scenario] = {
    "constant_turn": Scenario(heading=ConstantTurn(rate_deg_s=5.0)),
    "sinusoidal": Scenario(heading=Sinusoidal(amplitude_deg=30.0, period_s=20.0)),
    "step_turns": Scenario(
        heading=StepTurns(
            legs=(
                (10.0, 0.0),
                (2.0, 45.0),
                (10.0, 0.0),
                (2.0, -45.0),
                (10.0, 0.0),
            ),
        ),
    ),
    "static": Scenario(heading=Static(heading_deg=0.0)),
    "heeling_tack": Scenario(
        heading=StepTurns(
            legs=(
                (5.0, 0.0),
                (3.0, 30.0),
                (15.0, 0.0),
            ),
        ),
        attitude=ConstantHeel(angle_deg=20.0),
    ),
    "wave_tack": Scenario(
        heading=StepTurns(
            legs=(
                (5.0, 0.0),
                (3.0, 30.0),
                (15.0, 0.0),
            ),
        ),
        attitude=WaveMotion(
            roll_amplitude_deg=15.0,
            roll_period_s=4.0,
            pitch_amplitude_deg=4.0,
            pitch_period_s=3.0,
        ),
    ),
}


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m plrs_sim")
    # Optional: a bare invocation drops into the interactive selector.
    sub = p.add_subparsers(dest="cmd", required=False)

    sim = sub.add_parser("sim", help="run a simulated scenario and plot the result")
    sim.add_argument("scenario", choices=sorted(SCENARIOS.keys()))
    sim.add_argument(
        "--view",
        choices=VIEWS,
        default="timeseries",
        help="timeseries plot, mounting geometry, or truth-vs-estimate pose",
    )
    sim.add_argument("--duration", type=float, default=10.0, metavar="SECONDS")
    sim.add_argument("--seed", type=int, default=0)
    sim.add_argument("--imu-rate-hz", type=float, default=100.0)
    sim.add_argument("--gnss-rate-hz", type=float, default=5.0)

    sim.add_argument(
        "--gyro-white",
        type=float,
        default=MTI3_GYRO_WHITE_STD_RAD_S,
        help="gyro white-noise std (rad/s); 0 disables (default: MTi-3 datasheet)",
    )
    sim.add_argument(
        "--gyro-bias",
        type=float,
        default=0.005,
        help="gyro constant bias on the vertical (z) axis (rad/s); 0 disables",
    )
    sim.add_argument(
        "--gyro-bias-x",
        type=float,
        default=0.0,
        help="gyro constant bias on body X (rad/s); projects into heading at heel",
    )
    sim.add_argument(
        "--gyro-bias-y",
        type=float,
        default=0.0,
        help="gyro constant bias on body Y (rad/s); projects into heading at heel",
    )
    sim.add_argument(
        "--gyro-walk",
        type=float,
        default=0.001,
        help="gyro bias random-walk std (rad/s/sqrt(s)); 0 disables",
    )
    sim.add_argument(
        "--mti-attitude-std",
        type=float,
        default=1.0,
        help="MTi roll/pitch noise std (deg); 0 disables",
    )
    sim.add_argument(
        "--gnss-std",
        type=float,
        default=1.0,
        help="GNSS heading std (deg); 0 disables",
    )
    sim.add_argument(
        "--gnss-dropout",
        type=float,
        default=0.0,
        help="GNSS dropout probability [0,1]",
    )
    sim.add_argument(
        "--gnss-outage-s",
        type=float,
        default=None,
        metavar="SECONDS",
        help="start a sustained GNSS outage at this time; heading then rides on "
        "the gyro bias and mag alone",
    )
    sim.add_argument(
        "--gnss-outage-end-s",
        type=float,
        default=None,
        metavar="SECONDS",
        help="end the GNSS outage at this time (fixes resume); default never recovers",
    )

    # These override the matching value from tuning.toml when given.
    sim.add_argument("--q-heading", type=float, default=None)
    sim.add_argument("--q-bias", type=float, default=None)
    sim.add_argument("--p0-heading", type=float, default=None)
    sim.add_argument("--p0-bias", type=float, default=None)
    sim.add_argument(
        "--baseline-offset",
        type=float,
        default=None,
        metavar="DEG",
        help="GNSS antenna baseline offset from boat-forward; overrides tuning.toml",
    )
    sim.add_argument("--mount-roll", type=float, default=None, metavar="DEG")
    sim.add_argument("--mount-pitch", type=float, default=None, metavar="DEG")
    sim.add_argument("--mount-yaw", type=float, default=None, metavar="DEG")

    sim.add_argument(
        "--save",
        type=Path,
        default=None,
        metavar="PATH",
        help="save plot to file in addition to (or instead of) showing it",
    )
    sim.add_argument("--no-show", action="store_true", help="skip interactive window")

    mon = sub.add_parser(
        "monitor", help="visualize live RP2040 telemetry, or replay a capture"
    )
    src = mon.add_mutually_exclusive_group()
    src.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="serial port of the RP2040 (default; live source)",
    )
    src.add_argument(
        "--replay", type=Path, default=None, metavar="FILE", help="replay a capture"
    )
    src.add_argument(
        "--synthetic",
        choices=sorted(SCENARIOS.keys()),
        default=None,
        help="replay a simulated scenario, no hardware needed",
    )
    mon.add_argument("--baud", type=int, default=115200)
    mon.add_argument(
        "--record",
        type=Path,
        default=None,
        metavar="PATH",
        help="capture file to write (default captures/<timestamp>.log for live)",
    )
    mon.add_argument(
        "--speed", type=float, default=1.0, help="replay speed multiplier (replay only)"
    )
    mon.add_argument(
        "--duration", type=float, default=30.0, metavar="SECONDS", help="synthetic only"
    )
    mon.add_argument("--no-show", action="store_true", help="record only, no window")
    mon.add_argument(
        "--align",
        action="store_true",
        help="sensor-alignment view: live IMU axes vs GNSS heading on a level hull",
    )

    return p


def _zero_to_none(x: float) -> float | None:
    return x if x > 0.0 else None


def _bias_vec(x: float, y: float, z: float) -> Vec3 | None:
    """Assemble the body-frame gyro bias, or None when every axis is zero."""
    return Vec3(x=x, y=y, z=z) if (x or y or z) else None


def _select_interactively(parser: argparse.ArgumentParser) -> argparse.Namespace | None:
    """Prompt for view then (if needed) scenario; returns parsed args or None."""
    import questionary

    view_label = questionary.select("View", choices=[*_VIEW_LABELS, "quit"]).ask()
    if view_label is None or view_label == "quit":
        return None
    view = _VIEW_LABELS[view_label]

    if view == "mounting":
        # Config geometry is pure tuning.toml; _run_view never reads the scenario.
        return parser.parse_args(["sim", "static", "--view", "mounting"])

    if view == "monitor":
        return _select_monitor(parser)

    scenario = questionary.select("Scenario", choices=sorted(SCENARIOS)).ask()
    if scenario is None:
        return None
    duration = ["--duration", "50"] if view == "simulate" else []
    return parser.parse_args(["sim", scenario, "--view", view, *duration])


def _select_monitor(parser: argparse.ArgumentParser) -> argparse.Namespace | None:
    """Prompt for a telemetry source: live serial, replay, or synthetic."""
    import questionary

    source = questionary.select(
        "Telemetry source",
        choices=["serial (live)", "replay a capture", "synthetic (no hardware)"],
    ).ask()
    if source is None:
        return None

    if source.startswith("serial"):
        args = ["monitor"]
    elif source.startswith("replay"):
        captures = sorted(Path("captures").glob("*.log"))
        if not captures:
            print("No captures/*.log to replay yet.")
            return None
        choice = questionary.select("Capture", choices=[str(p) for p in captures]).ask()
        if choice is None:
            return None
        args = ["monitor", "--replay", choice]
    else:
        scenario = questionary.select("Scenario", choices=sorted(SCENARIOS)).ask()
        if scenario is None:
            return None
        args = ["monitor", "--synthetic", scenario]

    if questionary.confirm("Sensor-alignment view?", default=False).ask():
        args.append("--align")
    return parser.parse_args(args)


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.cmd is None:
        while True:
            args = _select_interactively(parser)
            if args is None:
                break
            _dispatch(args)
    else:
        _dispatch(args)


def _dispatch(args: argparse.Namespace) -> None:
    if args.cmd == "monitor":
        _cmd_monitor(args)
    else:
        _run_view(args)


def _cmd_monitor(args: argparse.Namespace) -> None:
    from datetime import datetime

    from .live import monitor, pace, replay_file, serial_lines

    if args.replay is not None:
        lines = pace(replay_file(args.replay), speed=args.speed)
        record = args.record
    elif args.synthetic is not None:
        from .export import export_telemetry

        # Realistic gyro noise/bias so the open-loop track visibly drifts off
        # the GNSS-corrected fused estimate -- the point of the comparison.
        source = SimulatedSource(
            scenario=SCENARIOS[args.synthetic],
            imu_noise=ImuNoiseModel(
                gyro_white_std_rad_s=MTI3_GYRO_WHITE_STD_RAD_S,
                gyro_constant_bias_rad_s=Vec3(x=0.0, y=0.0, z=0.005),
                gyro_bias_walk_std_rad_s_sqrt_s=0.001,
                mti_attitude_std_deg=1.0,
            ),
            gnss_noise=GnssNoiseModel(heading_std_deg=1.0),
            duration_s=args.duration,
            seed=0,
        )
        lines = pace(export_telemetry(source, load_tuning()), speed=args.speed)
        record = args.record
    else:
        lines = serial_lines(args.port, args.baud)
        stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        record = args.record or Path("captures") / f"{stamp}.log"

    monitor(lines, record=record, show=not args.no_show, align=args.align)


def _run_view(args: argparse.Namespace) -> None:
    mount = load_mount()
    if args.baseline_offset is not None:
        mount = replace(mount, baseline_offset_deg=args.baseline_offset)

    overrides = {
        "q_heading_deg2": args.q_heading,
        "q_bias_deg2_s2": args.q_bias,
        "p0_heading_deg2": args.p0_heading,
        "p0_bias_deg2_s2": args.p0_bias,
        "mount_roll_deg": args.mount_roll,
        "mount_pitch_deg": args.mount_pitch,
        "mount_yaw_deg": args.mount_yaw,
    }
    cfg = replace(
        load_tuning(), **{k: v for k, v in overrides.items() if v is not None}
    )

    # Mounting is pure config geometry -- no run needed.
    if args.view == "mounting":
        plot_mounting(cfg, mount, show=not args.no_show, save=args.save)
        return

    # Tilt the synthesized IMU by the same mount the filter corrects for.
    imu_mount = euler_to_quaternion(
        cfg.mount_roll_deg, cfg.mount_pitch_deg, cfg.mount_yaw_deg
    )

    src = SimulatedSource(
        scenario=SCENARIOS[args.scenario],
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=_zero_to_none(args.gyro_white),
            gyro_constant_bias_rad_s=_bias_vec(
                args.gyro_bias_x, args.gyro_bias_y, args.gyro_bias
            ),
            gyro_bias_walk_std_rad_s_sqrt_s=_zero_to_none(args.gyro_walk),
            mti_attitude_std_deg=_zero_to_none(args.mti_attitude_std),
        ),
        gnss_noise=GnssNoiseModel(
            heading_std_deg=_zero_to_none(args.gnss_std),
            dropout_prob=_zero_to_none(args.gnss_dropout),
            outage_start_s=args.gnss_outage_s,
            outage_end_s=args.gnss_outage_end_s,
        ),
        duration_s=args.duration,
        seed=args.seed,
        mount=mount,
        imu_mount=imu_mount,
        imu_rate_hz=args.imu_rate_hz,
        gnss_rate_hz=args.gnss_rate_hz,
    )
    trace = run(src, cfg)
    title = f"{args.scenario} (seed={args.seed})"
    if args.view == "simulate":
        plot_animate(trace, show=not args.no_show, save=args.save, title=title)
    elif args.view == "pose":
        plot_pose(trace, show=not args.no_show, save=args.save, title=title)
    else:
        plot_trace(trace, show=not args.no_show, save=args.save, title=title)


if __name__ == "__main__":
    main()
