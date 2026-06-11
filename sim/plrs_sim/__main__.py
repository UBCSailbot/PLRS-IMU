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
from .plot import plot_pose, plot_trace
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
    WaveMotion,
)

VIEWS = ("timeseries", "mounting", "pose")

SCENARIOS: dict[str, Scenario] = {
    "constant_turn": Scenario(yaw=ConstantTurn(rate_deg_s=5.0)),
    "sinusoidal": Scenario(yaw=Sinusoidal(amplitude_deg=30.0, period_s=20.0)),
    "step_turns": Scenario(
        yaw=StepTurns(
            legs=(
                (10.0, 0.0),
                (2.0, 45.0),
                (10.0, 0.0),
                (2.0, -45.0),
                (10.0, 0.0),
            ),
        ),
    ),
    "static": Scenario(yaw=Static(heading_deg=0.0)),
    "heeling_tack": Scenario(
        yaw=StepTurns(
            legs=(
                (5.0, 0.0),
                (3.0, 30.0),
                (15.0, 0.0),
            ),
        ),
        attitude=ConstantHeel(angle_deg=20.0),
    ),
    "wave_tack": Scenario(
        yaw=StepTurns(
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
        default=0.01,
        help="gyro white-noise std (rad/s); 0 disables",
    )
    sim.add_argument(
        "--gyro-bias",
        type=float,
        default=0.005,
        help="gyro constant bias (rad/s); 0 disables",
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

    return p


def _zero_to_none(x: float) -> float | None:
    return x if x > 0.0 else None


def _select_interactively(parser: argparse.ArgumentParser) -> argparse.Namespace | None:
    """Arrow-key prompt for scenario + view; returns parsed args, or None to quit."""
    import questionary

    scenario = questionary.select(
        "Scenario", choices=[*sorted(SCENARIOS), "quit"]
    ).ask()
    if scenario is None or scenario == "quit":  # Ctrl-C or explicit quit
        return None
    view = questionary.select("View", choices=list(VIEWS)).ask()
    if view is None:  # Ctrl-C
        return None
    return parser.parse_args(["sim", scenario, "--view", view])


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.cmd is None:
        while True:
            args = _select_interactively(parser)
            if args is None:
                break
            _run_view(args)
    else:
        _run_view(args)


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
            gyro_constant_bias_rad_s=_zero_to_none(args.gyro_bias),
            gyro_bias_walk_std_rad_s_sqrt_s=_zero_to_none(args.gyro_walk),
            mti_attitude_std_deg=_zero_to_none(args.mti_attitude_std),
        ),
        gnss_noise=GnssNoiseModel(
            heading_std_deg=_zero_to_none(args.gnss_std),
            dropout_prob=_zero_to_none(args.gnss_dropout),
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
    if args.view == "pose":
        plot_pose(trace, show=not args.no_show, save=args.save, title=title)
    else:
        plot_trace(trace, show=not args.no_show, save=args.save, title=title)


if __name__ == "__main__":
    main()
