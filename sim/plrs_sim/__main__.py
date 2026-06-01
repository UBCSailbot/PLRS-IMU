"""CLI entry: `python -m plrs_sim sim <scenario> [flags...]`.

Each scenario has a baked-in trajectory shape. Per-scenario tuning of the
trajectory itself (rate, period, leg pattern) lives in the example
scripts under sim/examples/; the CLI tunes noise and EKF parameters on
top of a canned scenario.

A flag value of 0.0 for any noise std/bias disables that effect entirely.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .plot import plot_heading
from .runner import run
from .source import SimulatedSource
from .truth import Trajectory
from .types import (
    ConstantTurn,
    EkfConfig,
    GnssNoiseModel,
    ImuNoiseModel,
    Sinusoidal,
    Static,
    StepTurns,
)

SCENARIOS: dict[str, Trajectory] = {
    "constant_turn": ConstantTurn(rate_deg_s=5.0),
    "sinusoidal": Sinusoidal(amplitude_deg=30.0, period_s=20.0),
    "step_turns": StepTurns(
        legs=(
            (10.0, 0.0),
            (2.0, 45.0),
            (10.0, 0.0),
            (2.0, -45.0),
            (10.0, 0.0),
        ),
    ),
    "static": Static(heading_deg=0.0),
}


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m plrs_sim")
    sub = p.add_subparsers(dest="cmd", required=True)

    sim = sub.add_parser("sim", help="run a simulated scenario and plot the result")
    sim.add_argument("scenario", choices=sorted(SCENARIOS.keys()))
    sim.add_argument("--duration", type=float, default=30.0, metavar="SECONDS")
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

    sim.add_argument("--q-heading", type=float, default=0.01)
    sim.add_argument("--q-bias", type=float, default=0.0001)
    sim.add_argument("--p0-heading", type=float, default=1000.0)
    sim.add_argument("--p0-bias", type=float, default=1.0)

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


def main(argv: list[str] | None = None) -> None:
    args = _build_parser().parse_args(argv)

    src = SimulatedSource(
        trajectory=SCENARIOS[args.scenario],
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=_zero_to_none(args.gyro_white),
            gyro_constant_bias_rad_s=_zero_to_none(args.gyro_bias),
            gyro_bias_walk_std_rad_s_sqrt_s=_zero_to_none(args.gyro_walk),
        ),
        gnss_noise=GnssNoiseModel(
            heading_std_deg=_zero_to_none(args.gnss_std),
            dropout_prob=_zero_to_none(args.gnss_dropout),
        ),
        duration_s=args.duration,
        seed=args.seed,
        imu_rate_hz=args.imu_rate_hz,
        gnss_rate_hz=args.gnss_rate_hz,
    )
    cfg = EkfConfig(
        q_heading_deg2=args.q_heading,
        q_bias_deg2_s2=args.q_bias,
        p0_heading_deg2=args.p0_heading,
        p0_bias_deg2_s2=args.p0_bias,
    )
    plot_heading(
        run(src, cfg),
        show=not args.no_show,
        save=args.save,
        title=f"{args.scenario} (seed={args.seed})",
    )


if __name__ == "__main__":
    main()
