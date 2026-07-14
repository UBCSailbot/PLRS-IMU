"""Drift sweep: hunt the field's deg/s heading ramps in the sim.

Runs a grid of bench/sailing conditions (boat attitude, gyro turn-on bias,
indoor mag, a sustained GNSS outage) through the real C++ EKF and ranks them by
how far fused heading walks once GNSS drops. The attitude axis spans roll heel
(the sailing regime) and pitch trim toward vertical (the bench regime): near
90 deg pitch the ZYX-Euler heading kinematics go singular and sec(pitch)
amplifies every error, which is where the drifty field captures actually sat.
In the sailing envelope the drift stays bounded; a hot row there is a lead.

    uv run python examples/drift_sweep.py
    uv run python examples/drift_sweep.py --seeds 4 --duration 200 --top 12

Truth heading is static, so any fused heading motion in the outage window is
error, not turning. Metrics per case (worst over seeds):

    peak_err  max |fused - truth| heading error over the outage (deg)
    drift     net unwrapped heading walk / outage duration (deg/s)
    final     |fused - truth| at the end of the run (deg)
"""

from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from plrs_sim import (
    ConstantHeel,
    ConstantTrim,
    GnssNoiseModel,
    ImuNoiseModel,
    LevelAttitude,
    MagNoiseModel,
    Scenario,
    Static,
    Vec3,
)
from plrs_sim.angles import wrap180
from plrs_sim.noise import MTI3_GYRO_WHITE_STD_RAD_S
from plrs_sim.runner import run
from plrs_sim.source import SimulatedSource
from plrs_sim.tuning import load_tuning
from plrs_sim.types import AttitudeProfile

_DEG_TO_RAD = math.pi / 180.0

# Aggressive-but-realistic indoor mag: an orientation-dependent iron lobe plus
# a snap every ~30 s that re-settles over 20 s (see docs/tuning.md, mti_yaw).
_INDOOR_MAG = MagNoiseModel(
    iron_deg=20.0, snap_deg=40.0, snap_interval_s=30.0, snap_tau_s=20.0
)

# Turn-on bias magnitude: the MTi-3's ~0.2 deg/s turn-on repeatability, the
# spec behind the heeled-bench ramp that motivated the 3-axis bias state.
_BIAS_DPS = 0.2


@dataclass(frozen=True, slots=True)
class Case:
    """One sweep condition; the grid is the product of the axes below."""

    name: str
    attitude: AttitudeProfile
    bias_dps: Vec3  # body-frame gyro turn-on bias, deg/s
    mag: MagNoiseModel | None
    outage_start_s: float


@dataclass(frozen=True, slots=True)
class Result:
    peak_err_deg: float
    drift_dps: float
    final_err_deg: float
    seed: int


def _source(case: Case, seed: int, duration_s: float) -> SimulatedSource:
    bias_rad = Vec3(
        x=case.bias_dps.x * _DEG_TO_RAD,
        y=case.bias_dps.y * _DEG_TO_RAD,
        z=case.bias_dps.z * _DEG_TO_RAD,
    )
    return SimulatedSource(
        scenario=Scenario(heading=Static(heading_deg=0.0), attitude=case.attitude),
        imu_noise=ImuNoiseModel(
            gyro_white_std_rad_s=MTI3_GYRO_WHITE_STD_RAD_S,
            gyro_constant_bias_rad_s=bias_rad,
            mti_attitude_std_deg=0.5,
            mag=case.mag,
        ),
        gnss_noise=GnssNoiseModel(
            heading_std_deg=1.0, outage_start_s=case.outage_start_s
        ),
        duration_s=duration_s,
        seed=seed,
        gnss_rate_hz=1.0,
    )


def _metrics(trace, outage_start_ms: int, seed: int) -> Result:
    ch = trace.channels["heading"]
    win = trace.t_ms >= outage_start_ms
    resid = wrap180(ch.estimate[win] - ch.truth[win])
    est_unwrapped = np.degrees(np.unwrap(np.radians(ch.estimate[win])))
    dt_s = float(trace.t_ms[win][-1] - trace.t_ms[win][0]) / 1000.0
    drift = abs(est_unwrapped[-1] - est_unwrapped[0]) / dt_s if dt_s > 0.0 else 0.0
    return Result(
        peak_err_deg=float(np.max(np.abs(resid))),
        drift_dps=float(drift),
        final_err_deg=float(abs(resid[-1])),
        seed=seed,
    )


# Roll heel is the sailing regime; trim (pitch) toward vertical is the bench
# regime, where sec(pitch) amplifies error into heading. trim85 is near the
# Euler blind spot the drifty field capture actually sat in.
_ATTITUDES: dict[str, AttitudeProfile] = {
    "level": LevelAttitude(),
    "heel45": ConstantHeel(angle_deg=45.0),
    "heel77": ConstantHeel(angle_deg=77.0),
    "trim45": ConstantTrim(angle_deg=45.0),
    "trim70": ConstantTrim(angle_deg=70.0),
    "trim85": ConstantTrim(angle_deg=85.0),
}
_BIASES: dict[str, Vec3] = {
    "no-bias": Vec3(x=0.0, y=0.0, z=0.0),
    "z-bias": Vec3(x=0.0, y=0.0, z=_BIAS_DPS),
    "y-bias": Vec3(x=0.0, y=_BIAS_DPS, z=0.0),
}
_MAGS: dict[str, MagNoiseModel | None] = {"clean-mag": None, "indoor-mag": _INDOOR_MAG}


def _grid() -> list[Case]:
    cases = []
    for (aname, att), (bname, bias), (mname, mag) in itertools.product(
        _ATTITUDES.items(), _BIASES.items(), _MAGS.items()
    ):
        cases.append(
            Case(
                name=f"{aname}/{bname}/{mname}",
                attitude=att,
                bias_dps=bias,
                mag=mag,
                outage_start_s=30.0,
            )
        )
    return cases


def _baseline_tuning(cfg):
    """Revert the runtime knobs that fixed the field drift, to reproduce it.

    The gates, sigma caps, and 3-axis bias are compiled into the filter and
    cannot be turned off from here; the two config knobs that carried most of
    the fix can. Tight q_offset forces indoor mag wander into heading instead
    of the offset state; the loose bias prior lets a run of fixes retrain the
    gyro bias into a ramp. See tuning.toml and docs/tuning.md.
    """
    yaw = cfg.mti_yaw
    if yaw is not None:
        yaw = replace(yaw, q_offset_deg2=1e-4)
    return replace(cfg, mti_yaw=yaw, p0_bias_deg2_s2=1.0)


def _worst_over_seeds(case: Case, seeds: int, duration_s: float, cfg) -> Result:
    outage_ms = round(case.outage_start_s * 1000.0)
    runs = [
        _metrics(run(_source(case, seed, duration_s), cfg), outage_ms, seed)
        for seed in range(seeds)
    ]
    return max(runs, key=lambda r: r.peak_err_deg)


def _plot_summary(save, seeds: int, duration_s: float, cfg) -> None:
    """Heatmap of peak outage heading error over attitude x bias, indoor mag.

    The sailing rows (level, heel) stay cool whatever the bias: the 3-axis
    filter observes and removes it through the outage. Only the near-vertical
    bench trim rows go hot, where the ZYX kinematics are singular by design.
    """
    import matplotlib.pyplot as plt

    biases = list(_BIASES)
    attitudes = list(_ATTITUDES)
    grid = np.array(
        [
            [
                _worst_over_seeds(
                    Case(
                        name=f"{aname}/{bname}",
                        attitude=_ATTITUDES[aname],
                        bias_dps=_BIASES[bname],
                        mag=_INDOOR_MAG,
                        outage_start_s=30.0,
                    ),
                    seeds,
                    duration_s,
                    cfg,
                ).peak_err_deg
                for bname in biases
            ]
            for aname in attitudes
        ]
    )

    fig, ax = plt.subplots(figsize=(6.0, 5.0))
    im = ax.imshow(grid, cmap="inferno", aspect="auto")
    ax.set_xticks(range(len(biases)), biases)
    ax.set_yticks(range(len(attitudes)), attitudes)
    ax.set_xlabel("gyro turn-on bias")
    ax.set_ylabel("boat attitude")
    ax.set_title("Peak heading error in a 30 s GNSS outage (indoor mag, deg)")
    for i in range(len(attitudes)):
        for j in range(len(biases)):
            ax.text(
                j,
                i,
                f"{grid[i, j]:.0f}",
                ha="center",
                va="center",
                color="white" if grid[i, j] < grid.max() * 0.6 else "black",
            )
    fig.colorbar(im, ax=ax, label="peak error (deg)")
    fig.tight_layout()
    fig.savefig(save, dpi=110)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="drift_sweep")
    p.add_argument("--duration", type=float, default=150.0, metavar="SECONDS")
    p.add_argument("--seeds", type=int, default=2, help="seeds per case; worst wins")
    p.add_argument("--top", type=int, default=0, help="show only the worst N (0=all)")
    p.add_argument(
        "--baseline",
        action="store_true",
        help="revert the drift-fix tuning knobs, to reproduce the field drift",
    )
    p.add_argument(
        "--q-bias",
        type=float,
        default=None,
        metavar="DEG2_S2",
        help="override gyro-bias process noise (deg/s)^2; datasheet 6 deg/h ~= 3e-8",
    )
    p.add_argument(
        "--save",
        type=Path,
        default=None,
        metavar="PATH",
        help="render the attitude x bias heatmap to PATH instead of the table",
    )
    args = p.parse_args(argv)

    cfg = load_tuning()
    if args.baseline:
        cfg = _baseline_tuning(cfg)
    if args.q_bias is not None:
        cfg = replace(cfg, q_bias_deg2_s2=args.q_bias)
    if args.save is not None:
        _plot_summary(args.save, args.seeds, args.duration, cfg)
        return
    ranked = sorted(
        ((c, _worst_over_seeds(c, args.seeds, args.duration, cfg)) for c in _grid()),
        key=lambda cr: cr[1].peak_err_deg,
        reverse=True,
    )
    rows = ranked[: args.top] if args.top > 0 else ranked

    print(f"{'case':28} {'peak_err':>9} {'drift':>8} {'final':>7}  seed")
    print(f"{'':28} {'deg':>9} {'deg/s':>8} {'deg':>7}")
    for case, r in rows:
        print(
            f"{case.name:28} {r.peak_err_deg:9.1f} {r.drift_dps:8.3f} "
            f"{r.final_err_deg:7.1f}  {r.seed}"
        )

    tuning = "baseline (pre-fix)" if args.baseline else "shipped"
    hot = sum(1 for _, r in ranked if r.drift_dps > 0.1)
    print(
        f"\n{len(ranked)} cases, {args.seeds} seeds each, {tuning} tuning; "
        f"{hot} drift > 0.1 deg/s in the outage."
    )


if __name__ == "__main__":
    main()
