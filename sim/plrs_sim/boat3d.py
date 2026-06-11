"""Wireframe boat geometry for the 3D verification views.

Pure geometry and matplotlib drawing, no filter knowledge. The hull is a
handful of line segments in boat-body axes (X bow, Y port, Z up); rotate()
poses it by ZYX intrinsic Euler angles, matching quaternion_to_euler_zyx.
plot_mounting draws the boat level with the IMU axis triad rotated by the
mount offset and the GNSS antenna baseline arrow, so the static calibration
is visible at a glance.

Headless callers should set MPLBACKEND=Agg before importing this module.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .types import EkfConfig, GnssAttitudeMount

_DEG_TO_RAD = np.pi / 180.0

# Hull in body axes (X bow, Y port, Z up): a deck outline, a flat rectangular
# transom at the stern, and a centerline keel -- enough to read heel, trim,
# and heading.
_VERTICES = np.array(
    [
        [1.0, 0.0, 0.0],    # 0 bow
        [0.5, 0.35, 0.0],   # 1 fwd port
        [-0.9, 0.30, 0.0],  # 2 transom port top
        [-0.9, -0.30, 0.0], # 3 transom stbd top
        [0.5, -0.35, 0.0],  # 4 fwd stbd
        [-0.9, 0.30, -0.30],  # 5 transom port bottom (directly below 2)
        [-0.9, -0.30, -0.30], # 6 transom stbd bottom (directly below 3)
        [0.7, 0.0, -0.25],  # 7 keel fwd
    ]
)

# fmt: off
_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 0),  # deck outline
    (2, 5), (5, 6), (6, 3),                   # transom (top edge 2->3 in deck)
    (0, 7), (7, 5), (7, 6),                   # keel
    (1, 7), (4, 7),                           # hull sides fwd
)
# fmt: on


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX intrinsic rotation (body -> world): Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = np.array([roll_deg, pitch_deg, yaw_deg]) * _DEG_TO_RAD
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def rotate(
    points: np.ndarray, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> np.ndarray:
    """Rotate (N,3) body-frame points into world by the ZYX Euler angles."""
    r = rotation_matrix(roll_deg, pitch_deg, yaw_deg)
    return np.asarray(points, dtype=float) @ r.T


def draw_boat(
    ax,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    *,
    color: str,
    alpha: float = 1.0,
    label: str | None = None,
) -> None:
    """Plot the wireframe hull posed at the given attitude on a 3D axes."""
    v = rotate(_VERTICES, roll_deg, pitch_deg, yaw_deg)
    for k, (i, j) in enumerate(_EDGES):
        ax.plot(
            [v[i, 0], v[j, 0]],
            [v[i, 1], v[j, 1]],
            [v[i, 2], v[j, 2]],
            color=color,
            alpha=alpha,
            linewidth=1.5,
            label=label if k == 0 else None,
        )


def set_equal_3d(ax, half: float = 1.2) -> None:
    """Symmetric limits and a cubic box so the hull is not distorted."""
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_zlim(-half, half)
    ax.set_box_aspect((1.0, 1.0, 1.0))
    ax.set_xlabel("X bow (m)")
    ax.set_ylabel("Y port (m)")
    ax.set_zlabel("Z up (m)")


def plot_mounting(
    cfg: EkfConfig,
    mount: GnssAttitudeMount,
    *,
    show: bool = True,
    save: Path | None = None,
) -> None:
    """Draw a level boat with the IMU mount triad and GNSS baseline arrow."""
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(projection="3d")

    # The level deck is the reference the mount offset is measured against.
    draw_boat(ax, 0.0, 0.0, 0.0, color="0.6", label="boat (level)")

    # IMU axis triad, rotated by the mount offset (boat -> IMU).
    triad = rotate(
        np.eye(3) * 0.6, cfg.mount_roll_deg, cfg.mount_pitch_deg, cfg.mount_yaw_deg
    )
    for vec, color, name in zip(
        triad,
        ("tab:red", "tab:green", "tab:blue"),
        ("IMU X", "IMU Y", "IMU Z"),
        strict=True,
    ):
        ax.quiver(0.0, 0.0, 0.0, *vec, color=color, label=name)

    # GNSS antenna baseline: boat-forward rotated by the offset, in the deck
    # plane. The offset is clockwise from forward (toward starboard), i.e. -yaw.
    baseline = rotate(
        np.array([[1.2, 0.0, 0.0]]), 0.0, 0.0, -mount.baseline_offset_deg
    )[0]
    ax.quiver(
        0.0, 0.0, 0.0, *baseline, color="tab:purple", linewidth=2, label="GNSS baseline"
    )

    set_equal_3d(ax)
    ax.set_title(
        f"Mount r/p/y = {cfg.mount_roll_deg:.0f}/{cfg.mount_pitch_deg:.0f}/"
        f"{cfg.mount_yaw_deg:.0f} deg, baseline offset = "
        f"{mount.baseline_offset_deg:.0f} deg"
    )
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
