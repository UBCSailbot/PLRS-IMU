"""Tests for the 3D boat geometry helpers.

Geometry only -- the drawing is exercised by eye. These pin the rotation
convention so the posed boat matches the filter's ZYX Euler angles.
"""

from __future__ import annotations

import numpy as np
import pytest

from plrs_sim.boat3d import rotate, rotation_matrix


def test_identity_rotation_is_a_noop() -> None:
    pts = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 0.0]])
    assert np.allclose(rotate(pts, 0.0, 0.0, 0.0), pts)


def test_rotation_matrix_is_orthonormal() -> None:
    r = rotation_matrix(20.0, -10.0, 35.0)
    assert np.allclose(r @ r.T, np.eye(3))
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_yaw_turns_bow_toward_port() -> None:
    # +90 deg heading rotates boat-forward (X) onto port (Y).
    bow = rotate(np.array([[1.0, 0.0, 0.0]]), 0.0, 0.0, 90.0)[0]
    assert np.allclose(bow, [0.0, 1.0, 0.0], atol=1e-12)


def test_roll_lifts_port_axis_up() -> None:
    # +90 deg roll about body X rotates port (Y) onto up (Z).
    port = rotate(np.array([[0.0, 1.0, 0.0]]), 90.0, 0.0, 0.0)[0]
    assert np.allclose(port, [0.0, 0.0, 1.0], atol=1e-12)
