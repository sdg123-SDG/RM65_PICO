"""Gravity-preserving operator heading, following PiperXR's yaw-only design.

References: refs/PiperXR/piper_xr/common/pose_mapping.py (_capture_yaw_align)
and refs/droid/droid/controllers/oculus_controller.py (forward reset).
This module implements the coordinate math independently using numpy/MuJoCo.
"""
from __future__ import annotations

import numpy as np


def heading_basis(quat_wxyz: np.ndarray) -> np.ndarray:
    from sim_control import quat_matrix

    forward = quat_matrix(quat_wxyz) @ np.array([0., 0., -1.])
    forward[1] = 0.
    length = np.linalg.norm(forward)
    if length < 0.15:
        raise ValueError("look/point horizontally to set forward")
    forward /= length
    up = np.array([0., 1., 0.])
    return np.column_stack([forward, np.cross(forward, up), up])


def world_mapping(basis: np.ndarray, base_rotation: np.ndarray, yaw_deg: float = 0.) -> np.ndarray:
    # Robot base: +X forward, -Y right, +Z up. Translation of the base is
    # applied to absolute poses, never to relative hand displacement.
    c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
    yaw = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    target = np.diag([1., -1., 1.])
    mapping = np.asarray(base_rotation) @ yaw @ target @ basis.T
    if not np.allclose(mapping.T @ mapping, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(mapping), 1., atol=1e-6):
        raise ValueError("XR-to-robot mapping must be a proper rotation")
    return mapping
