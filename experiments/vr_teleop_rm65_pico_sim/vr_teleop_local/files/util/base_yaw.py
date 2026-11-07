"""Drive the RM65 base joint from the hand's horizontal azimuth.

The 6-D pose IK tends to satisfy small lateral hand motion with the wrist
joints, so the base barely turns.  In ``hand-azimuth`` mode the base joint is
instead set directly from how far the wrist target has swung around the base
axis since the clutch was engaged; IK then solves only the remaining joints.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HandAzimuthBaseYaw:
    """Map wrist-target azimuth around the base axis to ``joint_1``.

    The mapping is relative: at engage the current base angle is paired with
    the current azimuth of the end effector, so engaging never jumps the base.
    Targets closer to the base axis than ``min_radius_m`` have no reliable
    azimuth; the base then holds its last commanded angle.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_name: str = "joint_1",
        *,
        min_radius_m: float = 0.08,
        limit_margin_deg: float = 3.0,
    ) -> None:
        joint = model.joint(joint_name)
        self.joint_id = int(joint.id)
        self.qpos_index = int(model.jnt_qposadr[self.joint_id])
        self.dof_index = int(model.jnt_dofadr[self.joint_id])
        self.min_radius_m = float(min_radius_m)
        margin = math.radians(limit_margin_deg)
        if model.jnt_limited[self.joint_id]:
            low, high = (float(v) for v in model.jnt_range[self.joint_id])
            self.low, self.high = low + margin, high - margin
        else:
            self.low, self.high = -math.inf, math.inf
        self.anchor: np.ndarray | None = None
        self.axis: np.ndarray | None = None
        self.u: np.ndarray | None = None
        self.v: np.ndarray | None = None
        self.az0: float | None = None
        self.q0 = 0.0
        self.last_q = 0.0

    def engage(self, data: mujoco.MjData, reference_pos: np.ndarray, base_q: float) -> None:
        """Pair the current base angle with the reference position's azimuth.

        ``data`` must already be forwarded so the joint anchor/axis are current.
        """
        axis = np.asarray(data.xaxis[self.joint_id], dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        helper = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        u = helper - axis * float(helper @ axis)
        self.u = u / np.linalg.norm(u)
        self.v = np.cross(axis, self.u)
        self.axis = axis
        self.anchor = np.asarray(data.xanchor[self.joint_id], dtype=np.float64).copy()
        self.q0 = self.last_q = float(np.clip(base_q, self.low, self.high))
        self.az0 = self._azimuth(reference_pos)

    def _azimuth(self, pos: np.ndarray) -> float | None:
        r = np.asarray(pos, dtype=np.float64) - self.anchor
        r = r - self.axis * float(r @ self.axis)
        if np.linalg.norm(r) < self.min_radius_m:
            return None
        return math.atan2(float(r @ self.v), float(r @ self.u))

    def target(self, target_pos: np.ndarray) -> float:
        """Return the base joint angle (rad) for a wrist target position."""
        if self.anchor is None:
            raise RuntimeError("HandAzimuthBaseYaw.engage() must be called first")
        az = self._azimuth(target_pos)
        if az is None:
            return self.last_q
        if self.az0 is None:
            # Engaged too close to the axis: start tracking from here.
            self.az0, self.q0 = az, self.last_q
            return self.last_q
        q = float(np.clip(self.q0 + _wrap(az - self.az0), self.low, self.high))
        self.last_q = q
        return q
