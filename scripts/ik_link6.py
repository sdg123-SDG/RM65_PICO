"""Levenberg-Marquardt IK targeting RM65 body ``link_6`` (not flange)."""

from __future__ import annotations

import mujoco
import numpy as np


def _mat_to_quat_wxyz(mat: np.ndarray) -> np.ndarray:
    """3x3 rotation -> quaternion [w, x, y, z]."""
    m = np.asarray(mat, dtype=np.float64).reshape(3, 3)
    q = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(q, m.ravel())
    return q


def body_pose(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> tuple[np.ndarray, np.ndarray]:
    pos = data.xpos[body_id].copy()
    quat = _mat_to_quat_wxyz(data.xmat[body_id])
    return pos, quat


def rotation_error(target_rot: np.ndarray, current_rot: np.ndarray) -> np.ndarray:
    """Angular velocity-style error that rotates current -> target."""
    err = np.zeros(3, dtype=np.float64)
    # mj_jacBody returns angular velocity in WORLD coordinates. mju_subQuat
    # instead returns a tangent in the current quaternion's LOCAL frame.
    relative = np.asarray(target_rot).reshape(3, 3) @ np.asarray(current_rot).reshape(3, 3).T
    mujoco.mju_quat2Vel(err, _mat_to_quat_wxyz(relative), 1.0)
    return err


def solve_pose_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    target_pos: np.ndarray,
    target_quat_wxyz: np.ndarray,
    q_arm: np.ndarray,
    *,
    arm_qpos_addrs: np.ndarray,
    arm_dof_ids: np.ndarray,
    max_iters: int = 40,
    tol: float = 1e-3,
    damping: float = 1e-3,
    rot_weight: float = 0.5,
    scratch: mujoco.MjData | None = None,
) -> np.ndarray:
    """Solve IK for the 6 arm joints; returns updated arm qpos (6,)."""
    # Never overwrite the live physics state to evaluate candidate IK poses.
    live = data
    data = scratch if scratch is not None else mujoco.MjData(model)
    if data is live:
        raise ValueError("IK scratch must be separate from live MjData")
    data.qpos[:] = live.qpos
    q = np.asarray(q_arm, dtype=np.float64).copy()
    target_rot = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(target_rot, np.asarray(target_quat_wxyz, dtype=np.float64))
    target_rot = target_rot.reshape(3, 3)

    for _ in range(max_iters):
        for i, addr in enumerate(arm_qpos_addrs):
            data.qpos[addr] = q[i]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        err_pos = target_pos - data.xpos[body_id]
        cur_rot = data.xmat[body_id].reshape(3, 3)
        err_rot = rotation_error(target_rot, cur_rot)
        err = np.hstack([err_pos, rot_weight * err_rot])
        if np.linalg.norm(err) < tol:
            break

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        jac = np.vstack([jacp, rot_weight * jacr])[:, arm_dof_ids]

        jjt = jac @ jac.T + damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jjt, err)
        size = float(np.linalg.norm(dq))
        q = q + dq * min(1.0, 0.12 / max(size, 1e-12))

        # Soft joint limits from model.
        for i, jnt_id in enumerate(
            [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{k}") for k in range(1, 7)]
        ):
            lo, hi = model.jnt_range[jnt_id]
            if model.jnt_limited[jnt_id]:
                q[i] = float(np.clip(q[i], lo, hi))
    return q
