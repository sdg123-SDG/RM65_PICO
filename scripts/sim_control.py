"""Shared, simulation-only RM65 servo and XR coordinate mapping.

Position mode holds the initial wrist orientation, so translating the wrist
does not unintentionally swing the gripper tip. All positions are WORLD meters.
"""
from __future__ import annotations

from pathlib import Path
import os
import time

import mujoco
import numpy as np

from home_pose import HOME_JOINTS_RAD
from ik_link6 import solve_pose_ik

ROOT = Path(__file__).resolve().parents[1]
SCENE = Path(os.environ.get("RM65_SCENE", str(ROOT / "assets/scene/custom_rm65_box/scene_gemini_gravcomp.xml")))
CTRL_DT = 0.02
XR_TO_ROBOT = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def view_axes(azimuth_deg: float) -> tuple[np.ndarray, np.ndarray]:
    az = np.deg2rad(azimuth_deg)
    # MuJoCo camera.forward projects to (+cos(az), +sin(az)), not its negative.
    # camera.forward x camera.up is screen-right.
    return np.array([np.cos(az), np.sin(az), 0.]), np.array([np.sin(az), -np.cos(az), 0.])


def quat_matrix(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-6:
        raise ValueError("invalid quaternion")
    out = np.empty(9)
    mujoco.mju_quat2Mat(out, q / np.linalg.norm(q))
    return out.reshape(3, 3)


def controller_map(head_quat: np.ndarray | None, frame: str, azimuth: float, yaw_deg: float = 0.) -> np.ndarray:
    """Capture operator heading at Grip press; keep it fixed until release.

    Head and controller are expressed in XR tracking WORLD, not head-local.
    Missing Head data uses XR -Z as forward; --yaw-deg then calibrates that.
    """
    forward_xr = np.array([0., 0., -1.])
    if head_quat is not None:
        forward_xr = quat_matrix(head_quat) @ forward_xr
    heading = XR_TO_ROBOT @ forward_xr
    angle = np.arctan2(heading[1], heading[0]) if np.linalg.norm(heading[:2]) > 1e-4 else 0.
    forward = view_axes(azimuth)[0] if frame == "view" else np.array([1., 0., 0.])
    yaw = np.arctan2(forward[1], forward[0]) - angle + np.deg2rad(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ XR_TO_ROBOT


def fresh(sample, timeout: float = 0.25) -> bool:
    age = time.time() - sample.stamp
    return bool(sample.valid and np.isfinite(age) and 0. <= age < timeout)


class SimArm:
    def __init__(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        m = self.model
        self.data = mujoco.MjData(m)
        self.scratch = mujoco.MjData(m)
        self.ee = m.body("link_6").id
        self.base = m.body("base_link").id
        # RMG24's documented pinch point, expressed in the link_6/flange frame.
        self.tcp_offset = np.array([float(x) for x in os.environ.get("RM65_TCP_OFFSET", "0,0,0.137").split(",")])
        if self.tcp_offset.shape != (3,) or not np.all(np.isfinite(self.tcp_offset)):
            raise ValueError("RM65_TCP_OFFSET must contain three finite numbers in meters")
        # Old attached tool-local arrows were labelled as control axes, which
        # is misleading for base-frame translation. The viewer draws base axes.
        for gid in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if name.startswith("ee_axis_"):
                m.geom_rgba[gid, 3] = 0.
        ids = np.array([m.joint(f"joint_{i}").id for i in range(1, 7)])
        self.qadr = m.jnt_qposadr[ids]
        self.dofs = m.jnt_dofadr[ids]
        self.acts = np.array([m.actuator(f"joint_{i}_ctrl").id for i in range(1, 7)])
        self.gact = m.actuator("rmg24_ctrl").id
        self.limits = m.jnt_range[ids].copy()
        # The stock position servos lag noticeably under 6DoF jogging. Raise
        # the simulation-only servo bandwidth while preserving damping ratio.
        m.actuator_gainprm[self.acts, 0] *= 2.0
        m.actuator_biasprm[self.acts, 1] *= 2.0
        m.actuator_biasprm[self.acts, 2] *= np.sqrt(2.0)
        # Imported camera/adapter bodies were omitted from the gravcomp XML.
        # Compensate the COMPLETE arm subtree, preserving gravity on the box.
        base = m.body("base_link").id
        for b in range(1, m.nbody):
            parent = b
            while parent > 0 and parent != base:
                parent = m.body_parentid[parent]
            if parent == base:
                m.body_gravcomp[b] = 1.
        mujoco.mj_resetDataKeyframe(m, self.data, m.key("home").id)
        self.data.qpos[self.qadr] = HOME_JOINTS_RAD
        self.data.qpos[m.joint("rmg24_left").qposadr[0]] = -0.0325
        self.data.qpos[m.joint("rmg24_right").qposadr[0]] = 0.0325
        self.data.ctrl[self.gact] = 0.065
        self.qcmd = self.data.qpos[self.qadr].copy()
        self.data.ctrl[self.acts] = self.qcmd
        mujoco.mj_forward(m, self.data)
        self.holding = True
        self.position_error = 0.
        self.command_pos = self.pos

    @property
    def pos(self) -> np.ndarray:
        return self.data.xpos[self.ee].copy()

    @property
    def quat(self) -> np.ndarray:
        return self.data.xquat[self.ee].copy()

    @property
    def base_rotation(self) -> np.ndarray:
        return self.data.xmat[self.base].reshape(3, 3).copy()

    @property
    def tcp_pos(self) -> np.ndarray:
        return self.pos + self.data.xmat[self.ee].reshape(3, 3) @ self.tcp_offset

    def hold(self) -> None:
        if not self.holding:
            self.qcmd = self.data.qpos[self.qadr].copy()
            # Explicit simulation stop on release; IK itself never changes qvel.
            self.data.qvel[self.dofs] = 0.
            self.data.ctrl[self.acts] = self.qcmd
        self.holding = True
        self.command_pos = self.pos
        self.position_error = 0.

    def move(self, target: np.ndarray, quat: np.ndarray, speed: float) -> None:
        d, m = self.data, self.model
        self.holding = False
        p = self.pos
        target = np.clip(np.asarray(target), [-0.50, -0.50, 0.88], [0.65, 0.50, 1.45])
        distance = np.linalg.norm(target - p)
        # Advance a Cartesian reference at the control rate. Previously a fresh
        # 120-ms lookahead from the ACTUAL pose was used each tick; unequal
        # joint lag bent an otherwise straight command into a curved path.
        delta = target - self.command_pos
        target = self.command_pos + delta * min(1., speed * CTRL_DT / max(np.linalg.norm(delta), 1e-12))
        # Bound outstanding Cartesian command to avoid a long catch-up motion.
        gap = target - p
        target = p + gap * min(1., speed * 0.12 / max(np.linalg.norm(gap), 1e-12))
        self.command_pos = target.copy()
        q = solve_pose_ik(m, d, self.ee, target, quat, d.qpos[self.qadr],
                          arm_qpos_addrs=self.qadr, arm_dof_ids=self.dofs,
                          scratch=self.scratch, max_iters=25, tol=0.0001,
                          damping=1e-5, rot_weight=0.25)
        # Bound joint velocity and the distance from actual joints independently.
        previous_qcmd = self.qcmd.copy()
        step = q - self.qcmd
        step *= min(1., CTRL_DT * 2.0 / max(np.max(np.abs(step)), 1e-12))
        self.qcmd = np.clip(self.qcmd + step, d.qpos[self.qadr] - 0.10, d.qpos[self.qadr] + 0.10)
        self.qcmd = np.clip(self.qcmd, self.limits[:, 0], self.limits[:, 1])
        # Position servos also contain -kv*qvel. Compensate that term for the
        # reference velocity, otherwise different kv/kp ratios make individual
        # joints lag by different amounts and bend Cartesian translations.
        velocity = (self.qcmd - previous_qcmd) / CTRL_DT
        lead = -m.actuator_biasprm[self.acts, 2] / m.actuator_gainprm[self.acts, 0]
        servo = np.clip(self.qcmd + lead * velocity, d.qpos[self.qadr] - 0.10, d.qpos[self.qadr] + 0.10)
        d.ctrl[self.acts] = np.clip(servo, self.limits[:, 0], self.limits[:, 1])
        self.position_error = float(distance)

    def gripper(self, trigger: float) -> None:
        if np.isfinite(trigger):
            self.data.ctrl[self.gact] = 0.065 * (1. - np.clip(trigger, 0., 1.))

    def step(self) -> None:
        for _ in range(max(1, round(CTRL_DT / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)


class PoseTeleop:
    def __init__(self, arm: SimArm, *, frame="robot", scale=1., speed=0.06, mode="position", yaw_deg=0., require_heading=True):
        self.arm, self.frame, self.scale, self.speed = arm, frame, scale, speed
        self.mode, self.yaw_deg = mode, yaw_deg
        self.active = False
        self.reason = "waiting"
        self.previous_pos = None
        self.previous_stamp = None
        self.need_release = False
        self.direction_basis = None
        self.require_heading = require_heading
        self.heading_source = "waiting for Head Tracking"
        self._last_click = False
        self._last_auto = False
        self.target_world = self.arm.pos
        self.target_tcp = getattr(self.arm, "tcp_pos", self.arm.pos)
        self.operator_delta = np.zeros(3)
        self.actual_delta = np.zeros(3)
        self.capture_basis = None

    def set_direction_basis(self, basis: np.ndarray) -> None:
        """Operator forward/right/up expressed in the incoming tracking world."""
        self.arm.hold()
        self.active = False
        self.direction_basis = np.asarray(basis, dtype=float).copy()
        self.heading_source = "manual forward"

    def update(self, sample, azimuth: float) -> None:
        ok = fresh(sample) and sample.pos is not None and sample.quat_wxyz is not None
        if ok:
            self.arm.gripper(sample.trigger)
        grip = sample.grip > (0.15 if self.active else 0.25)
        # DROID-style explicit forward reset: point controller forward and click
        # its stick with Grip released. No motion strokes or pitch calibration.
        click = bool(getattr(sample, "axis_click", False))
        auto = bool(getattr(sample, "primary_button", False))
        click_edge, auto_edge = click and not self._last_click, auto and not self._last_auto
        self._last_click, self._last_auto = click, auto
        if ok and not grip:
            if auto_edge:
                self.direction_basis = None
                self.heading_source = "automatic Head yaw"
            elif click_edge:
                from xr_heading import heading_basis
                try:
                    self.set_direction_basis(heading_basis(sample.quat_wxyz))
                except ValueError as exc:
                    self.arm.hold()
                    self.reason = str(exc)
                    return
        if not ok or not grip or sample.menu:
            self.arm.hold()
            self.active = False
            self.reason = "no fresh tracking" if not ok else "released"
            if not ok:
                self.need_release = True
            elif not grip:
                self.need_release = False
            return
        if self.need_release:
            self.reason = "release Grip to re-arm"
            return
        pos = np.asarray(sample.pos)
        if not self.active:
            from xr_heading import heading_basis, world_mapping
            basis = self.direction_basis
            if basis is None and sample.headset_quat_wxyz is not None:
                try:
                    basis = heading_basis(sample.headset_quat_wxyz)
                    self.heading_source = "Head yaw (frozen during Grip)"
                except ValueError:
                    basis = None
            if basis is None and self.require_heading:
                self.arm.hold()
                self.heading_source = "MISSING Head"
                self.reason = "enable Head Tracking OR release Grip + point forward + click stick"
                return
            if basis is None:
                self.mapping = controller_map(None, self.frame, azimuth, self.yaw_deg)
                basis = np.column_stack([[0., 0., -1.], [1., 0., 0.], [0., 1., 0.]])
            elif self.frame == "view":
                from xr_direction_calibration import calibrated_map
                self.mapping = calibrated_map(basis, self.frame, azimuth, self.yaw_deg)
            else:
                self.mapping = world_mapping(basis, getattr(self.arm, "base_rotation", np.eye(3)), self.yaw_deg)
            self.capture_basis = basis.copy()
            self.ref_ctrl, self.ref_ee = pos.copy(), self.arm.pos
            self.ref_tcp = getattr(self.arm, "tcp_pos", self.arm.pos).copy()
            self.ref_rotation = quat_matrix(sample.quat_wxyz)
            self.ref_ee_rotation = quat_matrix(self.arm.quat)
            self.hold_quat = self.arm.quat
            self.previous_pos = pos.copy()
            self.previous_stamp = sample.stamp
            self.active = True
        # A tracking-origin reset must not become a robot motion command.
        if sample.stamp != self.previous_stamp:
            if np.linalg.norm(pos - self.previous_pos) > 0.15:
                self.arm.hold()
                self.active = False
                self.need_release = True
                self.reason = "tracking jump: release Grip"
                return
            self.previous_pos, self.previous_stamp = pos.copy(), sample.stamp
        delta = self.mapping @ (pos - self.ref_ctrl) * self.scale
        if np.linalg.norm(delta) < 0.001:
            delta[:] = 0.
        target = self.ref_ee + delta
        self.target_tcp = self.ref_tcp + delta
        quat = self.hold_quat
        if self.mode == "pose":
            rotation = self.mapping @ (quat_matrix(sample.quat_wxyz) @ self.ref_rotation.T) @ self.mapping.T
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, (rotation @ self.ref_ee_rotation).ravel())
            # Rotate around the gripper pinch point rather than orbiting it
            # around the wrist. Position mode has an unchanged fixed offset.
            if hasattr(self.arm, "tcp_offset"):
                target = self.target_tcp - quat_matrix(quat) @ self.arm.tcp_offset
        self.target_world = target.copy()
        self.operator_delta = self.capture_basis.T @ (pos - self.ref_ctrl) * self.scale
        self.arm.move(target, quat, self.speed)
        self.reason = "following"

    def motion_summary(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.active or self.capture_basis is None:
            return np.zeros(3), np.zeros(3)
        target_basis = self.mapping @ self.capture_basis
        tool = getattr(self.arm, "tcp_pos", self.arm.pos)
        self.actual_delta = target_basis.T @ (tool - self.ref_tcp)
        return self.operator_delta.copy(), self.actual_delta.copy()
