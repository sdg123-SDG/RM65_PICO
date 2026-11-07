"""Native PICO session plugin; default adapter is simulation-only.

Real hardware requires the separate explicit real entry and guarded adapter.
"""
from __future__ import annotations

import _thread
import json
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from sim_control import CTRL_DT, PoseTeleop, quat_matrix, fresh
from jog_rm65_mujoco import JogControl
from xr_input import XRoboToolkitInput
from xr_recording import SessionRecorder, new_log_path


class VdoArmAdapter:
    """World-space mapping facade; all motion is sent to native simulation APIs.

    Reference poses come from FK(commanded joints), not gravity sag. Measured
    TCP is exposed separately so recording does not confuse target and motion.
    """
    def __init__(self, native):
        if type(native).__module__ != "simu.rm65_arm" or type(native).__name__ != "Rm65ExtArm":
            raise TypeError("PICO adapter accepts the native simulation Rm65ExtArm only")
        self._bind_native(native)

    def _bind_native(self, native):
        self.native = native
        self.model, self.data = native.model, native.data
        self.base = self.model.body("base_link").id
        self.ee = self.model.body("link_6").id
        self.qadr = np.array([self.model.jnt_qposadr[self.model.joint(f"joint_{i}").id] for i in range(1, 7)])
        self.holding = True
        self.last_ret = 0
        self.position_error = 0.
        self._last_grip = None
        self.tcp_site = None
        with native.lock.data_lock():
            mujoco.mj_forward(self.model, self.data)
            for name in ("eg2_tcp", "rmg24_tcp"):
                site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
                if site >= 0:
                    self.tcp_site = name
                    r = self.data.xmat[self.ee].reshape(3, 3)
                    self.tcp_offset = r.T @ (self.data.site_xpos[site] - self.data.xpos[self.ee])
                    break
        if self.tcp_site is None:
            raise ValueError("Scene needs an eg2_tcp or rmg24_tcp site; no guessed tool offset")

    @property
    def base_rotation(self):
        with self.native.lock.data_lock():
            return self.data.xmat[self.base].reshape(3, 3).copy()

    @property
    def base_position(self):
        with self.native.lock.data_lock():
            return self.data.xpos[self.base].copy()

    def command_world(self):
        fk = self.native.fk_cmd()
        r = self.base_rotation
        p = self.base_position + r @ fk.pos
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, (r @ quat_matrix(fk.quat)).ravel())
        return p, q

    @property
    def pos(self):
        return self.command_world()[0]

    @property
    def quat(self):
        return self.command_world()[1]

    @property
    def tcp_pos(self):
        p, q = self.command_world()
        return p + quat_matrix(q) @ self.tcp_offset

    @property
    def measured_tcp(self):
        with self.native.lock.data_lock():
            return self.data.site_xpos[self.model.site(self.tcp_site).id].copy()

    @property
    def measured_pos(self):
        with self.native.lock.data_lock():
            return self.data.xpos[self.ee].copy()

    def hold(self):
        if not self.holding and not self.native._busy:
            # Stop only our arm stream; do not cancel the independent gripper.
            self.native.Movej_CANFD(self.native.q_deg)
        self.holding = True

    def gripper(self, trigger):
        opening = int(round((1. - np.clip(trigger, 0., 1.)) * 1000))
        if self.native._grip_busy or (self._last_grip is not None and abs(opening-self._last_grip) < 10):
            return
        if self.native.Set_Gripper_Position(opening, block=False) == 0:
            self._last_grip = opening

    def move(self, target, quat, speed):
        from kinemetics.pose import Pose
        from simu.msgs import ArmPose
        self.holding = False
        current, q = self.command_world()
        target = np.asarray(target)
        requested_tip = target + quat_matrix(quat) @ self.tcp_offset
        # Bound angular speed as well as translation when pose mode is requested.
        rotation = quat_matrix(quat) @ quat_matrix(q).T
        dq = np.empty(4)
        mujoco.mju_mat2Quat(dq, rotation.ravel())
        rv = np.empty(3)
        mujoco.mju_quat2Vel(rv, dq, 1.)
        angle = np.linalg.norm(rv)
        if angle > 1e-10:
            half = min(angle, .5 * CTRL_DT) / 2.
            inc = np.r_[np.cos(half), rv / angle * np.sin(half)]
            out = np.empty(4)
            mujoco.mju_mulQuat(out, inc, q)
            quat = out
        if getattr(self, 'preserve_tip_on_rotation', False):
            # PoseTeleop's wrist target corresponds to the requested final
            # orientation; use the rate-limited orientation for this tick.
            target = requested_tip - quat_matrix(quat) @ self.tcp_offset
        delta = target - current
        target = current + delta * min(1., speed * CTRL_DT / max(np.linalg.norm(delta), 1e-12))
        r = self.base_rotation
        base_target = r.T @ (target-self.base_position)
        base_quat = np.empty(4)
        mujoco.mju_mat2Quat(base_quat, (r.T @ quat_matrix(quat)).ravel())
        pose = ArmPose.from_xyz_rpy(base_target, Pose.quat_to_rpy(base_quat))
        self.last_ret = self.native.Movep_CANFD(pose)
        self.position_error = float(np.linalg.norm(np.asarray(target)-self.pos))


class PicoEeServo:
    """Session controller plugin with the same lifecycle hooks as XboxEeServo."""
    def __init__(self, arm, *, mode="teleop", speed=.06, scale=1., wrist="position", yaw=0.,
                 adapter_factory=VdoArmAdapter):
        self.arm = adapter_factory(arm)
        self.arm.preserve_tip_on_rotation = wrist == 'pose'
        self.mode = mode
        self.control = (PoseTeleop(self.arm, frame="robot", scale=scale, speed=speed, mode=wrist, yaw_deg=yaw)
                        if mode == "teleop" else JogControl(self.arm, speed=speed, frame="robot"))
        self.source = None
        self.recorder = None
        self.viewer = None
        self.enabled = False
        self._stop = threading.Event()
        self._thread = None
        self._status_lock = threading.Lock()
        self._status = "PICO waiting"
        self.error = None
        self.overlay = None
        self._axes = None

    def attach_scene_overlay(self, overlay):
        overlay.add(self)

    def attach_overlay(self, overlay):
        self.overlay = overlay

    def drop_data(self):
        self._axes = None

    def paint(self, scene):
        from simu.user_scn import add_arrow
        with self._status_lock:
            axes = self._axes
        if axes is None:
            return
        origin, basis = axes
        for axis, color in zip(basis.T, ([1.,.1,.1,1.],[.1,1.,.1,1.],[.1,.3,1.,1.])):
            add_arrow(scene, origin, origin+.08*axis, .003, np.asarray(color))

    def bind_shell(self, shell):
        shell.register("pico", self.handle_cli, usage="pico on|off|status",
                       help="PICO network teleoperation; Grip=follow, trigger=gripper")

    def handle_cli(self, tokens):
        action = tokens[0] if tokens else "status"
        if action == "on":
            self.start()
        elif action == "off":
            self.stop()
        elif action != "status":
            raise ValueError("pico on|off|status")
        print(f"[pico] enabled={self.enabled} {self._status}", flush=True)
        return 0

    def bind_viewer(self, viewer):
        self.viewer = viewer

    def unbind_viewer(self):
        self.viewer = None

    def draw_overlay(self, viewer):
        if hasattr(viewer, "set_texts"):
            with self._status_lock:
                text = self._status
            viewer.set_texts((None, None, f"PICO {self.mode} | base frame | {self.arm.tcp_site}",
                             text + "\nGrip=follow; release=stop; trigger=gripper\n"
                             "No Head: release Grip, point forward, press stick DOWN once\n"
                             "TCP arrows: red=base +X, green=-Y, blue=+Z\nCLI: pico off / pico on / pico status"))

    def start(self):
        if self.enabled:
            return
        if self.arm.native._busy:
            raise RuntimeError("Finish/stop the current native motion before pico on")
        self.source = XRoboToolkitInput(hand="left")
        self.arm._last_grip = None
        self.control.active = False
        if self.mode == "teleop":
            self.control.need_release = True
            self.recorder = SessionRecorder(new_log_path(),
                {"backend": "vdo", "mode": self.mode, "scene": str(self.arm.native.kin.xml_path),
                 "tcp_site": self.arm.tcp_site, "tcp_offset": self.arm.tcp_offset,
                 **self.record_metadata()}, "live")
        else:
            self.control.need_neutral = True
        self.error = None
        self._stop.clear()
        self.enabled = True
        self._thread = threading.Thread(target=self._loop, name="rm65-pico-servo", daemon=True)
        self._thread.start()
        print(f"[pico] native backend, {self.arm.tcp_site}, offset={self.arm.tcp_offset.round(6)} m; {self.mode}", flush=True)
        print("[pico] Grip must first be released. Missing Head: point forward + press left stick down.", flush=True)

    def _loop(self):
        last_log = last_diag = 0.
        failed = 0
        try:
            while not self._stop.is_set():
                tick = time.monotonic()
                if self.viewer is not None and not self.viewer.is_running():
                    _thread.interrupt_main()
                    break
                raw_sample = self.source.sample()
                sample = self.prepare_sample(raw_sample)
                if self.arm.native._busy:
                    self.arm.holding = True  # do not cancel an owned native CLI trajectory
                    self.control.active = False
                    if hasattr(self.control, "need_release"):
                        self.control.need_release = True
                    if hasattr(self.control, "need_neutral"):
                        self.control.need_neutral = True
                    status = "Native motion busy; release Grip and wait"
                else:
                    self.arm.last_ret = 0
                    self.control.update(sample, 0.)
                    failed = failed + 1 if self.arm.last_ret != 0 else 0
                    status = getattr(self.control, "reason", "jogging" if self.control.active else "neutral")
                    if self.arm.last_ret:
                        status = f"Native IK rejected ret={self.arm.last_ret}; failures={failed}"
                        detail=getattr(self.arm,'motion_reason','')
                        if detail:
                            status+='; '+detail
                    if failed >= 5:
                        self.arm.hold()
                        self.control.active = False
                        if hasattr(self.control, "need_release"):
                            self.control.need_release = True
                        if hasattr(self.control, "need_neutral"):
                            self.control.need_neutral = True
                        status += "; release Grip / neutral to re-arm"
                self.after_control_tick(sample)
                head = "OK" if sample.headset_quat_wxyz is not None else "MISSING"
                heading = getattr(self.control, "heading_source", "base joystick")
                # Native viewer holds the model-data lock while drawing the
                # overlay (which briefly reads _status_lock). Never call hooks
                # that can read model data while holding _status_lock: opposite
                # acquisition order freezes both workers, including shutdown.
                status_text = f"{status}; Head={head}; heading={heading}; Grip={raw_sample.grip:.2f}{self.status_suffix()}"
                with self._status_lock:
                    self._status = status_text
                origin = self.arm.measured_tcp
                yaw=np.deg2rad(getattr(self.control,'yaw_deg',0.))
                basis=self.arm.base_rotation @ np.array([[np.cos(yaw),np.sin(yaw),0.],
                                                         [np.sin(yaw),-np.cos(yaw),0.],[0.,0.,1.]])
                with self._status_lock:
                    self._axes = (origin, basis)
                if self.recorder is not None:
                    with self.arm.native.lock.data_lock():
                        self.recorder.write(raw_sample, self.control, self.arm)
                if self.overlay is not None and self.viewer is not None:
                    self.overlay.redraw()
                if tick-last_log > 2.:
                    print(f"[pico] {self._status}", flush=True)
                    last_log = tick
                if tick-last_diag > 5.:
                    print(self.source.diagnostics_summary(), flush=True)
                    last_diag = tick
                self._stop.wait(max(0., CTRL_DT-(time.monotonic()-tick)))
        except Exception as exc:
            self.error = str(exc)
            try:
                self.on_error(exc)
                if self.recorder is not None:
                    self.recorder.event('error',str(exc))
            except Exception as log_exc:
                print(f'[pico] Could not persist error details: {log_exc}',flush=True)
            print(f"[pico] STOPPED: {exc}", flush=True)
            _thread.interrupt_main()
        finally:
            self.arm.hold()
            self.enabled = False
            if self.source is not None:
                self.source.close()
                self.source = None
            if self.recorder is not None:
                self.recorder.close()
                self.recorder = None

    def stop(self):
        self._stop.set()
        th = self._thread
        if th is not None and th is not threading.current_thread():
            th.join(timeout=2.)
        if th is not None and th.is_alive():
            raise RuntimeError("PICO worker has not stopped; refusing a second worker")
        self._thread = None
        self.arm.hold()
        self.enabled = False

    def release(self):
        self.stop()

    def prepare_sample(self, sample):
        return sample

    def record_metadata(self):
        return {}

    def on_error(self,exc):
        pass

    def after_control_tick(self,sample):
        pass

    def status_suffix(self):
        return ""
