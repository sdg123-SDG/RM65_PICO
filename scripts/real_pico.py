"""Conservative real-arm adapter. No hardware connection occurs on import.

This entry verifies physical-angle FK and uses a dedicated no-legacy-sign plant.
Streaming uses bounded sendall (no fabricated acknowledgement), plus feedback.
"""
from __future__ import annotations
import _thread
from contextlib import contextmanager
from dataclasses import replace
import json
import threading
import time

import mujoco
import numpy as np

from sim_control import CTRL_DT, fresh, quat_matrix
from vdo_pico import VdoArmAdapter, PicoEeServo


class RealPicoFault(RuntimeError):
    pass


def validate_settings(cfg):
    limits = {
        "linear_speed_m_s": (.001,.03), "scale": (.05,1.),
        "joint_speed_deg_s": (.1,10.), "max_joint_step_deg": (.01,.25),
        "max_following_error_deg": (.1,3.), "state_timeout_s": (.05,.3),
        "max_tcp_following_m": (.002,.02),
        "io_timeout_s": (.02,.2), "max_stream_gap_s": (.05,.3),
        "session_radius_m": (.005,.10),
    }
    for key, (lo, hi) in limits.items():
        value = float(cfg[key])
        if not np.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{key} must be finite in [{lo}, {hi}]")
    box = cfg["workspace_base_m"]
    if box is not None:
        lo, hi = np.asarray(box["min"], float), np.asarray(box["max"], float)
        if lo.shape != (3,) or hi.shape != (3,) or not np.all(np.isfinite([lo,hi])) or np.any(lo >= hi):
            raise ValueError("workspace_base_m requires finite min < max, three meters each")
    if cfg.get('activation','grip') not in ('grip','cli'):
        raise ValueError('activation must be grip or cli')
    if cfg.get('robot_forward','+X') not in ('+X','-X'):
        raise ValueError('robot_forward must be +X or -X')
    if not isinstance(cfg["gripper_enabled"], bool):
        raise ValueError("gripper_enabled must be a JSON boolean")
    return cfg


class BoundedLock:
    def __init__(self, timeout):
        self.lock = threading.Lock()
        self.timeout = timeout

    def __enter__(self):
        if not self.lock.acquire(timeout=self.timeout):
            raise RealPicoFault("Robot I/O lock deadline exceeded")
        return self

    def __exit__(self, *args):
        self.lock.release()


class TimedStateMixin:
    def put(self, value):
        with self._lock:
            # Native world mirror republishes Get_Current_Arm_State's cached
            # object. Only a newly decoded network sample refreshes freshness.
            if value is not self._value:
                self.received_at = time.monotonic()
            self._value = value
            self._seq += 1
            return self._seq

    def snapshot(self):
        with self._lock:
            return self._value, self.received_at


class GuardedLink:
    """Serialize with the existing JSON parser; stop treating missing replies as OK."""
    def __init__(self, raw, timeout):
        self.raw = raw
        self.timeout = timeout
        self.armed = False
        self.closed = False
        self.violation = None
        self._local = threading.local()
        raw._lock = BoundedLock(min(timeout,.03))
        raw._sock.settimeout(timeout)

    @property
    def _sock(self):
        return self.raw._sock

    @contextmanager
    def teleop(self):
        before = getattr(self._local, "owned", False)
        self._local.owned = True
        try:
            yield
        finally:
            self._local.owned = before

    def _authorize(self, payload):
        if self.closed:
            raise RealPicoFault("Robot connection is closed")
        cmd = str(payload.get("command", ""))
        if cmd=='set_arm_stop':
            self.violation='Native emergency stop requested; PICO cannot resume automatically'
            return  # Never let writer ownership block the native CLI stop command.
        if self.armed and not cmd.startswith("get_") and not getattr(self._local, "owned", False):
            self.violation = "PICO armed: other motion/configuration writers are blocked; pico disarm first"
            raise RealPicoFault(self.violation)

    def rpc(self, payload, *, timeout_s=None):
        self._authorize(payload)
        timeout = min(self.timeout, timeout_s) if timeout_s is not None else self.timeout
        return self.raw.rpc(payload, timeout_s=timeout)

    def recv_json(self, *, timeout_s=None):
        return self.raw.recv_json(timeout_s=min(timeout_s or self.timeout, self.timeout))

    def send_stream(self, payload):
        self._authorize(payload)
        line = json.dumps(payload, separators=(",",":"), allow_nan=False).encode() + b"\r\n"
        with self.raw._lock:
            previous = self.raw._sock.gettimeout()
            try:
                self.raw._sock.settimeout(self.timeout)
                self.raw._sock.sendall(line)
            except Exception as exc:
                raise RealPicoFault(f"CANFD send failed (not success): {exc}") from exc
            finally:
                if not self.closed:
                    self.raw._sock.settimeout(previous)

    def close(self):
        self.closed = True
        self.raw.close()

    def abort(self):
        # No second concurrent motion writer: interrupt blocked I/O and let
        # the original session shutdown/guard attempt its safe-mode recovery.
        self.closed = True
        try:
            import socket
            self.raw._sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        self.raw.close()


def strict_plant(link):
    from real.json_plant import RealPlant
    from real.json_codec import encode_joint_deg
    class StrictPlant(RealPlant):
        def rpc(self, payload, *, timeout_s=None):
            # The current model FK matches controller angles directly. Keep this
            # convention inside this real entry only; do NOT apply legacy MDH signs.
            return self.link.rpc(payload,timeout_s=timeout_s)

        def canfd(self, joint_deg, *, follow=False):
            payload = {"command":"movej_canfd", "joint":encode_joint_deg(joint_deg), "follow":False}
            self.link.send_stream(payload)
            return 0  # locally sent, not controller acknowledgement / reached

        def leave_passthrough(self, joint_deg):
            q = np.asarray(joint_deg, float)
            if q.shape != (6,) or not np.all(np.isfinite(q)):
                raise RealPicoFault("Invalid measured joint hold")
            msg = self.rpc({"command":"movej", "joint":encode_joint_deg(q),
                            "v":10, "r":0, "trajectory_connect":0}, timeout_s=link.timeout)
            if not (msg.get("movej") is True or msg.get("receive_state") is True):
                raise RealPicoFault(f"Hold not acknowledged: {msg}")
            return 0
    return StrictPlant(link)


class RealPicoAdapter(VdoArmAdapter):
    def __init__(self, native, cfg, *, inventory=None):
        from real.real_arm import RealArm
        from simu.rm65_arm import Rm65ExtArm
        from simu.mailbox import LatestBox
        if type(native) is not RealArm:
            raise TypeError("Real PICO entry requires RealArm, no simulation fallback")
        validate_settings(cfg)
        from real_robot_profile import collect_on_link, hardware_profile
        self.inventory = inventory if inventory is not None else collect_on_link(native.link)
        self.hardware = hardware_profile(self.inventory)
        self.cfg = cfg
        self.armed = False
        self.fault = None
        self.sent = False
        self.gripper_allowed = False
        self.last_send = None
        self.last_grip_time = 0.
        self.motion_reason='No motion requested'
        self.frames_sent=0
        self.gripper_feedback=None
        self.gripper_label='UNKNOWN; OPEN VISUAL PREVIEW ONLY (no gripper command)'
        self.reason = "WAIT: live tracking + set forward + release Grip/trigger"
        class TimedBox(TimedStateMixin, LatestBox):
            received_at = 0.
        self.state = TimedBox()
        native.arm_state = self.state
        self.transport = GuardedLink(native.link, float(cfg["io_timeout_s"]))
        native.link = self.transport
        native.plant = strict_plant(self.transport)
        native.policy.plant = native.plant
        # Replace the initial legacy-signed measurement with a fresh physical-angle state.
        native.fetch_arm_state()
        self._bind_native(native)
        self.tcp_offset = self.hardware['tool_transform'][:3,3].copy()
        self.update_gripper_visual(self.inventory['responses'].get('get_gripper_state'))
        if cfg['gripper_enabled'] and self.gripper_feedback is None:
            raise RealPicoFault('Gripper control requires valid live gripper feedback; no automatic open/close sent')
        shadow_data = mujoco.MjData(native.model)
        with native.lock.data_lock():
            mujoco.mj_copyData(shadow_data, native.model, native.data)
        self.shadow = Rm65ExtArm(xml_path=native.kin.xml_path, model=native.model, data=shadow_data)
        # Real commissioning steps are below the native Movep_CANFD's 1 mm
        # acceptance tolerance. Use its isolated solver at tighter tolerance,
        # then independently bound joints before the hardware boundary.
        self.shadow.kin.pos_tol = 1e-5
        self.shadow.kin.rot_tol = 1e-4
        self.shadow.kin.damping = .02
        self.q_min = np.maximum(native.kin.q_min,np.deg2rad(self.hardware['joint_min_deg']))
        self.q_max = np.minimum(native.kin.q_max,np.deg2rad(self.hardware['joint_max_deg']))
        if np.any(self.q_min>=self.q_max):
            raise RealPicoFault('Hardware/model joint limits do not intersect')
        self.shadow.kin._q_min=self.q_min.copy()
        self.shadow.kin._q_max=self.q_max.copy()
        state=self.checked_state()
        self.shadow.set_q(np.deg2rad(state.joint))
        self.fk_validation = self.verify_model(state)
        from real_robot_profile import posture_metrics
        self.posture=posture_metrics(self.shadow.kin,state.joint)
        self.startup_tcp = self.tcp_pos.copy()
        self.anchor_tcp = None
        print('[pico-real] hardware FK checked:', self.fk_validation, flush=True)
        print(f"[pico-real] Controller TCP={self.hardware['tool']['tool_name']} offset={self.tcp_offset}; "
              f"forward={cfg.get('robot_forward','+X')}; radius={cfg['session_radius_m']}m",flush=True)
        if self.hardware['collision_stage']==0:
            print('[pico-real] WARNING: controller collision stage is 0; software cannot replace a physical emergency stop.',flush=True)
        if self.posture['near_singular']:
            print('[pico-real] NEAR SINGULAR: some translation directions may be unreachable. '
                  'No automatic home/reposition. Prepare a bent pose with a supervised procedure.',flush=True)
        print('[pico-real] Gripper mirror:',self.gripper_label,flush=True)

    def update_gripper_visual(self,reply):
        valid=isinstance(reply,dict) and reply.get('enable')==1 and reply.get('status')==1 and reply.get('error')==0
        opening=reply.get('actpos') if valid else None
        valid=valid and isinstance(opening,(int,float)) and np.isfinite(opening) and 0<=opening<=1000
        self.gripper_feedback=reply if valid else None
        self.gripper_label=(f'MEASURED startup opening={opening}/1000; not continuously sampled' if valid else
                            'UNKNOWN; OPEN VISUAL PREVIEW ONLY (no gripper command)')
        width=self.native._units_to_m(opening if valid else 1000)
        # The native mirror does not integrate finger physics. Pin fingers, not
        # just ctrl; NEVER call RealArm.Set_Gripper_* to change the display.
        with self.native.lock.data_lock():
            pair=self.native._grip_qadr
            if pair is not None:
                self.native.data.qpos[pair[0]]=-width/2
                self.native.data.qpos[pair[1]]=width/2
                for name in ('eg2_left','eg2_right'):
                    jid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,name)
                    if jid>=0:
                        self.native.data.qvel[self.model.jnt_dofadr[jid]]=0.
            if self.native._grip_act>=0:
                self.native.data.ctrl[self.native._grip_act]=width
            mujoco.mj_forward(self.model,self.data)

    def reject(self,reason):
        self.last_ret=1
        self.motion_reason=reason

    def verify_model(self, state):
        from real_robot_profile import check_fk
        p=state.pose
        raw=[p.px*1e6,p.py*1e6,p.pz*1e6,p.rx*1000,p.ry*1000,p.rz*1000]
        try:
            return check_fk(self.shadow.kin,state.joint,raw,self.hardware)
        except ValueError as exc:
            raise RealPicoFault(str(exc)) from exc

    @property
    def measured_tcp(self):
        st=self.state.get()
        if st is None:
            return super().measured_tcp
        return self.base_position+self.base_rotation@np.array([st.pose.px,st.pose.py,st.pose.pz])

    def checked_state(self):
        st, stamp = self.state.snapshot()
        if st is None or not 0 <= time.monotonic()-stamp <= self.cfg["state_timeout_s"]:
            raise RealPicoFault("Robot feedback stale / missing")
        if np.asarray(st.joint).shape != (6,) or not np.all(np.isfinite(st.joint)):
            raise RealPicoFault("Invalid measured joints")
        if st.arm_err or st.sys_err:
            raise RealPicoFault(f"Controller error arm={st.arm_err} sys={st.sys_err}")
        return st

    def command_world(self):
        if not hasattr(self, "shadow"):
            return super().command_world()
        fk = self.shadow.fk_cmd()
        r = self.base_rotation
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, (r @ quat_matrix(fk.quat)).ravel())
        return self.base_position+r@fk.pos, q

    def check_workspace(self, tcp):
        p = self.base_rotation.T @ (tcp-self.base_position)
        box = self.cfg["workspace_base_m"]
        if box is not None and (np.any(p < np.asarray(box["min"])) or np.any(p > np.asarray(box["max"]))):
            raise RealPicoFault(f"TCP outside configured base workspace: {p}")
        if hasattr(self,'startup_tcp') and np.linalg.norm(tcp-self.startup_tcp) > self.cfg['session_radius_m']:
            raise RealPicoFault('Startup displacement limit exceeded; cannot walk the boundary by re-gripping')
        if self.anchor_tcp is not None and np.linalg.norm(tcp-self.anchor_tcp) > self.cfg["session_radius_m"]:
            raise RealPicoFault("Session displacement limit exceeded")

    def arm(self):
        if self.armed:
            raise RealPicoFault("Already armed; pico disarm before re-arming")
        if self.fault:
            raise RealPicoFault(self.fault)
        if self.transport.violation:
            raise RealPicoFault(self.transport.violation)
        st = self.checked_state()
        self.verify_model(st)
        if np.max(np.abs(self.native._zero_rad)) > 1e-8:
            raise RealPicoFault("Custom joint zero offsets need separate commissioning")
        if self.native._busy:
            raise RealPicoFault("Native motion is busy")
        q = np.deg2rad(st.joint) + self.native._zero_rad
        if np.any(q < self.q_min) or np.any(q > self.q_max):
            raise RealPicoFault("Measured joints outside model limits")
        self.shadow.set_q(q)
        self.anchor_tcp = None
        self.check_workspace(self.tcp_pos)
        self.anchor_tcp = self.tcp_pos.copy()
        self.armed = self.transport.armed = True
        self.sent = False
        self.holding = True
        self.last_send = None
        self._last_grip = None
        self.reason = "ARMED; Grip=deadman (position only)"

    def disarm(self, reason="DISARMED"):
        try:
            self.hold()
        finally:
            self.armed = self.transport.armed = False
            self.gripper_allowed = False
            self.reason = reason

    def hold(self):
        try:
            if self.sent:
                try:
                    state = self.checked_state()
                except RealPicoFault:
                    self.native.fetch_arm_state()  # bounded fresh read, no stale-angle fallback
                    state = self.checked_state()
                with self.transport.teleop():
                    ret = self.native.leave_passthrough()
                if ret != 0:
                    raise RealPicoFault(f"leave_passthrough failed: {ret}")
                self.shadow.set_q(np.deg2rad(state.joint))
        except Exception as exc:
            self.fault = f"Hold unconfirmed: {exc}"
            print(f"[pico-real] {self.fault}; original shutdown/guard must recover", flush=True)
            self.transport.abort()
        finally:
            self.sent = False
            self.holding = True
            self.last_send = None

    def gripper(self, trigger):
        if not (self.armed and self.gripper_allowed and self.cfg["gripper_enabled"]):
            return
        self.checked_state()
        now = time.monotonic()
        target = int(round((1.-np.clip(trigger,0.,1.))*1000))
        if now-self.last_grip_time < .2 or (self._last_grip is not None and abs(target-self._last_grip)<20):
            return
        with self.transport.teleop():
            # Use strict RPC directly: original rpc_cmd can accept a timeout.
            msg = self.native.plant.rpc({"command":"gripper_position", "position":target})
        if msg.get("gripper_position") is not True:
            raise RealPicoFault(f"Gripper command unconfirmed: {msg}")
        self._last_grip, self.last_grip_time = target, now

    def move(self, target, quat, speed):
        if not self.armed:
            return
        from kinemetics.pose import Pose
        st = self.checked_state()
        self.verify_model(st)
        if self.fault or self.native._busy:
            raise RealPicoFault(self.fault or "Competing native motion")
        now = time.monotonic()
        if self.last_send is not None and now-self.last_send > self.cfg["max_stream_gap_s"]:
            raise RealPicoFault("Control loop gap exceeded; no catch-up frame sent")
        q0 = self.shadow.q_cmd_deg.copy()
        if np.max(np.abs(q0-st.joint)) > self.cfg["max_following_error_deg"]:
            raise RealPicoFault("Measured joint following error exceeded")
        current, held_quat = self.command_world()
        # Real entry intentionally supports translation only for commissioning.
        if abs(float(held_quat @ np.asarray(quat))) < .999:
            raise RealPicoFault("Real entry locks wrist orientation")
        self.check_workspace(np.asarray(target)+quat_matrix(quat)@self.tcp_offset)
        d = np.asarray(target)-current
        if np.linalg.norm(d)<5e-5:
            self.last_ret=0
            self.motion_reason='At target / no frame needed'
            return  # enabling/reference capture is not a reason to send a frame
        step = min(speed, self.cfg["linear_speed_m_s"])*CTRL_DT
        target = current+d*min(1.,step/max(np.linalg.norm(d),1e-12))
        r = self.base_rotation
        local_q = np.empty(4)
        mujoco.mju_mat2Quat(local_q, (r.T@quat_matrix(quat)).ravel())
        pose = Pose(r.T@(target-self.base_position), local_q)
        kin = self.shadow.kin
        result = kin.solve_isolated(pose, q_seed=np.deg2rad(q0), max_iterations=25, keep_on_fail=True)
        if not result.success:
            self.reject(f'IK BLOCKED: target unreachable or near singular; residual={result.pos_err*1000:.3f}mm; '
                        'prepare a bent initial pose, do not increase speed/limits')
            return
        proposed = np.rad2deg(result.q)
        if not np.all(np.isfinite(proposed)):
            self.reject('IK returned nonfinite joints')
            return
        cap = min(self.cfg["max_joint_step_deg"], self.cfg["joint_speed_deg_s"]*CTRL_DT)
        dq = proposed-q0
        proposed = q0+dq*min(1.,cap/max(np.max(np.abs(dq)),1e-12))
        qrad = np.deg2rad(proposed)
        if np.array_equal(np.rint(proposed*1000),np.rint(q0*1000)):
            self.reject('IK BLOCKED: no quantized joint progress')
            return
        if np.any(qrad < self.q_min) or np.any(qrad > self.q_max):
            raise RealPicoFault("IK candidate violates joint limits")
        candidate_fk = kin.fk_at(qrad)
        original_tcp = self.tcp_pos
        for _ in range(4):
            candidate_tcp = self.base_position + r @ (
                candidate_fk.pos+quat_matrix(candidate_fk.quat)@self.tcp_offset)
            travel = np.linalg.norm(candidate_tcp-original_tcp)
            if travel <= step+1e-7:
                break
            proposed = q0+(proposed-q0)*step/travel*.999
            qrad = np.deg2rad(proposed)
            candidate_fk = kin.fk_at(qrad)
        else:
            self.reject('IK candidate cannot satisfy Cartesian step bound')
            return
        angle_error = 2.*np.arccos(np.clip(abs(candidate_fk.quat@pose.quat),0.,1.))
        if angle_error > .01:
            self.reject('IK candidate violates locked wrist orientation')
            return
        actual_delta=candidate_tcp-original_tcp
        if np.linalg.norm(actual_delta)>1e-8:
            alignment=float(actual_delta@d/(np.linalg.norm(actual_delta)*np.linalg.norm(d)))
            if alignment<.98:
                self.reject('IK BLOCKED: bounded joint step would deviate from requested direction')
                return
        measured_fk = kin.fk_at(np.deg2rad(st.joint))
        measured_tcp = self.base_position + r @ (
            measured_fk.pos+quat_matrix(measured_fk.quat)@self.tcp_offset)
        if np.linalg.norm(candidate_tcp-measured_tcp) > self.cfg["max_tcp_following_m"]:
            raise RealPicoFault("TCP following gap exceeded")
        initial_error = np.linalg.norm(pose.pos-self.shadow.fk_cmd().pos)
        if np.linalg.norm(pose.pos-candidate_fk.pos) > initial_error+1e-6:
            self.reject('IK candidate increases target error')
            return
        self.shadow.set_q(np.deg2rad(proposed))
        self.check_workspace(self.tcp_pos)
        # Recheck freshness immediately before the only real motion boundary.
        self.checked_state()
        self.sent = True
        self.holding = False
        with self.transport.teleop():
            ret = self.native.Movej_CANFD(proposed)
        if ret:
            raise RealPicoFault(f"Joint passthrough rejected: {ret}")
        self.last_send = time.monotonic()
        self.frames_sent+=1
        self.last_ret = 0
        self.motion_reason='Frame sent; physical movement requires measured feedback (not acknowledgement)'
        self.position_error = float(np.linalg.norm(target-self.pos))


class RealPicoServo(PicoEeServo):
    def __init__(self, native, cfg):
        super().__init__(native, speed=cfg["linear_speed_m_s"], scale=cfg["scale"], wrist="position",
                         yaw=180. if cfg.get('robot_forward','+X')=='-X' else 0.,
                         adapter_factory=lambda a: RealPicoAdapter(a,cfg))
        self.last_sample = None
        self.last_tick = time.monotonic()
        self._watch_stop = threading.Event()
        self._watcher = None
        self._released_since = None
        self._grip_ready = False
        self._manual_inhibit = False
        self._last_stick = False

    def start(self):
        if self.enabled:
            return
        self._released_since=None
        self._grip_ready=False
        self.last_sample=None
        super().start()
        self.last_tick = time.monotonic()
        self._watch_stop.clear()
        if self._watcher is None or not self._watcher.is_alive():
            self._watcher = threading.Thread(target=self._watch, name="pico-real-watchdog", daemon=True)
            self._watcher.start()
        print(f"[pico-real] activation={self.arm.cfg.get('activation','grip')}. "
              "Set forward; release Grip/trigger for 0.3s, then hold Grip. No home motion.", flush=True)

    def bind_shell(self, shell):
        shell.register("pico", self.handle_cli,
                       usage="pico on|off|status|arm|disarm",
                       help="REAL: starts disarmed; arm + held Grip enables bounded translation")

    def draw_overlay(self, viewer):
        if hasattr(viewer, "set_texts"):
            with self._status_lock:
                status = self._status
            viewer.set_texts((None, None, "REAL RM65 | measured-joint mirror | position only",
                  status + "\nRelease Grip/trigger -> hold Grip to follow; CLI pico disarm disables\n"
                  f"Forward={self.arm.cfg.get('robot_forward','+X')}; controller TCP={self.arm.hardware['tool']['tool_name']}; "
                  f"gripper enabled={self.arm.cfg['gripper_enabled']}\nGripper: {self.arm.gripper_label}\n"
                  f"{'NEAR SINGULAR: prepare a bent starting pose; no auto-reposition' if self.arm.posture['near_singular'] else 'Posture: regular'}"))

    def _watch(self):
        while not self._watch_stop.wait(.03):
            if self.arm.armed and time.monotonic()-self.last_tick > .3:
                self.arm.fault = "PICO servo stalled"
                self.arm.transport.abort()
                _thread.interrupt_main()
                return

    def prepare_sample(self, sample):
        self.last_tick = time.monotonic()
        self.last_sample = sample
        if self.arm.fault:
            raise RealPicoFault(self.arm.fault)
        if self.arm.transport.violation:
            raise RealPicoFault(self.arm.transport.violation)
        stick=bool(sample.axis_click)
        if stick and not self._last_stick and sample.grip<=.15:
            self._manual_inhibit=False
        self._last_stick=stick
        if not fresh(sample) or sample.menu:
            self._released_since=None
            self._grip_ready=False
        if self.arm.armed:
            self.arm.checked_state()
            if self.arm.native._busy:
                raise RealPicoFault("Competing native motion; disarm before CLI motion")
            if not fresh(sample) or sample.menu or "tracking jump" in self.control.reason:
                self.arm.disarm("XR lost/menu/jumped; release Grip and set forward again")
                self.control.direction_basis = None
                self.control.active = False
                self.control.need_release = True
                self._grip_ready=False
            elif sample.grip<=.15 and self.arm.cfg.get('activation','grip')=='grip':
                self.arm.disarm('Grip released; hold requested; release trigger before next Grip')
                self._released_since=None
                self._grip_ready=False
        elif self.arm.cfg.get('activation','grip')=='grip' and not self._manual_inhibit:
            has_heading=self.control.direction_basis is not None or sample.headset_quat_wxyz is not None
            if fresh(sample) and not sample.menu and has_heading and sample.grip<=.15 and sample.trigger<=.1:
                if self._released_since is None:
                    self._released_since=time.monotonic()
                self._grip_ready=time.monotonic()-self._released_since>=.3
                self.arm.reason='READY: press Grip to follow' if self._grip_ready else 'WAIT: release Grip/trigger 0.3s'
            elif fresh(sample) and not sample.menu and sample.grip>.25 and self._grip_ready and sample.trigger<=.1:
                self._grip_ready=False
                self._released_since=None
                try:
                    self.arm.arm()
                    self.control.active=False
                    self.control.need_release=False
                except RealPicoFault as exc:
                    self.arm.reason='BLOCKED: '+str(exc)
            else:
                self._released_since=None
                if sample.grip<=.15 or sample.grip>.25:
                    self._grip_ready=False
                if not has_heading:
                    self.arm.reason='WAIT: set forward with left stick click; no motion enabled'
        self.arm.gripper_allowed = bool(self.arm.armed and fresh(sample) and sample.grip>.25 and self.control.active)
        return sample if self.arm.armed else replace(sample, grip=0.)

    def status_suffix(self):
        hand,actual=self.control.motion_summary()
        return ("; REAL " + self.arm.reason + f'; frames_sent={self.arm.frames_sent}; '
                + self.arm.motion_reason + f'; hand_delta={np.linalg.norm(hand)*1000:.2f}mm'
                + f'; measured_delta={np.linalg.norm(actual)*1000:.2f}mm')

    def on_error(self,exc):
        import traceback
        from datetime import datetime,timezone
        from pathlib import Path
        report={'created_utc':datetime.now(timezone.utc).isoformat(),'error':str(exc),
                'traceback':traceback.format_exc(),'last_status':self._status,
                'real_reason':self.arm.reason,'motion_reason':self.arm.motion_reason,
                'frames_sent':self.arm.frames_sent,'posture':self.arm.posture}
        state,stamp=self.arm.state.snapshot()
        report['feedback_age_s']=time.monotonic()-stamp
        report['measured_joint_deg']=state.joint.tolist() if state is not None else None
        report['command_joint_deg']=self.arm.shadow.q_cmd_deg.tolist()
        if self.last_sample is not None:
            report['input']={'grip':self.last_sample.grip,'trigger':self.last_sample.trigger,
                             'valid':self.last_sample.valid,'stamp':self.last_sample.stamp}
        path=Path(__file__).resolve().parents[1]/'logs/real/last_error.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('[pico-real] Saved failure details:',path,flush=True)

    def record_metadata(self):
        return {'backend':'real-vdo-physical-angles', 'activation':self.arm.cfg.get('activation','grip'),
                'robot_forward':self.arm.cfg.get('robot_forward','+X'),
                'controller_tool':self.arm.hardware['tool'],
                'fk_validation':self.arm.fk_validation}

    def handle_cli(self, tokens):
        action = tokens[0] if tokens else "status"
        if action == "arm":
            s = self.last_sample
            if not self.enabled or s is None or not fresh(s) or s.grip>.15 or s.trigger>.1:
                raise RealPicoFault("Need live PICO, released Grip and trigger before pico arm")
            if self.control.direction_basis is None and s.headset_quat_wxyz is None:
                raise RealPicoFault("Set forward with the stick click or enable valid Head Tracking")
            self.arm.arm()
            self.control.active = False
            self.control.need_release = False
            print("[pico-real] ARMED; hold Grip for bounded translation", flush=True)
            return 0
        if action == "disarm":
            self.arm.disarm()
            self._manual_inhibit=True
            self._grip_ready=False
            self._released_since=None
            self.arm.reason='DISABLED by CLI; release Grip and set forward (stick click) to unlock'
            self.control.active = False
            self.control.need_release = True
            print("[pico-real] DISARMED", flush=True)
            return 0
        return super().handle_cli(tokens)

    def stop(self):
        self._watch_stop.set()
        self._grip_ready=False
        self._released_since=None
        try:
            super().stop()
        finally:
            self.arm.disarm()
            if self.arm.fault:
                self.error=self.arm.fault
