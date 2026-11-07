"""Latest-target bridge: one socket owner, no motion RPC or replay FIFO."""
from __future__ import annotations
import json
import select
import socket
import threading
import time
import numpy as np


class LatestTransport:
    def __init__(self, real, cfg):
        self.real, self.cfg = real, cfg
        self.original = real.link
        self._sock = self.original._sock
        self.last_arm_state_received = self.original.last_arm_state_received
        self.last_movej_rpc = None
        self.last_gripper_rpc = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.done = threading.Event()
        self.target = None
        self.epoch = 0
        self.stop_pending = False
        self.stop_deadline = None
        self.stop_confirmed = True
        self.sent = False
        self.frames = 0
        self.last_goal = None
        self.fault = None
        self.thread = None
        self.closed = False
        self.initial_joint = real.arm_state.get().joint.copy()
        self.velocity = np.zeros(6)
        self.previous = None
        self.progress_time = time.monotonic()
        self.progress_joint = self.initial_joint.copy()
        self.raw_state = None
        self.buf = bytearray(getattr(self.original, '_buf', b''))
        self.trace = None
        self.trace_path = None
        self.trace_lock = threading.Lock()
        # Every native caller now uses this facade; only the worker reads TCP.
        real.link = real.plant.link = self

    def rpc(self, payload, *, timeout_s=None):
        if payload.get('command') == 'get_current_arm_state':
            with self.lock:
                raw = self.raw_state
            if raw is None:
                from real.json_codec import encode_arm_state
                # Native RealPlant will perform its normal wire->MDH conversion.
                from real.json_codec import apply_joint_axis_sign
                from copy import copy
                state = copy(self.real.arm_state.get())
                state.joint = apply_joint_axis_sign(state.joint)
                raw = encode_arm_state(state)
            return raw
        raise RuntimeError('Realtime transport owns robot; native motion/RPC is disabled')

    def state(self):
        state = self.real.arm_state.get()
        if time.monotonic()-self.last_arm_state_received > self.cfg['state_timeout_s']:
            raise RuntimeError('Realtime feedback stale')
        if state.arm_err or state.sys_err or not np.all(np.isfinite(state.joint)):
            raise RuntimeError('Realtime controller error')
        return state

    def submit(self, q, active):
        now = time.monotonic()
        if active and self.fault:
            raise RuntimeError(self.fault)
        if active:
            q = np.asarray(q, float)
            if q.shape != (6,) or not np.all(np.isfinite(q)):
                raise ValueError('Expected six finite joints')
        with self.lock:
            if active and (self.stop_pending or self.stop_deadline is not None):
                return  # A quick regrip cannot bypass an unconfirmed stop.
            if active:
                self.target = (q.copy(), now, self.epoch)
            else:
                self.target = None
                self.epoch += 1
                if self.sent and not self.stop_pending and self.stop_deadline is None:
                    self.stop_pending = True
                    self.stop_confirmed = False
        self.wake.set()

    def cancel_input(self):
        self.submit(None, False)

    def submit_gripper(self, position):
        raise RuntimeError('Realtime v1 requires --no-gripper; gripper is not validated')

    def replay_status(self):
        with self.lock:
            target = self.target
        return {'queued_frames': int(target is not None), 'replay_lag_s': 0.,
                'target_age_s': 0. if target is None else time.monotonic()-target[1],
                'source_frames': None, 'replayed_seq': None}

    def record(self, event, **fields):
        with self.trace_lock:
            if self.trace:
                self.trace.write(json.dumps({'type':event,'monotonic_s':time.monotonic(),**fields})+'\n')
                self.trace.flush()

    def start(self):
        from pathlib import Path
        from datetime import datetime
        self.trace_path = Path(__file__).resolve().parents[1]/'logs/real'/('latest_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
        self.trace_path.parent.mkdir(parents=True,exist_ok=True)
        self.trace = self.trace_path.open('x',encoding='utf-8')
        self._sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self._sock.setblocking(False)
        self.thread = threading.Thread(target=self._loop,name='rm65-latest-io',daemon=True)
        self.thread.start()

    def _write(self, payload):
        # A partial TCP message cannot be discarded and followed by a new one.
        data = memoryview((json.dumps(payload,separators=(',',':'))+'\r\n').encode())
        deadline = time.monotonic()+.01
        while data:
            left = deadline-time.monotonic()
            if left <= 0 or not select.select([], [self._sock], [], max(0.,left))[1]:
                raise TimeoutError('Realtime send exceeded 10ms; connection must close')
            try:
                n = self._sock.send(data)
            except BlockingIOError:
                continue
            if n == 0:
                raise ConnectionError('Realtime socket closed')
            data = data[n:]

    def _read(self):
        from real.json_codec import decode_arm_state, apply_joint_axis_sign
        while select.select([self._sock],[],[],0)[0]:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError('Controller disconnected')
            self.buf.extend(chunk)
            if len(self.buf)>1048576:
                raise RuntimeError('Controller message buffer exceeded limit')
            # Bound each receive pass so a busy peer cannot starve stop/output.
            break
        while b'\n' in self.buf:
            line,_,rest = self.buf.partition(b'\n')
            self.buf = bytearray(rest)
            if not line.strip():
                continue
            msg = json.loads(line)
            if msg.get('state') == 'current_arm_state' or 'arm_state' in msg:
                state = decode_arm_state(msg)
                state.joint = apply_joint_axis_sign(state.joint)
                if state.joint.shape != (6,) or not np.all(np.isfinite(state.joint)):
                    raise RuntimeError('Invalid feedback joints')
                self.real.arm_state.put(state)
                with self.lock:
                    self.raw_state = msg
                    self.last_arm_state_received = time.monotonic()
                self.record('feedback',joint_deg=state.joint.tolist(),arm_err=state.arm_err,sys_err=state.sys_err)
            elif msg.get('command') == 'set_arm_stop' and self.stop_deadline is not None:
                if msg.get('arm_stop') is not True:
                    raise RuntimeError('Controller rejected stop')
                with self.lock:
                    self.stop_deadline = None
                    self.stop_confirmed = True
                    self.sent = False
                self.last_goal = None
                self.previous = None
                self.velocity[:] = 0.
                self.record('stop_ack',reply=msg)
            elif (msg.get('command') == 'movej_canfd' and
                  (msg.get('state') is False or msg.get('receive_state') is False or msg.get('status') in (False, 'false', 'error'))):
                raise RuntimeError('Controller rejected CANFD frame')
            elif msg.get('arm_err',0) or msg.get('sys_err',0):
                raise RuntimeError('Controller reported motion error')
            else:
                self.record('reply',reply=msg)

    def _tick(self, now):
        from real.json_codec import apply_joint_axis_sign, encode_joint_deg
        with self.lock:
            request_stop = self.stop_pending
            if request_stop:
                self.target = None
                self.stop_pending = False
                self.stop_deadline = now+.5
            target = self.target
            stopping = self.stop_deadline is not None
        if request_stop:
            self._write({'command':'set_arm_stop'})
            self.record('stop_sent')
        if stopping:
            if now > self.stop_deadline:
                raise TimeoutError('Stop unconfirmed after 500ms')
            return
        if target is None:
            return
        q, stamp, epoch = target
        if now-stamp > self.cfg['max_stream_gap_s']:
            self.cancel_input()
            return
        state = self.state()
        if self.real._busy:
            raise RuntimeError('Other native motion active')
        reference = state.joint.copy() if self.last_goal is None else self.last_goal.copy()
        if np.max(np.abs(reference-state.joint))>self.cfg['max_following_error_deg']:
            raise RuntimeError('Realtime following error exceeded limit')
        if np.max(np.abs(state.joint-self.progress_joint))>=.01:
            self.progress_joint=state.joint.copy()
            self.progress_time=now
        elif np.max(np.abs(reference-state.joint))>=.05 and now-self.progress_time>1.:
            raise RuntimeError('No measured joint response for 1s')
        elif np.max(np.abs(reference-state.joint))<.05:
            self.progress_time=now
        dt=.02 if self.previous is None else min(.02,max(.001,now-self.previous))
        self.previous=now
        delta=q-reference
        desired=np.clip(delta/dt,-30.,30.)
        velocity=np.clip(desired,self.velocity-120.*dt,self.velocity+120.*dt)
        step=velocity*dt
        # Do not overshoot the newest target when decelerating or reversing.
        step=np.where(step*delta>0,np.sign(delta)*np.minimum(np.abs(step),np.abs(delta)),0.)
        goal=reference+step
        lead=np.max(np.abs(goal-state.joint))
        if lead>self.cfg['max_following_error_deg']:
            return
        if np.max(np.abs(goal-reference))<.001:
            return
        with self.lock:
            if self.target is None or epoch!=self.epoch or self.stop_pending:
                return
            self.sent=True
        self._write({'command':'movej_canfd','joint':encode_joint_deg(apply_joint_axis_sign(goal)),'follow':False})
        self.velocity=step/dt
        self.last_goal=goal
        self.frames+=1
        self.record('sent',joint_deg=goal.tolist(),real_joint_deg=state.joint.tolist(),target_age_s=now-stamp,confirmation='local send only')

    def _loop(self):
        next_send=next_query=time.monotonic()
        try:
            while not self.done.is_set():
                self._read()
                now=time.monotonic()
                if now>=next_query:
                    self._write({'command':'get_current_arm_state'})
                    next_query=now+.05
                if now>=next_send or self.stop_pending:
                    self._tick(now)
                    next_send=now+.02
                self.wake.wait(.002)
                self.wake.clear()
        except Exception as exc:
            self.fault=str(exc)
            if self.sent and not isinstance(exc, (TimeoutError, ConnectionError, OSError)):
                try:
                    self._write({'command':'set_arm_stop'})
                    self.stop_confirmed=False
                except Exception:
                    pass
            self.record('fault',error=self.fault)
            # Best effort only. A broken/partially written stream cannot confirm stop.
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        finally:
            self.done.set()

    def stop(self):
        self.cancel_input()
        deadline=time.monotonic()+.65
        while self.thread and self.thread.is_alive() and self.sent and time.monotonic()<deadline:
            self.wake.set()
            self.done.wait(.005)
        self.done.set()
        self.wake.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(.2)
        if self.thread and self.thread.is_alive():
            raise RuntimeError('Realtime worker did not stop')
        with self.trace_lock:
            if self.trace:
                self.trace.close()
                self.trace=None
        if self.sent and not self.stop_confirmed:
            raise RuntimeError('Realtime stop unconfirmed; check physical robot')

    def close(self):
        try:
            self.stop()
        finally:
            self._sock.close()
            self.closed=True


from sim_real_bridge import SimRealBridgeServo
from sim_control import fresh, CTRL_DT
import mujoco


class LatestBridgeServo(SimRealBridgeServo):
    def __init__(self, real, cfg):
        if cfg['gripper_enabled']:
            raise ValueError('Realtime bridge v1: use -NoGripper for arm-only validation')
        super().__init__(real, cfg)
        self.follower = LatestTransport(real, cfg)
        self.arm.follower = self.follower

    def prepare_sample(self, sample):
        if self.follower.fault:
            raise RuntimeError(self.follower.fault)
        return super().prepare_sample(sample)

    def after_control_tick(self, sample):
        active = bool(self.control.active and self.arm.input_permitted)
        if active and np.linalg.norm(self.control.operator_delta)>=.001:
            self.arm.motion_started=True
        with self.display_arm.lock.data_lock():
            if not active and self.follower.stop_confirmed and not self.follower.sent:
                state=self.follower.state()
                self.display_arm.pin_meas_q(self.real.api_joint_deg_to_mj(state.joint))
                self.display_arm.data.qvel[self.arm_dofs]=0.
            self.display_arm.set_q(np.deg2rad(self.display_arm.q_cmd_deg))
            if active and self.arm.motion_started:
                for _ in range(max(1,int(round(CTRL_DT/self.arm.model.opt.timestep)))):
                    mujoco.mj_step(self.arm.model,self.arm.data)
            mujoco.mj_forward(self.arm.model,self.arm.data)
            # Same model command is used in connected and disconnected modes.
            joints=self.display_arm.q_cmd_deg.copy()
        self.follower.submit(joints,active and self.arm.motion_started)

    def start(self):
        if self.enabled:
            return
        from vdo_pico import PicoEeServo
        self.follower.start()
        try:
            PicoEeServo.start(self)
        except Exception:
            self.follower.stop()
            raise
        print('[latest] 20ms latest-target send; 50ms state query; no FIFO; local send is not execution confirmation',flush=True)

    def status_suffix(self):
        state=self.follower.state()
        gap=0. if self.follower.last_goal is None else float(np.max(np.abs(self.follower.last_goal-state.joint)))
        movement=float(np.max(np.abs(state.joint-self.follower.initial_joint)))
        return f'; latest_sent={self.follower.frames} (local send only); REAL_joint_move={movement:.3f}deg; command_real_gap={gap:.3f}deg; stop_ack={self.follower.stop_confirmed}; GRIPPER DISABLED'

    def record_metadata(self):
        return {'backend':'latest-target-single-socket-owner','robot_forward':self.cfg['robot_forward'],'period_s':.02,'feedback_period_s':.05,'gripper_mode':'disabled'}
