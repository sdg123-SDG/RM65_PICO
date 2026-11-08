"""Latest-target bridge: one socket owner, no motion RPC or replay FIFO."""
from __future__ import annotations
import json
import queue
import re
import select
import socket
import threading
import time
import numpy as np

# The controller prints these when its parser sees the next JSON too early.
# They are not motion replies and must not pause or disconnect the session.
_CONTROLLER_PARSER_NOTE = re.compile(r"^\d+\s*,\s*.*\bexpected near\b", re.IGNORECASE)


def controller_parser_note(raw):
    return _CONTROLLER_PARSER_NOTE.match(str(raw).strip()) is not None


class TraceWriter:
    """Bounded diagnostics off the I/O/control threads; failure is observable."""
    def __init__(self, path):
        self.output=path.open('x',encoding='utf-8')
        self.queue=queue.Queue(maxsize=8192)
        self.done=threading.Event()
        self.error=None
        self.thread=threading.Thread(target=self._run,name='rm65-trace',daemon=True)
        self.thread.start()

    def enqueue(self, item):
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.error='Trace queue full; disk is too slow'

    def _run(self):
        last_flush=time.perf_counter()
        try:
            while not self.done.is_set() or not self.queue.empty():
                try:
                    item=self.queue.get(timeout=.05)
                except queue.Empty:
                    self.output.flush()
                    last_flush=time.perf_counter()
                    continue
                self.output.write(json.dumps(item)+'\n')
                # Batch flushing; never force a flush for every motion frame.
                if time.perf_counter()-last_flush>=.1:
                    self.output.flush()
                    last_flush=time.perf_counter()
        except Exception as exc:
            self.error=f'Trace writer failed: {exc}'
        finally:
            self.output.close()

    def close(self):
        self.done.set()
        self.thread.join(2.)
        if self.thread.is_alive():
            raise RuntimeError('Trace writer did not finish')
        if self.error:
            raise RuntimeError(self.error)


# This controller does not acknowledge non-blocking gripper commands, so waiting
# a full second per command made the gripper lag and swallowed quick trigger
# changes. Wait only briefly, keep gripper sends spaced (the controller has
# rejected adjacent mixed commands), and resend the last target once after the
# trigger settles in case the controller dropped it.
GRIPPER_ACK_WAIT_S = .15
GRIPPER_MIN_GAP_S = .15
GRIPPER_RESEND_AFTER_S = .5


class LatestTransport:
    def __init__(self, real, cfg):
        self.real, self.cfg = real, cfg
        self.original = real.link
        self._sock = self.original._sock
        self.last_arm_state_received = self.original.last_arm_state_received
        self.last_movej_rpc = None
        self.last_gripper_rpc = None
        self.gripper_target = None
        self.gripper_inflight = None
        self.gripper_last_send = 0.
        self.gripper_resend = None
        self.gripper_error = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.done = threading.Event()
        self.target = None
        self.epoch = 0
        self.stop_pending = False
        self.stop_deadline = None
        self.stop_confirmed = True
        self.hold_target = None
        self.hold_accepted = False
        self.hold_samples = []
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
        self.exit_confirmed = False
        self.pending_query = None
        self.diagnostics_enabled = True
        self.protocol_pause = None
        self.protocol_errors = []
        self.rearm_required = False
        self.operator_paused = False
        self._feedback = real.arm_state.get()
        real.halt_for_exit = self.halt_for_exit
        real.fetch_arm_state = self.cached_state
        real.Move_Stop_Cmd = self.operator_stop
        real.Move_Pause_Cmd = self.operator_stop
        real.Move_Continue_Cmd = self.operator_resume
        # Every native caller now uses this facade; only the worker reads TCP.
        real.link = real.plant.link = self

    def cached_state(self):
        # The native sampler must never republish an old RPC snapshot.
        from copy import copy
        with self.lock:
            state=copy(self._feedback)
            state.joint=state.joint.copy()
        return state

    def _publish_feedback(self,state):
        with self.lock:
            self._feedback=state
            self.last_arm_state_received=time.monotonic()
            self.real.arm_state.put(state)

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
        with self.lock:
            state=self._feedback
            stamp=self.last_arm_state_received
        age=time.monotonic()-stamp
        if age > self.cfg['state_timeout_s']:
            self.record('feedback_stale',age_s=age,last_feedback_monotonic_s=self.last_arm_state_received,io_alive=bool(self.thread and self.thread.is_alive()),sender_fault=self.fault,stop_ack=self.stop_confirmed)
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
            if active and (self.operator_paused or self.rearm_required or self.protocol_pause or self.stop_pending or self.stop_deadline is not None):
                return  # A quick regrip cannot bypass an unconfirmed stop.
            if active:
                self.target = (q.copy(), now, self.epoch)
            else:
                self.target = None
                self.epoch += 1
                if self.sent:
                    self.rearm_required=True
                if self.sent and not self.stop_pending and self.stop_deadline is None:
                    self.stop_pending = True
                    self.stop_confirmed = False
        self.wake.set()

    def cancel_input(self):
        self.submit(None, False)

    def operator_stop(self, block=True):
        self.operator_paused=True
        self.cancel_input()
        self.record('operator_pause',confirmation='requested; see hold_confirmed')
        return 0

    def operator_resume(self, block=True):
        self.operator_paused=False
        self.rearm_required=True
        self.record('operator_resume',requires='physical release after hold; no automatic movement')
        return 0

    def acknowledge_release(self, grip, valid):
        if not valid or grip>.1:
            return False
        self.state()
        with self.lock:
            if not self.stop_confirmed or self.sent or self.stop_pending or self.stop_deadline is not None:
                return False
            self.protocol_pause=None
            self.rearm_required=False
        self.record('rearm_released')
        return True

    def submit_gripper_position(self, position):
        if not self.cfg.get('gripper_enabled', False):
            raise RuntimeError('Gripper output is disabled')
        # EG2 opening command: 1=fully closed, 1000=fully open. Nonblocking lets
        # analog trigger updates replace stale unsent targets while moving.
        position = max(1, min(1000, int(position)))
        payload = {'command': 'set_gripper_position', 'position': position, 'block': False}
        with self.lock:
            self.gripper_error = None
            self.gripper_target = (payload, time.monotonic())
        self.wake.set()

    def submit_gripper_pick_on(self, speed, force):
        if not self.cfg.get('gripper_enabled', False):
            raise RuntimeError('Gripper output is disabled')
        # Force-controlled continuous grasp: keeps squeezing at `force` after
        # contact instead of stopping like set_gripper_position does.
        payload = {'command': 'set_gripper_pick_on', 'speed': max(1, min(1000, int(speed))),
                   'force': max(1, min(1000, int(force))), 'block': False}
        with self.lock:
            self.gripper_error = None
            self.gripper_target = (payload, time.monotonic())
        self.wake.set()

    def cancel_gripper(self):
        with self.lock:
            self.gripper_target = None
            self.gripper_resend = None

    def replay_status(self):
        with self.lock:
            target = self.target
        return {'queued_frames': int(target is not None), 'replay_lag_s': 0.,
                'target_age_s': 0. if target is None else time.monotonic()-target[1],
                'source_frames': None, 'replayed_seq': None}

    def record(self, event, **fields):
        with self.trace_lock:
            writer=self.trace
        if writer is not None:
            writer.enqueue({'type':event,'monotonic_s':time.monotonic(),
                            'wall_time_ns':time.time_ns(),'perf_counter_s':time.perf_counter(),**fields})

    def start(self):
        if self.thread is not None or self.done.is_set() or self.closed:
            raise RuntimeError('Realtime transport cannot restart; open a new session')
        from pathlib import Path
        from datetime import datetime
        trace_dir=Path(self.cfg.get('trace_dir',Path(__file__).resolve().parents[1]/'logs/real'))
        self.trace_path = trace_dir/('latest_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
        self.trace_path.parent.mkdir(parents=True,exist_ok=True)
        self.trace = TraceWriter(self.trace_path)
        self._sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self._sock.setblocking(False)
        self.thread = threading.Thread(target=self._loop,name='rm65-latest-io',daemon=True)
        self.thread.start()

    def _write(self, payload):
        # A partial TCP message cannot be discarded and followed by a new one.
        self.record('tx_attempt',payload=payload)
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
        self.record('tx_complete',payload=payload)
        if payload['command'].startswith('get_'):
            self.pending_query=(payload['command'],time.monotonic())

    def _handle_gripper_ack(self, msg):
        with self.lock:
            pending = self.gripper_inflight
            self.gripper_inflight = None
            if pending is not None:
                self.gripper_error = None if msg.get('state') is True else 'controller rejected gripper command'
                if msg.get('state') is True and self.gripper_resend == pending[0]:
                    self.gripper_resend = None  # confirmed; no safety resend needed
        self.record('gripper_ack', reply=msg, requested=pending)
        if pending is not None and msg.get('state') is not True:
            print('[gripper] command rejected; a later press can send a new command', flush=True)

    def _read(self):
        from real.json_codec import decode_arm_state, apply_joint_axis_sign
        while select.select([self._sock],[],[],0)[0]:
            chunk = self._sock.recv(16384)
            if not chunk:
                raise ConnectionError('Controller disconnected')
            self.buf.extend(chunk)
            if len(self.buf)>1048576:
                raise RuntimeError('Controller message buffer exceeded limit')
            # Bound each receive pass so a busy peer cannot starve stop/output.
            break
        parsed=0
        parse_deadline=time.perf_counter()+.003
        while b'\n' in self.buf and parsed<64 and time.perf_counter()<parse_deadline:
            parsed+=1
            line,_,rest = self.buf.partition(b'\n')
            self.buf = bytearray(rest)
            if not line.strip():
                continue
            try:
                decoded=line.decode('utf-8').strip()
                if not decoded.startswith('{'):
                    raise json.JSONDecodeError('Expected controller object',decoded,0)
                msg,end=json.JSONDecoder().raw_decode(decoded)
                remaining=decoded[end:].strip()
                if remaining:
                    self.buf[:0]=remaining.encode('utf-8')+b'\n'
            except json.JSONDecodeError:
                raw=line.decode('utf-8',errors='replace')
                # Some controller versions interleave a complete gripper ACK
                # into a current_arm_state or joint_state response. Recover only
                # those observed shapes, then reassemble the interrupted line.
                recovered = False
                state_prefix = (('current_arm_state' in raw and 'arm_state' in raw)
                                or 'joint_state' in raw)
                # The ACK can arrive after the short gripper wait has expired
                # (seen ~0.3-0.9 s after set_gripper_pick_on), so recover it
                # even when no gripper command is still marked in flight.
                if state_prefix:
                    for i, char in enumerate(raw):
                        if char != '{' or i == 0:
                            continue
                        try:
                            ack, end = json.JSONDecoder().raw_decode(raw, i)
                        except json.JSONDecodeError:
                            continue
                        if (isinstance(ack, dict) and ack.get('command') == 'set_gripper'
                                and isinstance(ack.get('state'), bool) and not raw[end:].strip()
                                and 'joint' in raw[:i]):
                            self.buf[:0] = raw[:i].encode('utf-8')
                            self._handle_gripper_ack(ack)
                            self.record('interleaved_gripper_ack_recovered')
                            recovered = True
                            break
                if recovered:
                    continue
                if controller_parser_note(raw):
                    self.record('controller_parser_note', raw=raw.strip())
                    continue
                self.record('invalid_rx',raw=raw)
                now=time.monotonic()
                self.protocol_errors=[t for t in self.protocol_errors if now-t<1.]+[now]
                if len(self.protocol_errors)>=3:
                    raise RuntimeError('Repeated controller protocol errors; hold cannot be trusted')
                if self.pending_query is not None:
                    self.record('query_protocol_error',query=self.pending_query[0])
                    self.pending_query=None
                self.protocol_pause=raw.strip()
                self.cancel_input()
                self.record('protocol_pause',reason=self.protocol_pause)
                continue
            if not isinstance(msg,dict):
                raise RuntimeError('Controller response is not an object')
            if self.pending_query is not None:
                command,_=self.pending_query
                if msg.get('state') in (command,command.removeprefix('get_')) or msg.get('command') in (command,command.removeprefix('get_')):
                    self.pending_query=None
            if msg.get('command') in ('set_gripper', 'set_gripper_position', 'set_gripper_pick_on'):
                self._handle_gripper_ack(msg)
            elif msg.get('state') == 'current_arm_state' or 'arm_state' in msg:
                state = decode_arm_state(msg)
                state.joint = apply_joint_axis_sign(state.joint)
                if state.joint.shape != (6,) or not np.all(np.isfinite(state.joint)):
                    raise RuntimeError('Invalid feedback joints')
                self._publish_feedback(state)
                with self.lock:
                    self.raw_state = msg
                    self.last_arm_state_received = time.monotonic()
                self.record('feedback',joint_deg=state.joint.tolist(),arm_err=state.arm_err,sys_err=state.sys_err)
                if self.hold_accepted and self.hold_target is not None:
                    now=time.monotonic()
                    if np.max(np.abs(state.joint-self.hold_target))<=.05:
                        self.hold_samples.append((now,state.joint.copy()))
                        self.hold_samples=self.hold_samples[-4:]
                        stable=len(self.hold_samples)>=3 and now-self.hold_samples[0][0]>=.09
                        stable=stable and np.max(np.ptp(np.array([q for _,q in self.hold_samples]),axis=0))<=.01
                        if stable:
                            with self.lock:
                                self.stop_deadline=None
                                self.stop_confirmed=True
                                self.sent=False
                            self.hold_accepted=False
                            self.hold_target=None
                            self.last_goal=None
                            self.previous=None
                            self.velocity[:]=0.
                            self.record('hold_confirmed',joint_deg=state.joint.tolist())
                    else:
                        self.hold_samples=[]
            elif str(msg.get('state','')).strip() == 'joint_state':
                # Gen-2 CANFD replies are measured joints, not an execution ACK.
                from copy import copy
                joints=np.asarray(msg.get('joint'),dtype=float)
                if joints.shape!=(6,) or not np.all(np.isfinite(joints)):
                    raise RuntimeError('Invalid CANFD joint feedback')
                if msg.get('arm_err',0):
                    raise RuntimeError('Controller CANFD arm error')
                state=self.cached_state()
                state.joint=apply_joint_axis_sign(joints/1000.)
                self._publish_feedback(state)
                # Keep the native cache consistent without inventing a fresh pose.
                self.raw_state={'state':'current_arm_state','arm_state':{
                    'joint':joints.astype(int).tolist(),'pose':None,
                    'arm_err':int(msg.get('arm_err',0)),'sys_err':state.sys_err}}
                from real.json_codec import encode_pose
                self.raw_state['arm_state']['pose']=encode_pose(state.pose)
                self.record('joint_feedback',joint_deg=state.joint.tolist(),arm_err=int(msg.get('arm_err',0)),sys_err_fresh=False)
            elif self.stop_deadline is not None and (msg.get('command') == 'movej' or msg.get('state') == 'current_trajectory_state'):
                accepted=(msg.get('receive_state') is True or msg.get('movej') is True or msg.get('state') is True or (msg.get('state')=='current_trajectory_state' and msg.get('trajectory_state') is True))
                rejected=(msg.get('trajectory_state') is False or msg.get('receive_state') is False or msg.get('movej') is False or msg.get('state') is False or msg.get('status') in (False,'false','error'))
                if rejected or not accepted:
                    raise RuntimeError('Controller rejected hold')
                if not self.hold_accepted:
                    self.stop_deadline=time.monotonic()+3.
                    self.hold_samples=[]
                self.hold_accepted=True
                self.record('hold_ack',reply=msg)
            elif msg.get('state') in ('joint_en_state','joint_err_flag'):
                enabled=msg.get('state')=='joint_en_state'
                values=msg.get('en_state' if enabled else 'err_flag')
                if not isinstance(values,list) or len(values)!=6:
                    raise RuntimeError('Malformed joint diagnostic feedback')
                abnormal=any(v!=1 for v in values) if enabled else any(v!=0 for v in values)
                if abnormal:
                    raise RuntimeError('Joint disabled' if enabled else 'Joint error flag reported')
                self.record('joint_diagnostic',reply=msg)
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
                self.stop_deadline = now+3.
                self.hold_accepted=False
                self.hold_samples=[]
            target = self.target
            stopping = self.stop_deadline is not None
        if request_stop:
            state=self.state()
            self.hold_target=state.joint.copy()
            self._write({'command':'movej','joint':encode_joint_deg(apply_joint_axis_sign(self.hold_target)),'v':10,'r':0,'trajectory_connect':0})
            self.record('hold_sent',joint_deg=self.hold_target.tolist())
        if stopping:
            if now > self.stop_deadline:
                raise TimeoutError('Hold unconfirmed: joint feedback or acceptance deadline exceeded')
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
        # Send every tick, including a zero step, so the CANFD stream has no gap.
        with self.lock:
            if self.target is None or epoch!=self.epoch or self.stop_pending:
                return
            self.sent=True
        self._write({'command':'movej_canfd','joint':encode_joint_deg(apply_joint_axis_sign(goal)),'follow':False})
        self.velocity=(goal-reference)/dt
        self.last_goal=goal
        self.frames+=1
        self.record('sent',joint_deg=goal.tolist(),real_joint_deg=state.joint.tolist(),target_age_s=now-stamp,confirmation='local send only')

    def _expire_query(self, now):
        if self.pending_query is None or now-self.pending_query[1]<.15:
            return
        command,stamp=self.pending_query
        self.record('query_timeout',command=command,age_s=now-stamp,
                    action='retry state query; no motion command retry')
        self.pending_query=None
        if command!='get_current_arm_state':
            self.diagnostics_enabled=False
            self.record('diagnostics_disabled',reason='missing optional query response')

    def _loop(self):
        next_query=time.monotonic()
        next_send=time.perf_counter()
        started=next_query
        next_diagnostic=next_query+.5
        diagnostic_index=0
        diagnostics=('get_joint_en_state','get_joint_err_flag')
        self.record('session',peer=self._sock.getpeername(),period_s=.02,state_period_s=.05,diagnostic_period_s=.5)
        try:
            while not self.done.is_set():
                if self.trace is not None and self.trace.error:
                    raise RuntimeError(self.trace.error)
                self._read()
                now=time.monotonic()
                with self.lock:
                    streaming=self.target is not None and not self.stop_pending and self.stop_deadline is None
                    stopping=self.stop_pending or self.stop_deadline is not None
                # At most ONE outbound command per 20ms slot. This controller
                # has returned request-parser errors for adjacent mixed commands.
                if now-started>self.cfg['state_timeout_s'] and now-self.last_arm_state_received>self.cfg['state_timeout_s']:
                    raise RuntimeError('Hold unconfirmed: feedback stale while awaiting hold' if self.stop_deadline is not None else 'Realtime feedback stale')
                self._expire_query(now)
                with self.lock:
                    if self.gripper_inflight is not None and now-self.gripper_inflight[1] > GRIPPER_ACK_WAIT_S:
                        self.gripper_error = 'gripper command not acknowledged'
                        self.gripper_inflight = None
                        self.record('gripper_timeout')
                if time.perf_counter()>=next_send:
                    if self.pending_query is not None:
                        pass  # No mixed request while a read-only reply is outstanding.
                    elif (not stopping and self.cfg.get('gripper_enabled', False) and self.gripper_inflight is None
                          and now-self.gripper_last_send >= GRIPPER_MIN_GAP_S
                          and (self.gripper_target is not None
                               or (self.gripper_resend is not None and now-self.gripper_last_send >= GRIPPER_RESEND_AFTER_S))):
                        with self.lock:
                            grip = self.gripper_target
                            self.gripper_target = None
                            resend = grip is None
                            if resend:
                                payload, self.gripper_resend = self.gripper_resend, None
                            else:
                                payload = self.gripper_resend = grip[0]
                        if payload is not None:
                            self._write(payload)
                            with self.lock:
                                self.gripper_inflight = (payload, time.monotonic())
                            self.gripper_last_send = now
                            self.record('gripper_sent',payload=payload,resend=resend)
                    elif streaming or self.stop_pending:
                        self._tick(now)
                    elif now>=next_query:
                        self._write({'command':'get_current_arm_state'})
                        next_query=now+.05
                    elif now>=next_diagnostic and not stopping and self.diagnostics_enabled:
                        self._write({'command':diagnostics[diagnostic_index % len(diagnostics)]})
                        diagnostic_index+=1
                        next_diagnostic=now+.5
                    if stopping and not self.stop_pending and self.stop_deadline is not None and now>self.stop_deadline:
                        raise TimeoutError('Hold unconfirmed: '+('joint feedback did not settle after accepted hold' if self.hold_accepted else 'no matching movej acceptance/completion'))
                    next_send=time.perf_counter()+.02
                self.wake.wait(.002)
                self.wake.clear()
        except Exception as exc:
            self.fault=str(exc)
            self.record('fault',error=self.fault)
            # Best effort only. A broken/partially written stream cannot confirm stop.
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        finally:
            self.record('io_exit',fault=self.fault)
            self.done.set()

    def stop(self):
        self.cancel_input()
        deadline=time.monotonic()+6.15
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
            writer=self.trace
            self.trace=None
        if writer is not None:
            writer.close()
        if self.sent and not self.stop_confirmed:
            raise RuntimeError('Realtime stop unconfirmed; check physical robot')

    def halt_for_exit(self):
        # One owner handles release and process shutdown; no native reconnect.
        self.stop()
        self.exit_confirmed=(not self.sent and self.stop_confirmed and
                             time.monotonic()-self.last_arm_state_received<=self.cfg['state_timeout_s'])
        if not self.exit_confirmed:
            raise RuntimeError('Exit hold not confirmed; independent guard remains responsible')

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
        f=self.follower
        # Stop/re-arm gates apply to the MODEL too, not just hardware output.
        blocked=f.operator_paused or f.protocol_pause or f.rearm_required or f.stop_pending or f.stop_deadline is not None
        if blocked:
            if not f.operator_paused and f.acknowledge_release(sample.grip,fresh(sample)):
                pass
            else:
                from copy import copy
                sample=copy(sample)
                sample.grip=0.
                self.arm.motion_started=False
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
        return f'; operator_paused={self.follower.operator_paused}; rearm_required={self.follower.rearm_required}; protocol_pause={self.follower.protocol_pause or "none"}; latest_sent={self.follower.frames} (local send only); REAL_joint_move={movement:.3f}deg; command_real_gap={gap:.3f}deg; hold_accepted={self.follower.hold_accepted}; hold_stable={self.follower.stop_confirmed}; GRIPPER DISABLED'

    def record_metadata(self):
        return {'backend':'latest-target-single-socket-owner','robot_forward':self.cfg['robot_forward'],'period_s':.02,'feedback_period_s':.05,'gripper_mode':'disabled'}
