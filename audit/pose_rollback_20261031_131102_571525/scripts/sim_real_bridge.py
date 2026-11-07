"""PICO -> existing native MuJoCo servo -> model joints -> original RealArm.

No independent IK, Cartesian hardware commands, plant replacement, or sign overrides.
"""
from __future__ import annotations
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import threading
import time
import mujoco
import numpy as np
from sim_control import CTRL_DT,fresh,quat_matrix
from vdo_pico import PicoEeServo,VdoArmAdapter


def validate_bridge(cfg):
    cfg.setdefault('wrist_mode', 'position')
    if cfg['wrist_mode'] not in ('position', 'pose'):
        raise ValueError('wrist_mode must be position or pose')
    cfg.setdefault('feedback_seed', False)
    if not isinstance(cfg['feedback_seed'], bool):
        raise ValueError('feedback_seed must be a boolean')
    cfg.setdefault('replay_delay_s',0.)
    cfg.setdefault('max_replay_lag_s',.5)
    for key,low,high in (
        ('linear_speed_m_s',.001,.30),('scale',.05,1.),
        ('max_joint_step_deg',.001,10.),('max_following_error_deg',.1,2.),
        ('state_timeout_s',.1,.5),('max_stream_gap_s',.05,.3),
        ('replay_delay_s',0.,1.),('max_replay_lag_s',.2,30.)):
        v=cfg[key]
        if not isinstance(v,(int,float)) or not np.isfinite(v) or not low<=v<=high:
            raise ValueError(f'{key} must be in [{low},{high}]')
    if cfg['max_replay_lag_s']<=cfg['replay_delay_s']:
        raise ValueError('max_replay_lag_s must exceed replay_delay_s')
    if cfg['robot_forward'] not in ('+X','-X') or not isinstance(cfg['gripper_enabled'],bool):
        raise ValueError('Invalid robot_forward or gripper_enabled')
    return cfg


class JointFollower:
    def __init__(self,real,cfg):
        self.real,self.cfg=real,cfg
        self.active=False
        self.target=None
        self.target_time=0.
        self.queue=deque()
        self.generation=0
        self.source_frames=0
        self.replayed_seq=None
        self.trace=None
        self.trace_path=None
        self.trace_lock=threading.Lock()
        self.grip_target=None
        self.sent=False
        self.frames=0
        self.fault=None
        self.last_send=None
        self.last_goal=None
        self.command_reference=None
        self.pending_since=None
        self.progress_joint=None
        initial=real.arm_state.get()
        self.initial_joint=None if initial is None else initial.joint.copy()
        self.lock=threading.Lock()
        self.io_lock=threading.Lock()

    def state(self):
        state=self.real.arm_state.get()
        stamp=getattr(self.real.link,'last_arm_state_received',0.)
        if state is None or not 0<=time.monotonic()-stamp<=self.cfg['state_timeout_s']:
            raise RuntimeError('Real feedback stale; bridge stopped')
        if state.arm_err or state.sys_err or not np.all(np.isfinite(state.joint)):
            raise RuntimeError('Real controller feedback error')
        return state

    def submit(self,q,active):
        if active:
            q=np.asarray(q,float)
            if q.shape!=(6,) or not np.all(np.isfinite(q)):
                raise ValueError('Simulation joint target must contain six finite degrees')
        with self.lock:
            now=time.monotonic()
            self.active=bool(active)
            self.target=np.asarray(q,float).copy() if active else None
            self.target_time=now
            if active:
                if self.queue and now-self.queue[0][1]>self.cfg['max_replay_lag_s']:
                    raise RuntimeError('Replay backlog exceeded max_replay_lag_s; release Grip; queued motion cancelled on stop')
                self.source_frames+=1
                self.queue.append((self.source_frames,now,self.target.copy(),self.generation))
                self.record('simulation',seq=self.source_frames,source_monotonic_s=now,joint_deg=self.target.tolist())
            else:
                self._clear_queue()

    def _clear_queue(self):
        # Caller owns self.lock. Generation prevents an old in-flight frame
        # being re-authorized by a quick Grip release/repress.
        if self.queue:
            self.record('discard',count=len(self.queue),reason='clutch/source/stop')
        self.queue.clear()
        self.generation+=1

    def record(self,event,**fields):
        with self.trace_lock:
            if self.trace is not None:
                self.trace.write(json.dumps({'type':event,'monotonic_s':time.monotonic(),**fields})+'\n')
                self.trace.flush()

    def replay_status(self):
        with self.lock:
            return {'queued_frames':len(self.queue),
                    'replay_lag_s':0. if not self.queue else max(0.,time.monotonic()-self.queue[0][1]),
                    'source_frames':self.source_frames,'replayed_seq':self.replayed_seq}

    def submit_gripper(self,position):
        with self.lock:
            self.grip_target=(int(position),time.monotonic())

    def cancel_input(self):
        with self.lock:
            self.active=False
            self.target=None
            self.grip_target=None
            self._clear_queue()

    def hold(self):
        if self.sent:
            # Original native policy reads measured joints and exits CANFD.
            result=self.real.leave_passthrough(confirmed=True)
            if result:
                detail=getattr(self.real.link,'last_movej_rpc',None)
                raise RuntimeError(f'Native hold unconfirmed: {result}; reply={json.dumps(detail,ensure_ascii=False)}; use physical emergency stop if needed')
            self.sent=False
        self.last_send=None
        self.pending_since=None
        self.progress_joint=None
        self.command_reference=None

    def step(self):
        if self.fault:
            raise RuntimeError(self.fault)
        with self.io_lock:
            with self.lock:
                active,timestamp,grip=self.active,self.target_time,self.grip_target
                epoch=self.generation
                frame=self.queue[0] if self.queue else None
                self.grip_target=None
            if not active:
                self.hold()
            else:
                state=self.state()
                now=time.monotonic()
                if now-timestamp>self.cfg['max_stream_gap_s']:
                    raise RuntimeError('Simulation/PICO source stale; no catch-up frame sent')
                if self.real._busy:
                    raise RuntimeError('Other native motion active; do not mix CLI/UI with PICO')
                if frame is not None:
                    seq,source_time,q,generation=frame
                    if now-source_time>self.cfg['max_replay_lag_s']:
                        raise RuntimeError('Replay backlog exceeded max_replay_lag_s; queued motion cancelled on stop')
                    if now-source_time<self.cfg['replay_delay_s']:
                        frame=None
                # An empty queue holds the last replayed point, never jumps
                # to the latest source pose or skips intermediate frames.
                if frame is None:
                    q=state.joint if self.command_reference is None else self.command_reference
                if np.max(np.abs(q-state.joint))<.05:
                    self.pending_since=None
                if self.progress_joint is not None:
                    if np.max(np.abs(state.joint-self.progress_joint))>=.01:
                        self.pending_since=None
                        self.progress_joint=state.joint.copy()
                    elif self.pending_since is not None and now-self.pending_since>1.:
                        raise RuntimeError('NO REAL JOINT RESPONSE: commands sent but measured joints did not move 0.01deg in 1s; stopped, not increasing limits')
                # Match simu.session's native joint slider: at most 8 degrees
                # from the previous command. The former 5 deg/s cap throttled
                # the real arm while the independent simulation kept advancing.
                reference=state.joint if self.command_reference is None else self.command_reference
                reference_gap=float(np.max(np.abs(reference-state.joint)))
                waiting_for_feedback=reference_gap>self.cfg['max_following_error_deg']
                if waiting_for_feedback:
                    # Keep the FIFO head until measured joints catch up. Do
                    # not send more commands or treat ordinary lag as a fault.
                    goal=reference.copy()
                else:
                    command_delta=q-reference
                    cap=min(self.cfg['max_joint_step_deg'],8.0)
                    span=max(np.max(np.abs(command_delta)),1e-12)
                    scale=min(1.,cap/span)
                    goal=reference+command_delta*scale
                    goal_gap=float(np.max(np.abs(goal-state.joint)))
                    if goal_gap>self.cfg['max_following_error_deg']:
                        # Approach the existing feedback window, then wait
                        # there for the physical arm instead of sending a
                        # >2-degree lead or faulting immediately.
                        available=max(0.,self.cfg['max_following_error_deg']-reference_gap)
                        goal=reference+command_delta*min(scale,available/span)
                        waiting_for_feedback=available<.001
                    else:
                        waiting_for_feedback=False
                command_advance=float(np.max(np.abs(goal-reference)))
                if not waiting_for_feedback and command_advance>=.001 and np.max(np.abs(goal-state.joint))>=.001:
                    # Re-check clutch immediately before the only arm output.
                    with self.lock:
                        allowed=self.active and epoch==self.generation
                    if allowed:
                        self.sent=True
                        # Same request/reply CANFD transport as the original
                        # UI slider (RealArm.set_q -> JsonMotionPolicy.canfd_deg).
                        # The sender now runs inline with PICO's control tick,
                        # so a slow controller paces the source instead of
                        # accumulating an unbounded background replay backlog.
                        ret=self.real.Movej_CANFD(goal)
                        if ret:
                            raise RuntimeError(f'Native joint send failed: {ret}')
                        self.frames+=1
                        self.last_goal=goal.copy()
                        self.command_reference=goal.copy()
                        if self.pending_since is None and np.max(np.abs(goal-state.joint))>=.05:
                            self.pending_since=time.monotonic()
                            self.progress_joint=state.joint.copy()
                        self.last_send=time.monotonic()
                        self.record('sent',seq=None if frame is None else seq,
                                    source_monotonic_s=None if frame is None else source_time,
                                    joint_deg=goal.tolist(),real_joint_deg=state.joint.tolist(),
                                    native_api_ret=int(ret))
                if frame is not None and np.max(np.abs(q-goal))<1e-8:
                    with self.lock:
                        if self.active and generation==self.generation and self.queue and self.queue[0][0]==seq:
                            self.queue.popleft()
                            self.replayed_seq=seq
            if grip is not None:
                position,stamp=grip
                if time.monotonic()-stamp>self.cfg['max_stream_gap_s']:
                    raise RuntimeError('Gripper source stale; queued command discarded')
                self.state()
                if self.real._busy:
                    raise RuntimeError('Other native motion active; gripper command withheld')
                ret=self.real.Set_Gripper_Position(position,False,canonical=True)
                if ret:
                    detail=getattr(self.real,'last_gripper_command',{})
                    raise RuntimeError(f'Gripper command unconfirmed: {ret}; detail={json.dumps(detail,ensure_ascii=False)}; no automatic retry')

    def start(self):
        self.trace_path=Path(__file__).resolve().parents[1]/'logs/real'/('trajectory_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
        self.trace_path.parent.mkdir(parents=True,exist_ok=True)
        self.trace=self.trace_path.open('x',encoding='utf-8')
        self.record('meta',replay_delay_s=self.cfg['replay_delay_s'],max_replay_lag_s=self.cfg['max_replay_lag_s'],
                    joint_send_mode='inline native request/reply',
                    max_joint_step_deg=min(self.cfg['max_joint_step_deg'],8.),
                    initial_joint_deg=None if self.initial_joint is None else self.initial_joint.tolist())
        print('[bridge] trajectory recording:',self.trace_path,flush=True)

    def stop(self):
        self.submit(None,False)
        with self.lock:
            self.grip_target=None
        with self.io_lock:
            try:
                self.hold()
            finally:
                with self.trace_lock:
                    if self.trace is not None:
                        self.trace.close()
                        self.trace=None


class BridgeSimAdapter(VdoArmAdapter):
    def __init__(self,sim,follower,cfg):
        super().__init__(sim)
        self.follower,self.cfg=follower,cfg
        self.origin_tcp=self.tcp_pos.copy()
        self.trigger_ready=False
        self.previous_trigger=0.
        self.gripper_label='OPEN command preview only; no startup gripper command'
        if not cfg['gripper_enabled']:
            self.gripper_label='REAL GRIPPER DISABLED; trigger has no hardware output'
        self.last_gripper_time=0.
        self.input_permitted=False
        self.motion_started=False
        self.stalled_ticks=0
        self.motion_reason=''
        self.preview_gripper(1000)

    def preview_gripper(self,position):
        width=self.native._units_to_m(position)
        with self.native.lock.data_lock():
            if self.native._grip_qadr is not None:
                l,r=self.native._grip_qadr
                self.data.qpos[l],self.data.qpos[r]=-width/2,width/2
            if self.native._grip_act>=0:
                self.data.ctrl[self.native._grip_act]=width
            mujoco.mj_forward(self.model,self.data)

    def gripper(self,trigger):
        if not self.input_permitted or not self.cfg['gripper_enabled']:
            return
        trigger=float(np.clip(trigger,0.,1.))
        if not self.trigger_ready:
            if trigger<=.1:
                self.trigger_ready=True
                self.previous_trigger=trigger
            return
        if abs(trigger-self.previous_trigger)<.03 or time.monotonic()-self.last_gripper_time<.2:
            return
        target=int(np.clip(round((1-trigger)*1000),1,1000))
        self.preview_gripper(target)
        self.follower.submit_gripper(target)
        self.previous_trigger=trigger
        self.last_gripper_time=time.monotonic()
        self.gripper_label=f'PICO command={target}/1000 (not measured)'

    def move(self,target,quat,speed):
        self.motion_reason=''
        previous=self.native.q_cmd_deg.copy()
        super().move(target,quat,speed)  # The existing working simulation servo/IK.
        delta=self.native.q_cmd_deg-previous
        self.stalled_ticks=self.stalled_ticks+1 if np.linalg.norm(np.asarray(target)-self.pos)>.001 and np.max(np.abs(delta))<1e-5 else 0
        if self.stalled_ticks>=40:
            if not getattr(self.follower,'disconnected',False):
                raise RuntimeError('Native simulation IK made no progress; release Grip. Check singular pose/workspace; no automatic repositioning')
            self.last_ret=1
            self.motion_reason='Preview IK cannot reach this direction from current pose; release Grip and change direction'
        # The source model runs independently. Physical speed and tracking
        # limits belong to the delayed FIFO sender, not the PICO simulation.
        cap=self.joint_step_cap()
        if np.max(np.abs(delta))>cap:
            self.native.set_q(np.deg2rad(previous+delta*cap/np.max(np.abs(delta))))

    def joint_step_cap(self):
        return self.cfg['max_joint_step_deg']

    def hold(self):
        self.follower.submit(None,False)
        self.motion_started=False
        self.stalled_ticks=0
        # Local model only. Never call RealArm.set_q to synchronize the model.
        self.native.set_q(np.deg2rad(self.native.q_deg))
        self.holding=True

    def recording_state(self):
        if getattr(self.follower,'disconnected',False):
            return {'real_joint_deg':None,
                    'sim_joint_deg':self.native.q_deg.copy(),
                    'sent_joint_deg':None if self.follower.last_goal is None else self.follower.last_goal.copy(),
                    'sent_frames':self.follower.frames,
                    'feedback_age_s':None}
        state=self.follower.real.arm_state.get()
        return {'real_joint_deg':None if state is None else state.joint.copy(),
                'sim_joint_deg':self.native.q_deg.copy(),
                'sent_joint_deg':None if self.follower.last_goal is None else self.follower.last_goal.copy(),
                'sent_frames':self.follower.frames,
                **self.follower.replay_status(),
                'feedback_age_s':time.monotonic()-getattr(self.follower.real.link,'last_arm_state_received',0.)}


class DisconnectedFollower:
    """Preview the bridge output without opening any RM65 socket."""
    disconnected=True

    def __init__(self,initial_joint):
        self.initial_joint=np.asarray(initial_joint,float).copy()
        self.target=None
        self.last_goal=None
        self.frames=0
        self.sent=False
        self.fault=None

    def submit(self,q,active):
        self.target=None if not active else np.asarray(q,float).copy()
        self.sent=bool(active)
        if active:
            self.last_goal=self.target.copy()
            self.frames+=1

    def submit_gripper(self,position):
        del position

    def cancel_input(self):
        self.target=None
        self.sent=False

    def start(self):
        pass

    def stop(self):
        self.cancel_input()


class SimRealDisconnectedServo(PicoEeServo):
    """The real-bridge UI and limits with the final hardware link removed."""

    def __init__(self,sim,cfg):
        validate_bridge(cfg)
        self.cfg=cfg
        self.display_arm=sim
        if 'preview_joint_deg' in cfg:
            cached=np.asarray(cfg['preview_joint_deg'],float)
            if cached.shape!=(6,) or not np.all(np.isfinite(cached)):
                raise ValueError('preview_joint_deg must contain six finite MDH joint angles')
            q=sim.api_joint_deg_to_mj(cached)
            if np.any(q<sim.kin.q_min) or np.any(q>sim.kin.q_max):
                raise ValueError('Cached preview pose outside model joint limits')
            sim.pin_meas_q(q)
            mujoco.mj_forward(sim.model,sim.data)
            print('[bridge] cached real MDH joints (not live):',cached.tolist(),flush=True)
        sim._pico_pos_tol=5e-5
        base=sim.model.body('base_link').id
        for body in range(base,sim.model.nbody):
            ancestor=body
            while ancestor>0 and ancestor!=base:
                ancestor=int(sim.model.body_parentid[ancestor])
            if ancestor==base:
                sim.model.body_gravcomp[body]=1.
        self.follower=DisconnectedFollower(sim.q_deg)
        print(f"[bridge] preview settings: scale={cfg['scale']}; TCP speed={cfg['linear_speed_m_s']}m/s; joint step <= {cfg['max_joint_step_deg']}deg/tick",flush=True)
        super().__init__(sim,speed=cfg['linear_speed_m_s'],scale=cfg['scale'],wrist=cfg['wrist_mode'],
                         yaw=180. if cfg['robot_forward']=='-X' else 0.,
                         adapter_factory=lambda s:BridgeSimAdapter(s,self.follower,cfg))

    def start(self):
        if self.enabled:
            return
        self.arm.trigger_ready=False
        super().start()
        print('[bridge] DISCONNECTED PREVIEW: PICO -> native simulation -> model joints; no RM65 socket.',flush=True)

    def prepare_sample(self,sample):
        self.arm.input_permitted=bool(fresh(sample) and not sample.menu)
        if not self.arm.input_permitted:
            self.arm.trigger_ready=False
            self.follower.cancel_input()
        return sample

    def after_control_tick(self,sample):
        active=bool(self.control.active and self.arm.input_permitted)
        if active and np.linalg.norm(self.control.operator_delta)>=.001:
            self.arm.motion_started=True
        with self.display_arm.lock.data_lock():
            self.display_arm.set_q(np.deg2rad(self.display_arm.q_cmd_deg))
            if active and self.arm.motion_started:
                for _ in range(max(1,int(round(CTRL_DT/self.arm.model.opt.timestep)))):
                    mujoco.mj_step(self.arm.model,self.arm.data)
            mujoco.mj_forward(self.arm.model,self.arm.data)
            joints=self.display_arm.q_deg.copy()
        self.follower.submit(joints,active and self.arm.motion_started)

    def status_suffix(self):
        sim_mm=float(np.linalg.norm(self.arm.measured_tcp-self.arm.origin_tcp)*1000)
        limit=f'; {self.arm.motion_reason}' if self.arm.motion_reason else ''
        return f'; preview_frames={self.follower.frames}; SIM_move={sim_mm:.2f}mm; RM65 DISCONNECTED; {self.arm.gripper_label}{limit}'

    def draw_overlay(self,viewer):
        if hasattr(viewer,'set_texts'):
            with self._status_lock:
                text=self._status
            viewer.set_texts((None,None,'PICO -> SIM MODEL | RM65 DISCONNECTED',
                              text+'\nGrip: clutch; no robot network connection\n'
                              +f"Forward={self.cfg['robot_forward']}; same bridge model/IK/limits"))

    def record_metadata(self):
        return {'backend':'pico-simulation-to-real-preview','robot_forward':self.cfg['robot_forward'],
                'real_connection':'disabled'}


class SimRealBridgeServo(PicoEeServo):
    def __init__(self,real,cfg):
        from simu.rm65_arm import Rm65ExtArm
        validate_bridge(cfg)
        real.fetch_arm_state()
        self.follower=JointFollower(real,cfg)
        state=self.follower.state()
        sim=Rm65ExtArm(xml_path=real.kin.xml_path,model=real.model,data=real.data)
        sim.lock=real.lock  # Shared display data always uses ONE data lock.
        # The scene's gravcomp variant omits some attached payload bodies.
        # Compensate the ENTIRE simulated robot subtree, not the table/objects.
        # Otherwise passive model sag is inadvertently sent as operator motion.
        base=sim.model.body('base_link').id
        for body in range(base,sim.model.nbody):
            ancestor=body
            while ancestor>0 and ancestor!=base:
                ancestor=int(sim.model.body_parentid[ancestor])
            if ancestor==base:
                sim.model.body_gravcomp[body]=1.
        self.arm_dofs=np.array([sim.model.jnt_dofadr[sim.model.joint(f'joint_{i}').id] for i in range(1,7)])
        q=real.api_joint_deg_to_mj(state.joint)
        if np.any(q<sim.kin.q_min-1e-6) or np.any(q>sim.kin.q_max+1e-6):
            raise RuntimeError('Measured real pose outside simulation joint range; no startup motion')
        sim.pin_meas_q(q)
        sim._pico_pos_tol=5e-5
        self.display_arm=sim
        self.real=real
        self.real.mirror_meas=False
        super().__init__(sim,speed=cfg['linear_speed_m_s'],scale=cfg['scale'],wrist=cfg['wrist_mode'],
                         yaw=180. if cfg['robot_forward']=='-X' else 0.,
                         adapter_factory=lambda s:BridgeSimAdapter(s,self.follower,cfg))
        self.cfg=cfg

    def start(self):
        if self.enabled:
            return
        self.arm.trigger_ready=False
        self.follower.start()
        try:
            super().start()
        except Exception:
            self.follower.stop()
            raise
        print(f"[bridge] PICO -> native simulation -> original RealArm request/reply joint API; inline, max step={min(self.cfg['max_joint_step_deg'],8.):g}deg. Release Grip discards pending points.",flush=True)

    def prepare_sample(self,sample):
        if self.follower.fault:
            raise RuntimeError(self.follower.fault)
        self.arm.input_permitted=bool(fresh(sample) and not sample.menu)
        if not self.arm.input_permitted:
            self.arm.trigger_ready=False
            self.follower.cancel_input()
        if self.real._busy:
            raise RuntimeError('Native motion is busy; PICO bridge cannot run simultaneously')
        return sample

    def after_control_tick(self,sample):
        active=bool(self.control.active and self.arm.input_permitted)
        if active and np.linalg.norm(self.control.operator_delta)>=.001:
            self.arm.motion_started=True
        # On clutch release, only re-align the LOCAL model after the output
        # worker has completed its hold. Never hold the data lock while waiting
        # for controller I/O. No new hardware command is made by this branch.
        if not active and self.follower.io_lock.acquire(blocking=False):
            try:
                if not self.follower.sent:
                    state=self.follower.state()
                    with self.display_arm.lock.data_lock():
                        self.display_arm.pin_meas_q(self.real.api_joint_deg_to_mj(state.joint))
                        self.display_arm.data.qvel[self.arm_dofs]=0.
            finally:
                self.follower.io_lock.release()
        with self.display_arm.lock.data_lock():
            # RealArm CANFD updates its local ctrl too. Restore the independent
            # simulation command before stepping source physics.
            self.display_arm.set_q(np.deg2rad(self.display_arm.q_cmd_deg))
            if active and self.arm.motion_started:
                for _ in range(max(1,int(round(CTRL_DT/self.arm.model.opt.timestep)))):
                    mujoco.mj_step(self.arm.model,self.arm.data)
            mujoco.mj_forward(self.arm.model,self.arm.data)
            measured=self.display_arm.q_deg.copy()
            commanded=self.display_arm.q_cmd_deg.copy()
            # Match the original UI slider: use ctrl when its target is >0.5
            # degrees ahead of qpos; otherwise use the measured model joints.
            joints=commanded if np.max(np.abs(commanded-measured))>.5 else measured
        self.follower.submit(joints,active and self.arm.motion_started)
        # Use the original request/reply native joint API inline with this
        # source tick. The PICO worker cannot outrun the physical sender.
        self.follower.step()

    def stop(self):
        self._stop.set()
        self.follower.cancel_input()
        try:
            super().stop()
        finally:
            self.follower.stop()

    def status_suffix(self):
        state=self.follower.state()
        gap=float(np.max(np.abs(self.display_arm.q_deg-state.joint)))
        sim_mm=float(np.linalg.norm(self.arm.measured_tcp-self.arm.origin_tcp)*1000)
        real_deg=float(np.max(np.abs(state.joint-self.follower.initial_joint)))
        command_gap=0. if self.follower.last_goal is None else float(np.max(np.abs(self.follower.last_goal-state.joint)))
        replay=self.follower.replay_status()
        return f"; sent_frames={self.follower.frames} (native API returned success; feedback confirms motion); SIM_move={sim_mm:.2f}mm; REAL_joint_move={real_deg:.3f}deg; sim_real_gap={gap:.3f}deg; command_real_gap={command_gap:.3f}deg; queue={replay['queued_frames']}; replay_lag={replay['replay_lag_s']:.2f}s; {self.arm.gripper_label}"

    def draw_overlay(self,viewer):
        if hasattr(viewer,'set_texts'):
            with self._status_lock:
                text=self._status
            viewer.set_texts((None,None,'PICO -> SIM MODEL -> REAL RM65',
                              text+'\nGrip: clutch; trigger: gripper command (no gripper feedback)\n'
                              +f"Forward={self.cfg['robot_forward']}; model initialized from real joints; no auto-home"))

    def record_metadata(self):
        return {'backend':'pico-simulation-to-native-real','robot_forward':self.cfg['robot_forward'],
                'gripper_mode':('trigger command only; no feedback synchronization' if self.cfg['gripper_enabled'] else 'disabled; no real gripper output')}

    def on_error(self,exc):
        self.follower.cancel_input()
        self.arm.input_permitted=False
        path=Path(__file__).resolve().parents[1]/'logs/real/bridge_error.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps({'error':str(exc),'sender_fault':self.follower.fault,
                                    'frames_sent':self.follower.frames,'status':self._status,
                                    'last_sent_joint_deg':None if self.follower.last_goal is None else self.follower.last_goal.tolist(),
                                    'real_state_joint_deg':self.real.arm_state.get().joint.tolist(),
                                    'sim_joint_deg':self.display_arm.q_deg.tolist(),
                                    'hold_rpc':getattr(self.real.link,'last_movej_rpc',None),
                                    'gripper_command':getattr(self.real,'last_gripper_command',None)},ensure_ascii=False,indent=2),encoding='utf-8')
