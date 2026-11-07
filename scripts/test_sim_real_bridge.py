"""Offline real native model and fake controller; never opens a robot socket."""
import json
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from launch_vdo_pico import bootstrap,ROOT
from test_real_pico import FakeLink
from test_vdo_pico import sample
from sim_real_bridge import SimRealBridgeServo, SimRealDisconnectedServo


class BridgeLink(FakeLink):
    def __init__(self,wire_deg):
        super().__init__(wire_deg)
        self.follow_fraction=1.0

    def rpc(self,payload,*,timeout_s=None):
        if payload['command']=='set_gripper_position':
            self.calls.append(dict(payload))
            return {'command':'set_gripper','state':True}
        if payload['command']=='movej_canfd':
            self.sent.append(dict(payload))
            if self.follow:
                goal=np.asarray(payload['joint'],float)/1000.
                self.wire_deg+=self.follow_fraction*(goal-self.wire_deg)
        result=super().rpc(payload,timeout_s=timeout_s)
        if payload['command']=='get_current_arm_state':
            self.last_arm_state_received=time.monotonic()
        return result

    def send_stream(self,payload,*,timeout_s=.1):
        self._sock.sendall(json.dumps(payload).encode())


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            bootstrap()
        except FileNotFoundError:
            raise unittest.SkipTest('Native source not installed')

    def make(self,joints=None):
        from simu.box_scene import BoxScene,apply_home_keyframe
        from real.real_arm import RealArm
        from real.json_codec import JOINT_AXIS_SIGN
        box=BoxScene(camera='d435i',gravcomp=True)
        m=mujoco.MjModel.from_xml_path(str(box.scene_xml))
        d=mujoco.MjData(m)
        apply_home_keyframe(m,d)
        home=np.rad2deg([d.qpos[m.jnt_qposadr[m.joint(f'joint_{i}').id]] for i in range(1,7)])
        if joints is not None:
            home=np.asarray(joints,float)
        link=BridgeLink(home*np.asarray(JOINT_AXIS_SIGN))
        real=RealArm(link,xml_path=box.scene_xml,model=m,data=d)
        link.kin=real.kin
        cfg=json.loads((ROOT/'config/sim_real_bridge.example.json').read_text())
        cfg['replay_delay_s']=0.  # Deterministic fast ticks; delay is tested separately.
        bridge=SimRealBridgeServo(real,cfg)
        return bridge,real,link

    def test_sim_joint_target_uses_original_ui_per_frame_cap(self):
        b,r,link=self.make()
        b.cfg['max_joint_step_deg']=10.0
        b.arm.cfg['max_joint_step_deg']=10.0
        before=b.display_arm.q_cmd_deg.copy()
        target=b.arm.pos+np.array([.01,0.,0.])
        b.arm.move(target,b.arm.quat,.08)
        moved=b.display_arm.q_cmd_deg-before
        self.assertGreater(np.max(np.abs(moved)),0.)
        self.assertLessEqual(np.max(np.abs(moved)),b.cfg['max_joint_step_deg'])

    def test_sim_joint_slew_is_independent_of_real_gap(self):
        b,r,link=self.make()
        nominal=b.arm.joint_step_cap()
        measured=b.follower.state().joint.copy()
        simulated=measured.copy()
        simulated[0]+=.5
        b.display_arm.pin_meas_q(b.display_arm.api_joint_deg_to_mj(simulated))
        slowed=b.arm.joint_step_cap()
        self.assertGreater(nominal,0.)
        self.assertGreater(slowed,0.)
        self.assertEqual(slowed,nominal)

    def test_inline_real_send_stays_paced_with_native_model(self):
        b,r,link=self.make()
        b.control.speed=.08
        b.control.scale=.8
        s=sample([0.,1.,0.],click=True)
        self.tick(b,r,s)
        s.axis_click=False
        s.grip=1.
        self.tick(b,r,s)
        max_command_gap=0.
        for k in range(120):
            s.pos=np.array([0.,1.,0.])+np.array([0.,0.,-.10])*min(1.,k/35.)
            s.stamp=time.time()
            self.tick(b,r,s)
            self.assertIsNone(b.follower.fault)
            real=r.arm_state.get().joint
            if b.follower.last_goal is not None:
                command_gap=np.max(np.abs(b.follower.last_goal-real))
                max_command_gap=max(max_command_gap,command_gap)
                self.assertLessEqual(command_gap,b.cfg['max_following_error_deg'])
        self.assertGreater(len(link.sent),10)
        self.assertLessEqual(max_command_gap,b.cfg['max_following_error_deg'])
        self.assertEqual(b.follower.replay_status()['queued_frames'],0)

    def test_fifo_delay_preserves_turning_points_and_records_both_ends(self):
        import io
        b,r,link=self.make()
        f=b.follower
        f.cfg['replay_delay_s']=.1
        f.trace=io.StringIO()
        start=time.monotonic()
        q=f.state().joint.copy()
        with patch('sim_real_bridge.time.monotonic',return_value=start) as clock:
            for t,offset in ((0.,.1),(.02,-.1),(.04,.2)):
                clock.return_value=start+t
                f.submit(q+offset,True)
            clock.return_value=start+.099
            r.fetch_arm_state()
            f.step()
            self.assertEqual(link.sent,[])
            for t in (.101,.121,.141):
                clock.return_value=start+t
                r.fetch_arm_state()
                f.step()
        goals=np.array([np.asarray(x['joint'],float)/1000. for x in link.sent])
        from real.json_codec import JOINT_AXIS_SIGN
        goals*=np.asarray(JOINT_AXIS_SIGN)
        np.testing.assert_allclose(goals-q,np.array([[.1]*6,[-.1]*6,[.2]*6]),atol=.0011)
        self.assertEqual(f.replayed_seq,3)
        self.assertEqual(len(f.queue),0)
        records=[json.loads(row) for row in f.trace.getvalue().splitlines()]
        self.assertEqual([x['seq'] for x in records if x['type']=='simulation'],[1,2,3])
        self.assertEqual([x['seq'] for x in records if x['type']=='sent'],[1,2,3])

    def test_release_discards_delayed_motion_and_repress_never_resumes_it(self):
        b,r,link=self.make()
        f=b.follower
        f.cfg['replay_delay_s']=.1
        q=f.state().joint.copy()
        f.submit(q+.5,True)
        f.step()
        self.assertEqual(link.sent,[])
        f.submit(None,False)
        f.step()
        self.assertEqual(len(f.queue),0)
        f.cfg['replay_delay_s']=0.
        f.submit(q-.04,True)
        f.step()
        np.testing.assert_allclose(f.last_goal,q-.04)
        self.assertEqual(f.frames,1)

    def test_backlog_cannot_replay_indefinitely(self):
        b,r,link=self.make()
        f=b.follower
        start=time.monotonic()
        with patch('sim_real_bridge.time.monotonic',return_value=start) as clock:
            f.submit(f.state().joint+.1,True)
            clock.return_value=start+f.cfg['max_replay_lag_s']+.01
            with self.assertRaisesRegex(RuntimeError,'backlog'):
                f.submit(f.target,True)
        f.cancel_input()
        self.assertEqual(len(f.queue),0)
        self.assertEqual(link.sent,[])

    def test_inline_replay_drains_each_fresh_target_without_backlog(self):
        b,r,link=self.make()
        f=b.follower
        q=f.state().joint.copy()
        f.submit(q+1.,True)
        f.step()
        self.assertEqual(len(f.queue),0)
        for _ in range(10):
            r.fetch_arm_state()
            f.submit(q+1.,True)
            f.step()
        self.assertIsNone(f.fault)
        self.assertEqual(f.frames,1)
        self.assertEqual(len(f.queue),0)
        np.testing.assert_allclose(f.last_goal,q+1.,atol=.001)

    def test_slow_hardware_waits_without_skipping_the_fifo_head(self):
        b,r,link=self.make()
        f=b.follower
        link.follow=False
        q=f.state().joint.copy()
        f.submit(q+10.,True)
        for _ in range(30):
            f.step()
        self.assertEqual(len(f.queue),1)
        self.assertIsNone(f.replayed_seq)
        self.assertLessEqual(np.max(np.abs(f.last_goal-q)),f.cfg['max_following_error_deg'])
        self.assertGreater(np.max(np.abs(f.last_goal-q)),1.8)
        f.submit(None,False)
        f.step()
        self.assertFalse(f.sent)
        self.assertEqual(len(f.queue),0)

    def test_disconnected_real_entry_keeps_bridge_controller_without_real_link(self):
        from simu.box_scene import BoxScene,apply_home_keyframe
        from simu.rm65_arm import Rm65ExtArm
        box=BoxScene(camera='d435i',gravcomp=True)
        m=mujoco.MjModel.from_xml_path(str(box.scene_xml))
        d=mujoco.MjData(m)
        apply_home_keyframe(m,d)
        sim=Rm65ExtArm(xml_path=box.scene_xml,model=m,data=d)
        cfg=json.loads((ROOT/'config/sim_real_bridge.example.json').read_text())
        cfg['gripper_enabled']=False
        bridge=SimRealDisconnectedServo(sim,cfg)
        self.assertTrue(bridge.follower.disconnected)
        self.assertFalse(hasattr(sim,'link'))
        self.assertIn('RM65 DISCONNECTED',bridge.status_suffix())
        tcp0=bridge.arm.measured_tcp.copy()
        for _ in range(500):
            mujoco.mj_step(m,d)
        self.assertLess(np.linalg.norm(bridge.arm.measured_tcp-tcp0),.0005)
        s=sample([0.,1.,0.],click=True)
        bridge.prepare_sample(s)
        bridge.control.update(s,0.)
        bridge.after_control_tick(s)
        self.assertEqual(bridge.follower.frames,0)

    def test_disconnected_preview_loads_cached_real_joints_without_socket(self):
        from simu.box_scene import BoxScene
        from simu.rm65_arm import Rm65ExtArm
        box=BoxScene(camera='d435i',gravcomp=True)
        sim=Rm65ExtArm(xml_path=box.scene_xml)
        cfg=json.loads((ROOT/'config/sim_real_bridge.example.json').read_text())
        cfg['preview_joint_deg']=[0.233,-0.127,-0.219,-0.087,2.033,12.595]
        with patch('socket.create_connection',side_effect=AssertionError('Unexpected hardware connection')):
            bridge=SimRealDisconnectedServo(sim,cfg)
            np.testing.assert_allclose(sim.q_deg,cfg['preview_joint_deg'],atol=1e-8)
            self.assertEqual(bridge.arm.native._pico_pos_tol,5e-5)
            self.assertTrue(bridge.follower.disconnected)

    def test_preview_beyond_old_trial_radius_keeps_running_and_releases(self):
        b,r,link=self.make()
        preview=SimRealDisconnectedServo(b.display_arm,b.cfg)
        start=preview.arm.measured_tcp.copy()
        for pos,grip,click in (([0.,1.,0.],0.,True),([0.,1.,0.],1.,False)):
            s=sample(pos,grip=grip,click=click)
            preview.prepare_sample(s)
            preview.control.update(s,0.)
            preview.after_control_tick(s)
        for k in range(100):
            s=sample([0.,1.,-.25*min(1.,(k+1)/20.)],grip=1.)
            preview.prepare_sample(s)
            preview.control.update(s,0.)
            preview.after_control_tick(s)
        self.assertGreater(np.linalg.norm(preview.control.target_tcp-preview.arm.origin_tcp),.03)
        self.assertGreater(preview.follower.frames,50)
        self.assertGreater(np.linalg.norm(preview.arm.measured_tcp-start),.001)
        self.assertNotIn('workspace limit',preview.status_suffix())
        s=sample([0.,1.,-.25],grip=0.)
        preview.prepare_sample(s)
        preview.control.update(s,0.)
        preview.after_control_tick(s)
        self.assertFalse(preview.control.active)
        self.assertFalse(preview.follower.sent)
        self.assertEqual(link.sent,[])

    def tick(self,b,r,s):
        r.fetch_arm_state()
        b.prepare_sample(s)
        b.control.update(s,0.)
        b.after_control_tick(s)

    def test_startup_only_reads_joints_no_motion_no_gripper_sync(self):
        b,r,link=self.make()
        np.testing.assert_allclose(b.display_arm.q_deg,r.arm_state.get().joint,atol=1e-7)
        self.assertFalse(r.mirror_meas)
        self.assertEqual(link.sent,[])
        self.assertEqual([x['command'] for x in link.calls],['get_current_arm_state'])
        self.assertIs(b.display_arm.data,r.data)
        self.assertIs(b.display_arm.lock,r.lock)
        self.assertEqual(type(r.plant).__name__,'RealPlant')

    def test_native_joint_sign_applied_exactly_once(self):
        from real.json_codec import JOINT_AXIS_SIGN
        b,r,link=self.make()
        goal=r.arm_state.get().joint+np.array([.04,.04,.04,.04,.04,.04])
        b.follower.submit(goal,True)
        b.follower.step()
        self.assertEqual(link.sent[-1]['command'],'movej_canfd')
        np.testing.assert_array_equal(link.sent[-1]['joint'],np.rint(goal*np.asarray(JOINT_AXIS_SIGN)*1000))
        b.follower.submit(None,False)
        b.follower.step()
        self.assertFalse(b.follower.sent)

    def test_real_feedback_and_session_do_not_overwrite_source(self):
        from simu.session import ArmSession
        b,r,link=self.make()
        session=ArmSession(r,model=r.model,data=r.data,world_mode='mirror',xbox=b)
        q=b.display_arm.q_cmd_deg+np.array([.1,0,0,0,0,0])
        b.display_arm.pin_meas_q(np.deg2rad(q))
        r.fetch_arm_state()
        session._mirror_qpos()
        np.testing.assert_allclose(b.display_arm.q_deg,q)
        np.testing.assert_allclose(b.display_arm.q_cmd_deg,q)

    def test_idle_and_grip_capture_send_no_frames(self):
        b,r,link=self.make()
        s=sample([0.,1.,0.],click=True)
        for _ in range(15):
            s.stamp=time.time()
            self.tick(b,r,s)
        s.axis_click=False
        s.grip=1.
        for _ in range(15):
            s.stamp=time.time()
            self.tick(b,r,s)
        self.assertTrue(b.control.active)
        self.assertEqual(link.sent,[])

    def test_viewer_and_status_do_not_invert_locks_and_worker_can_stop(self):
        b,r,link=self.make()
        entered=threading.Event()
        viewer_holds_data=threading.Event()
        blocked=[]
        closed=[]
        viewer=SimpleNamespace(is_running=lambda:True,set_texts=lambda texts:None)
        b.viewer=viewer
        b.source=SimpleNamespace(sample=lambda:sample([0.,1.,0.]),
                                 close=lambda:closed.append(True),diagnostics_summary=lambda:'FAKE SDK')
        original=b.status_suffix
        def status():
            entered.set()
            if not viewer_holds_data.wait(1.):
                raise RuntimeError('Fake viewer did not reach the test barrier')
            b._stop.set()  # One whole control tick, including its finally block.
            return original()
        b.status_suffix=status
        def render():
            if not entered.wait(1.):
                blocked.append('status never reached')
                return
            with b.display_arm.lock.data_lock():
                viewer_holds_data.set()
                # Same lock order as native viewer: model data -> overlay.
                # A timeout makes the old deadlock fail without hanging tests.
                acquired=b._status_lock.acquire(timeout=.2)
                if not acquired:
                    blocked.append('viewer waits for status while status waits for model')
                else:
                    b._status_lock.release()
                    b.draw_overlay(viewer)
        render_thread=threading.Thread(target=render,daemon=True)
        render_thread.start()
        b._thread=threading.Thread(target=b._loop,daemon=True)
        b.enabled=True
        with patch('vdo_pico._thread.interrupt_main'):
            b._thread.start()
            b._thread.join(timeout=2.)
            render_thread.join(timeout=2.)
        self.assertFalse(b._thread.is_alive(),'PICO worker failed to exit')
        self.assertFalse(render_thread.is_alive(),'viewer failed to exit')
        self.assertEqual(blocked,[])
        self.assertIsNone(b.error)
        b.stop()
        self.assertEqual(closed,[True])
        self.assertEqual(link.sent,[])

    def test_tracking_and_joint_output_continue_with_concurrent_viewer(self):
        b,r,link=self.make()
        b.cfg['replay_delay_s']=.1
        render_stop=threading.Event()
        counts={'samples':0,'renders':0,'closed':0}
        errors=[]
        viewer=SimpleNamespace(is_running=lambda:True,set_texts=lambda texts:None)
        b.viewer=viewer
        def next_sample():
            r.fetch_arm_state()  # Fake transport only, no socket.
            k=counts['samples']
            counts['samples']+=1
            s=sample([0.,1.,0.],click=(k==0),grip=0. if k==0 else 1.)
            if k>1:
                s.pos[2]-=.008*min(1.,(k-1)/25.)
            if k>=89:
                b._stop.set()
            return s
        def close_source():
            counts['closed']+=1
        b.source=SimpleNamespace(sample=next_sample,close=close_source,diagnostics_summary=lambda:'FAKE SDK')
        b.on_error=lambda exc:errors.append(str(exc))
        def render():
            while not render_stop.is_set():
                with b.display_arm.lock.data_lock():
                    b.draw_overlay(viewer)
                    counts['renders']+=1
                render_stop.wait(.005)
        render_thread=threading.Thread(target=render,daemon=True)
        b._thread=threading.Thread(target=b._loop,daemon=True)
        b.enabled=True
        try:
            with patch('vdo_pico._thread.interrupt_main'),patch('builtins.print'):
                render_thread.start()
                b.follower.start()
                b._thread.start()
                b._thread.join(timeout=6.)
            self.assertFalse(b._thread.is_alive(),'Tracking/rendering deadlocked')
        finally:
            render_stop.set()
            render_thread.join(timeout=1.)
            if not b._thread.is_alive():
                b.stop()
        self.assertFalse(render_thread.is_alive())
        self.assertEqual(errors,[])
        self.assertIsNone(b.error)
        self.assertEqual(counts['samples'],90)
        self.assertGreater(counts['renders'],30)
        self.assertGreater(b.follower.frames,5)
        self.assertEqual(counts['closed'],1)
        self.assertFalse(hasattr(b.follower,'thread'))

    def test_six_directions_run_existing_sim_ik_then_joint_stream(self):
        for axis,sign in ((0,1),(0,-1),(1,1),(1,-1),(2,1),(2,-1)):
            with self.subTest(axis=axis,sign=sign):
                b,r,link=self.make()
                s=sample([0.,1.,0.],click=True)
                self.tick(b,r,s)
                s.axis_click=False
                s.grip=1.
                self.tick(b,r,s)
                start=b.arm.pos.copy()
                measured_start=b.arm.measured_tcp.copy()
                path=[]
                raw=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,1.,0.]])[axis]
                with patch.object(b.display_arm,'Movep_CANFD',wraps=b.display_arm.Movep_CANFD) as ik:
                    for k in range(75):
                        s.pos=np.array([0.,1.,0.])+raw*sign*.008*min(1.,k/20.)
                        s.stamp=time.time()
                        self.tick(b,r,s)
                        path.append(b.arm.measured_tcp-measured_start)
                    self.assertGreater(ik.call_count,50)
                delta=b.arm.pos-start
                direction=np.eye(3)[axis]*(sign if axis!=1 else -sign)
                along=float(delta@direction)
                lateral=np.linalg.norm(delta-along*direction)
                print(f'BRIDGE axis={axis} sign={sign}: model along={along*1000:.3f}mm lateral={lateral*1000:.3f}mm frames={len(link.sent)}',flush=True)
                self.assertGreater(along,.001)
                self.assertLess(lateral,.001)
                physical=np.asarray(path)
                physical_along=physical[-1]@direction
                max_cross=np.max(np.linalg.norm(physical-np.outer(physical@direction,direction),axis=1))
                print(f'  source physics along={physical_along*1000:.3f}mm max_cross={max_cross*1000:.3f}mm',flush=True)
                self.assertGreater(physical_along,.001)
                self.assertLess(physical_along,.0035)
                self.assertLess(max_cross,.0005)
                self.assertGreater(len(link.sent),5)
                self.assertTrue(all(x['command']=='movej_canfd' for x in link.sent))
                self.assertFalse(any(x['command']=='movep_canfd' for x in link.calls))
                s.grip=0.
                self.tick(b,r,s)
                self.assertFalse(b.follower.sent)

    def test_gripper_independent_trigger_release_gate_and_canonical_command(self):
        b,r,link=self.make()
        s=sample([0.,1.,0.])
        s.trigger=1.
        self.tick(b,r,s)
        self.assertFalse(any('gripper' in x['command'] for x in link.calls))
        s.trigger=0.
        self.tick(b,r,s)
        s.trigger=.6
        self.tick(b,r,s)
        commands=[x for x in link.calls if 'gripper' in x['command']]
        self.assertEqual(commands,[{'command':'set_gripper_position','position':400,'block':False}])
        self.assertEqual(link.sent,[])

    def test_straight_pose_up_does_not_turn_gravity_into_real_movement(self):
        b,r,link=self.make([0.]*6)
        s=sample([0.,1.,0.],click=True)
        self.tick(b,r,s)
        s.axis_click=False
        s.grip=1.
        self.tick(b,r,s)
        q=b.display_arm.q_deg.copy()
        for _ in range(20):
            s.pos=np.array([0.,1.008,0.])
            s.stamp=time.time()
            self.tick(b,r,s)
        np.testing.assert_allclose(b.display_arm.q_deg,q,atol=1e-5)
        self.assertEqual(link.sent,[])

    def test_subtree_compensation_does_not_change_table_or_objects(self):
        b,r,link=self.make()
        for name in ('link_1','link_6','adapter','d435i','eg2_finger_left'):
            self.assertEqual(b.arm.model.body_gravcomp[b.arm.model.body(name).id],1.)
        for name in ('table','wood_box_rm65'):
            self.assertEqual(b.arm.model.body_gravcomp[b.arm.model.body(name).id],0.)

    def test_stream_lock_released_even_if_socket_is_closed(self):
        from real.json_link import JsonArmLink
        b,r,link=self.make()
        transport=JsonArmLink(link._sock)
        with patch.object(link._sock,'gettimeout',side_effect=OSError('closed socket')):
            with self.assertRaises(OSError):
                transport.send_stream({'command':'movej_canfd','joint':[0]*6})
        self.assertTrue(transport._lock.acquire(blocking=False))
        transport._lock.release()

    def test_gripper_rpc_ignores_unrelated_motion_ack(self):
        from real.json_link import JsonArmLink
        b,r,link=self.make()
        link.follow=False
        transport=JsonArmLink(link._sock)
        responses=[{'command':'movej_canfd','state':True},
                   {'state':'current_trajectory_state','trajectory_state':True},
                   {'command':'set_gripper','state':True}]
        packet=('\r\n'.join(json.dumps(x) for x in responses)+'\r\n').encode()
        with patch.object(link._sock,'recv',create=True,return_value=packet):
            reply=transport.rpc({'command':'set_gripper_position','position':400,'block':False},timeout_s=.25)
        self.assertEqual(reply,responses[-1])
        self.assertEqual(transport.last_gripper_rpc['skipped'],responses[:-1])

    def test_hold_skips_queued_canfd_joint_state_before_its_ack(self):
        from real.json_link import JsonArmLink
        for accepted in (True,False):
            b,r,link=self.make()
            link.follow=False
            transport=JsonArmLink(link._sock)
            r.link=r.plant.link=transport
            r.policy._canfd_open=True
            b.follower.sent=True
            queued={'state':'joint_state','joint':[0]*6,'arm_err':0}
            ack={'command':'movej','receive_state':accepted}
            packet=(json.dumps(queued)+'\r\n'+json.dumps(ack)+'\r\n').encode()
            with patch.object(link._sock,'recv',create=True,return_value=packet):
                if accepted:
                    b.follower.hold()
                    self.assertFalse(b.follower.sent)
                else:
                    with self.assertRaisesRegex(RuntimeError,'hold unconfirmed'):
                        b.follower.hold()
                    self.assertTrue(b.follower.sent)
            self.assertEqual(transport.last_movej_rpc['reply'],ack)
            self.assertEqual(transport.last_movej_rpc['skipped'],[queued])

    def test_hold_accepts_completion_only_reply_seen_on_rm65_firmware(self):
        from real.json_link import JsonArmLink
        b,r,link=self.make()
        link.follow=False
        transport=JsonArmLink(link._sock)
        r.link=r.plant.link=transport
        r.policy._canfd_open=True
        b.follower.sent=True
        completion={'state':'current_trajectory_state','trajectory_state':True}
        packet=(json.dumps(completion)+'\r\n').encode()
        with patch.object(link._sock,'recv',create=True,return_value=packet):
            b.follower.hold()
        self.assertFalse(b.follower.sent)
        self.assertEqual(transport.last_movej_rpc['reply'],completion)
        self.assertEqual(transport.last_movej_rpc['skipped'],[])

    def test_bridge_uses_original_acknowledged_native_canfd_api(self):
        b,r,link=self.make()
        b.follower.submit(r.arm_state.get().joint+np.array([.04,0,0,0,0,0]),True)
        with patch.object(r,'Movej_CANFD',wraps=r.Movej_CANFD) as movej:
            b.follower.step()
        movej.assert_called_once()
        self.assertEqual(movej.call_args.kwargs,{})
        self.assertEqual(b.follower.frames,1)
        self.assertEqual(link.sent[-1]['command'],'movej_canfd')

    def test_no_measured_response_stops_without_increasing_joint_limit(self):
        b,r,link=self.make()
        link.follow=False
        b.follower.submit(r.arm_state.get().joint+.1,True)
        b.follower.step()
        self.assertEqual(b.follower.frames,1)
        b.follower.pending_since=time.monotonic()-2.
        with self.assertRaisesRegex(RuntimeError,'NO REAL JOINT RESPONSE'):
            b.follower.step()
        self.assertEqual(b.follower.frames,1)

    def test_delayed_feedback_does_not_pin_command_to_identical_point(self):
        b,r,link=self.make()
        link.follow=False
        measured=r.arm_state.get().joint.copy()
        b.follower.submit(measured+.4,True)
        goals=[]
        for _ in range(4):
            b.follower.step()
            goals.append(b.follower.last_goal.copy())
        np.testing.assert_allclose(np.array(goals)-measured,.4,atol=1e-8)
        self.assertEqual(len(link.sent),1,'unchanged target must not be resent every PICO tick')
        self.assertLessEqual(np.max(np.abs(goals[-1]-measured)),b.cfg['max_following_error_deg'])

    def test_reached_target_does_not_trigger_no_response_watchdog(self):
        b,r,link=self.make()
        b.follower.submit(r.arm_state.get().joint+.1,True)
        b.follower.step()
        r.fetch_arm_state()
        b.follower.pending_since=time.monotonic()-2.
        b.follower.step()
        self.assertIsNone(b.follower.pending_since)

    def test_gripper_rpc_timeout_records_unrelated_ack_not_false_success(self):
        from real.json_link import JsonArmLink
        b,r,link=self.make()
        link.follow=False
        transport=JsonArmLink(link._sock)
        packet=b'{"command":"movej_canfd","state":true}\r\n'
        with patch.object(link._sock,'recv',create=True,side_effect=[packet,TimeoutError('no gripper ACK')]):
            with self.assertRaises(TimeoutError):
                transport.rpc({'command':'set_gripper_position','position':400,'block':False},timeout_s=.25)
        self.assertIsNone(transport.last_gripper_rpc['reply'])
        self.assertIn('no gripper ACK',transport.last_gripper_rpc['error'])

    def test_gripper_rejection_and_timeout_preserved_without_retry(self):
        for reply in ({'command':'set_gripper','state':False},TimeoutError('missing ACK')):
            b,r,link=self.make()
            b.follower.submit_gripper(400)
            with patch.object(link,'rpc',side_effect=[reply] if isinstance(reply,Exception) else None,return_value=reply) as rpc:
                with self.assertRaisesRegex(RuntimeError,'no automatic retry'):
                    b.follower.step()
                self.assertEqual(rpc.call_count,1)
                self.assertFalse(r.last_gripper_command['accepted'])
                if isinstance(reply,Exception):
                    self.assertIn('TimeoutError',r.last_gripper_command['error'])
                else:
                    self.assertEqual(r.last_gripper_command['reply'],reply)
            self.assertIsNone(b.follower.grip_target)
            self.assertEqual(link.sent,[])

    def test_matching_echo_ack_supported_and_false_is_never_success(self):
        b,r,link=self.make()
        for accepted in (True,False):
            with patch.object(link,'rpc',return_value={'command':'set_gripper_position','state':accepted}):
                ret=r.Set_Gripper_Position(400,False,canonical=True)
            self.assertEqual(ret==0,accepted)

    def test_explicit_no_gripper_keeps_trigger_off_hardware(self):
        b,r,link=self.make()
        b.cfg['gripper_enabled']=False
        s=sample([0.,1.,0.])
        for trigger in (0.,1.,.5,0.):
            s.trigger=trigger
            self.tick(b,r,s)
        self.assertFalse(any('gripper' in x['command'] for x in link.calls))
        b.follower.submit(r.arm_state.get().joint+.04,True)
        b.follower.step()
        self.assertEqual(len(link.sent),1)

    def test_stale_input_clears_arm_and_gripper_queue(self):
        b,r,link=self.make()
        b.follower.submit(r.arm_state.get().joint+.05,True)
        b.follower.submit_gripper(500)
        s=sample([0.,1.,0.])
        s.stamp=time.time()-2.
        self.tick(b,r,s)
        self.assertEqual(link.sent,[])
        self.assertIsNone(b.follower.grip_target)
        self.assertFalse(any('gripper' in x['command'] for x in link.calls))

    def test_feedback_source_gap_and_invalid_target_block(self):
        for failure in ('feedback','source','gap','invalid'):
            b,r,link=self.make()
            q=r.arm_state.get().joint.copy()
            if failure=='invalid':
                with self.assertRaises(ValueError):
                    b.follower.submit([np.nan]*6,True)
                continue
            b.follower.submit(q+.05,True)
            if failure=='gap':
                b.follower.command_reference=q+3.
                b.follower.step()
                self.assertEqual(link.sent,[])
                self.assertEqual(len(b.follower.queue),1)
                continue
            if failure=='feedback':
                link.last_arm_state_received=time.monotonic()-1.
            if failure=='source':
                b.follower.target_time=time.monotonic()-1.
            with self.assertRaises(RuntimeError):
                b.follower.step()
            self.assertEqual(link.sent,[])

    def test_send_failure_and_hold_timeout_not_reported_as_success(self):
        b,r,link=self.make()
        b.follower.submit(r.arm_state.get().joint+.05,True)
        original_rpc=link.rpc
        def reject_canfd(payload,*,timeout_s=None):
            if payload['command']=='movej_canfd':
                return {'command':'movej_canfd','status':False}
            return original_rpc(payload,timeout_s=timeout_s)
        with patch.object(link,'rpc',side_effect=reject_canfd):
            with self.assertRaisesRegex(RuntimeError,'send failed'):
                b.follower.step()
        self.assertEqual(b.follower.frames,0)
        link.fail_hold=True
        b.follower.submit(None,False)
        with self.assertRaisesRegex(RuntimeError,'hold unconfirmed'):
            b.follower.step()
        self.assertTrue(b.follower.sent)


if __name__=='__main__':
    unittest.main()
