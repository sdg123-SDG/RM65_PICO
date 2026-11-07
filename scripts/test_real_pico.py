"""Fake controller transport only: no robot IP, SDK, or real guard process."""
import copy
import json
import os
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from launch_vdo_pico import ROOT, bootstrap
from real_pico import RealPicoAdapter, RealPicoServo, RealPicoFault, validate_settings, GuardedLink
from real_pico_guard import pid_alive


class FakeSocket:
    def __init__(self, owner):
        self.owner = owner
        self.timeout = .1
        self.fail_send = False

    def settimeout(self, value): self.timeout = value
    def gettimeout(self): return self.timeout
    def getpeername(self): return ("fake-controller",8080)
    def shutdown(self, *args): self.owner.closed = True
    def sendall(self, line):
        if self.fail_send:
            raise TimeoutError("fake send timed out")
        data = json.loads(line)
        self.owner.sent.append(data)
        if self.owner.follow:
            self.owner.wire_deg = np.asarray(data["joint"],float)/1000.


class FakeLink:
    def __init__(self, wire_deg):
        self.wire_deg = np.array(wire_deg)
        self.sent = []
        self.calls = []
        self.closed = False
        self.follow = True
        self.fail_read = False
        self.fail_hold = False
        self._sock = FakeSocket(self)
        self._lock = threading.Lock()

    def rpc(self, payload, *, timeout_s=None):
        with self._lock:
            self.calls.append(copy.deepcopy(payload))
            cmd = payload["command"]
            if cmd == "get_current_arm_state":
                if self.fail_read:
                    raise TimeoutError("fake state timeout")
                from kinemetics.pose import Pose
                fk=self.kin.fk_at(np.deg2rad(self.wire_deg))
                pose=np.r_[np.rint(fk.pos*1e6),np.rint(Pose.quat_to_rpy(fk.quat)*1000)].astype(int).tolist()
                return {"state":"current_arm_state", "arm_state":{
                    "joint":np.rint(self.wire_deg*1000).astype(int).tolist(),
                    "pose":pose, "arm_err":0, "sys_err":0}}
            if cmd == "movej" and self.fail_hold:
                raise TimeoutError("fake hold timeout")
            return {"command":cmd, cmd:True}

    def close(self):
        self.closed = True


class RealPicoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            bootstrap()
        except FileNotFoundError:
            raise unittest.SkipTest("Private VDO source not installed")

    def make(self,joint_deg=None):
        from simu.box_scene import BoxScene, apply_home_keyframe
        from real.real_arm import RealArm
        box = BoxScene(camera="d435i", gravcomp=False)
        model = mujoco.MjModel.from_xml_path(str(box.scene_xml))
        data = mujoco.MjData(model)
        apply_home_keyframe(model,data)
        initial = np.array([data.qpos[model.jnt_qposadr[model.joint(f"joint_{i}").id]] for i in range(1,7)])
        link = FakeLink(np.rad2deg(initial) if joint_deg is None else joint_deg)
        native = RealArm(link, xml_path=box.scene_xml, model=model, data=data)
        link.kin=native.kin
        native.fetch_arm_state()
        cfg = json.loads((ROOT/"config/real_pico.example.json").read_text())
        inventory={'responses':{
            'get_arm_software_version':{'Ctrl_version':0x6b0400},
            'get_current_arm_state':link.rpc({'command':'get_current_arm_state'}),
            'get_current_tool_frame':{'tool_name':'Arm_Tip','pose':[0]*6},
            'get_current_work_frame':{'frame_name':'World','pose':[0]*6},
            'get_joint_min_pos':{'min_pos':np.rint(np.rad2deg(native.kin.q_min)*1000).astype(int).tolist()},
            'get_joint_max_pos':{'max_pos':np.rint(np.rad2deg(native.kin.q_max)*1000).astype(int).tolist()},
            'get_joint_en_state':{'en_state':[1]*6},
            'get_joint_err_flag':{'err_flag':[0]*6},
            'get_collision_stage':{'collision_stage':0}}}
        adapter = RealPicoAdapter(native,cfg,inventory=inventory)
        return native,adapter,link

    def test_construct_and_disarmed_inputs_send_no_motion(self):
        native, arm, link = self.make()
        arm.move(arm.pos+np.array([.005,0.,0.]),arm.quat,.02)
        arm.gripper(1.)
        arm.hold()
        self.assertFalse(arm.armed)
        self.assertEqual(link.sent,[])
        self.assertTrue(all(x["command"]=="get_current_arm_state" for x in link.calls))

    def test_repeated_arm_cannot_reset_session_boundary(self):
        native, arm, link = self.make()
        arm.arm()
        anchor = arm.anchor_tcp.copy()
        arm.sent = True
        with self.assertRaises(RealPicoFault):
            arm.arm()
        np.testing.assert_array_equal(arm.anchor_tcp, anchor)
        self.assertTrue(arm.sent)

    def test_bounded_joint_stream_uses_verified_physical_angles(self):
        native,arm,link = self.make()
        arm.arm()
        initial = arm.pos.copy()
        previous = arm.shadow.q_cmd_deg.copy()
        previous_tcp = arm.tcp_pos.copy()
        for _ in range(35):
            native.fetch_arm_state()
            arm.move(initial+np.array([.005,0.,0.]),arm.quat,.02)
            self.assertEqual(arm.last_ret,0)
            frame = link.sent[-1]
            expected = np.rint(arm.shadow.q_cmd_deg*1000).astype(int).tolist()
            self.assertEqual(frame["joint"],expected)
            self.assertFalse(frame["follow"])
            self.assertLessEqual(np.max(np.abs(arm.shadow.q_cmd_deg-previous)), .10001)
            self.assertLessEqual(np.linalg.norm(arm.tcp_pos-previous_tcp), .000401)
            previous = arm.shadow.q_cmd_deg.copy()
            previous_tcp = arm.tcp_pos.copy()
        self.assertGreater(arm.pos[0]-initial[0],.001)

    def test_stale_feedback_blocks_next_frame(self):
        native,arm,link = self.make()
        arm.arm()
        arm.state.received_at = time.monotonic()-1.
        with self.assertRaisesRegex(RealPicoFault,"stale"):
            arm.move(arm.pos+np.array([.005,0.,0.]),arm.quat,.02)
        self.assertEqual(link.sent,[])

    def test_following_error_and_workspace_block(self):
        native,arm,link = self.make()
        arm.arm()
        target = arm.pos+np.array([.2,0.,0.])
        with self.assertRaisesRegex(RealPicoFault,"displacement|workspace"):
            arm.move(target,arm.quat,.02)
        q = arm.shadow.q_cmd_deg.copy()
        arm.shadow.set_q(np.deg2rad(q+np.array([3.,0,0,0,0,0])))
        with self.assertRaisesRegex(RealPicoFault,"following"):
            arm.move(arm.pos,arm.quat,.02)
        self.assertEqual(link.sent,[])

    def test_send_timeout_is_not_success(self):
        native,arm,link = self.make()
        arm.arm()
        link._sock.fail_send = True
        with self.assertRaisesRegex(RealPicoFault,"send failed"):
            arm.move(arm.pos+np.array([.005,0.,0.]),arm.quat,.02)
        self.assertEqual(link.sent,[])

    def test_controller_error_and_control_gap_block(self):
        native,arm,link = self.make()
        arm.arm()
        st = native.arm_state.get()
        st.arm_err = 1
        with self.assertRaisesRegex(RealPicoFault,"Controller error"):
            arm.move(arm.pos,arm.quat,.02)
        st.arm_err = 0
        native.arm_state.put(st)
        arm.last_send = time.monotonic()-1.
        with self.assertRaisesRegex(RealPicoFault,"gap"):
            arm.move(arm.pos,arm.quat,.02)
        self.assertEqual(link.sent,[])

    def test_release_hold_uses_measured_joints_and_timeout_latches(self):
        native,arm,link = self.make()
        arm.arm()
        arm.move(arm.pos+np.array([.005,0.,0.]),arm.quat,.02)
        native.fetch_arm_state()
        latest_wire = np.rint(link.wire_deg*1000).astype(int).tolist()
        arm.hold()
        hold = [x for x in link.calls if x["command"]=="movej"][-1]
        self.assertEqual(hold["joint"],latest_wire)
        self.assertEqual(hold["v"],10)
        arm.sent = True
        native.policy._canfd_open = True
        link.fail_hold = True
        arm.hold()
        self.assertIsNotNone(arm.fault)
        self.assertTrue(link.closed)

    def test_other_writers_blocked_when_armed(self):
        native,arm,link = self.make()
        arm.arm()
        before = len(link.calls)
        with self.assertRaisesRegex(RealPicoFault,"other motion"):
            arm.transport.rpc({"command":"movej","joint":[0]*6})
        self.assertEqual(len(link.calls),before)
        self.assertIsNotNone(arm.transport.violation)

    def test_real_check_never_connects(self):
        import launch_vdo_pico_real as launch
        with patch("socket.create_connection",side_effect=AssertionError("no sockets in check")):
            with patch("sys.argv",["launch_vdo_pico_real.py","--check"]):
                self.assertEqual(launch.main(),0)

    def test_windows_pid_query_does_not_call_os_kill(self):
        if os.name != "nt":
            self.skipTest("Windows-specific regression")
        with patch("os.kill",side_effect=AssertionError("would terminate process on Windows")):
            self.assertTrue(pid_alive(os.getpid()))

    def test_cached_world_publish_cannot_refresh_network_timestamp(self):
        native,arm,link=self.make()
        state=arm.state.get()
        arm.state.received_at=time.monotonic()-1.
        arm.state.put(state)  # exactly what the native world loop does
        with self.assertRaisesRegex(RealPicoFault,'stale'):
            arm.checked_state()
        native.fetch_arm_state()
        self.assertIsNotNone(arm.checked_state())

    def servo(self, native, arm):
        with patch('real_pico.RealPicoAdapter',return_value=arm):
            servo=RealPicoServo(native,arm.cfg)
        servo.control.direction_basis=np.eye(3)
        servo.enabled=True
        return servo

    def hand(self, grip=0., valid=True, menu=False, trigger=0.):
        from xr_input import ControllerSample
        return ControllerSample(np.zeros(3),np.array([1.,0.,0.,0.]),grip,trigger,menu,time.time(),valid)

    def test_grip_requires_release_then_press_without_cli_arm(self):
        native,arm,link=self.make()
        servo=self.servo(native,arm)
        servo.prepare_sample(self.hand(1.))
        self.assertFalse(arm.armed)
        servo.prepare_sample(self.hand())
        servo._released_since=time.monotonic()-.31
        servo.prepare_sample(self.hand())
        self.assertTrue(servo._grip_ready)
        servo.prepare_sample(self.hand(1.))
        self.assertTrue(arm.armed)
        arm.move(arm.pos,arm.quat,.01)
        self.assertEqual(link.sent,[])
        servo.prepare_sample(self.hand())
        self.assertFalse(arm.armed)

    def test_stream_loss_requires_new_heading_and_release(self):
        native,arm,link=self.make()
        servo=self.servo(native,arm)
        arm.arm()
        servo.prepare_sample(self.hand(1.,valid=False))
        self.assertFalse(arm.armed)
        self.assertIsNone(servo.control.direction_basis)
        servo.prepare_sample(self.hand(1.))
        self.assertFalse(arm.armed)
        self.assertEqual(link.sent,[])

    def test_manual_disarm_cannot_auto_rearm_on_grip(self):
        native,arm,link=self.make()
        servo=self.servo(native,arm)
        servo.handle_cli(['disarm'])
        servo.prepare_sample(self.hand())
        servo._released_since=time.monotonic()-.31
        servo.prepare_sample(self.hand())
        servo.prepare_sample(self.hand(1.))
        self.assertFalse(arm.armed)
        self.assertEqual(link.sent,[])

    def test_flange_model_mismatch_blocks_before_any_send(self):
        native,arm,link=self.make()
        state=arm.state.get()
        state.pose.px+=.01
        with self.assertRaisesRegex(RealPicoFault,'FK mismatch'):
            arm.arm()
        self.assertEqual(link.sent,[])

    def test_real_snapshot_physical_angles_match_better_than_legacy(self):
        from real_robot_profile import hardware_profile,check_fk
        native,arm,link=self.make()
        # Read-only 2026-10-19 snapshot, not synthetic FK-generated pose.
        q=np.array([-.001,-.119,.270,-.090,.192,10.783])
        pose=[-882,0,850396,1,5,-2954]
        result=check_fk(native.kin,q,pose,arm.hardware)
        self.assertLess(result['position_error_mm'],.12)
        with self.assertRaisesRegex(ValueError,'FK mismatch'):
            check_fk(native.kin,q*np.array([1,-1,-1,1,-1,1]),pose,arm.hardware)

    def test_regripping_cannot_walk_startup_boundary(self):
        native,arm,link=self.make()
        arm.anchor_tcp=arm.startup_tcp+np.array([.02,0.,0.])
        with self.assertRaisesRegex(RealPicoFault,'Startup displacement'):
            arm.check_workspace(arm.startup_tcp+np.array([.04,0.,0.]))

    def test_probe_has_hard_read_only_allowlist(self):
        from probe_real_robot import ReadOnlyClient
        class NoSend:
            def sendall(self,*args):
                raise AssertionError('must not send')
        with self.assertRaisesRegex(ValueError,'not allowed'):
            ReadOnlyClient(NoSend()).query('movej')

    def test_native_stop_is_never_blocked_by_writer_ownership(self):
        native,arm,link=self.make()
        arm.arm()
        arm.transport.rpc({'command':'set_arm_stop'})
        self.assertEqual(link.calls[-1]['command'],'set_arm_stop')
        self.assertIsNotNone(arm.transport.violation)

    def test_forward_minus_x_rotates_horizontal_axes_not_vertical(self):
        from xr_heading import world_mapping,heading_basis
        basis=heading_basis(np.array([1.,0.,0.,0.]))
        mapping=world_mapping(basis,np.eye(3),180.)
        np.testing.assert_allclose(mapping@basis,np.diag([-1.,1.,1.]),atol=1e-10)

    def test_trigger_held_at_grip_press_requires_new_release(self):
        native,arm,link=self.make()
        servo=self.servo(native,arm)
        servo.prepare_sample(self.hand())
        servo._released_since=time.monotonic()-.31
        servo.prepare_sample(self.hand())
        servo.prepare_sample(self.hand(1.,trigger=1.))
        servo.prepare_sample(self.hand(1.))
        self.assertFalse(arm.armed)

    def test_unknown_gripper_is_open_preview_without_real_write(self):
        native,arm,link=self.make()
        self.assertAlmostEqual(native._grip_width_m(),.07,places=6)
        self.assertIn('VISUAL PREVIEW ONLY',arm.gripper_label)
        self.assertIsNone(arm.gripper_feedback)
        self.assertEqual(link.sent,[])
        self.assertTrue(all(x['command']=='get_current_arm_state' for x in link.calls))

    def test_measured_gripper_opening_is_pinned_without_actuation(self):
        native,arm,link=self.make()
        before=len(link.calls)
        arm.update_gripper_visual({'enable':1,'status':1,'error':0,'actpos':600})
        self.assertAlmostEqual(native._grip_width_m(),.042,places=6)
        self.assertIn('MEASURED',arm.gripper_label)
        self.assertEqual(len(link.calls),before)
        self.assertEqual(link.sent,[])

    def test_captured_straight_pose_up_is_blocked_without_send(self):
        native,arm,link=self.make([.006,-.112,.277,-.087,.185,10.780])
        arm.arm()
        self.assertTrue(arm.posture['near_singular'])
        arm.move(arm.pos+np.array([0.,0.,.0028]),arm.quat,.01)
        self.assertNotEqual(arm.last_ret,0)
        self.assertIn('IK BLOCKED',arm.motion_reason)
        self.assertEqual(link.sent,[])

    def test_unconverged_ik_result_cannot_leak_a_frame(self):
        from types import SimpleNamespace
        native,arm,link=self.make()
        arm.arm()
        result=SimpleNamespace(q=np.deg2rad(arm.shadow.q_cmd_deg+.01),success=False,pos_err=.0002)
        with patch.object(arm.shadow.kin,'solve_isolated',return_value=result):
            arm.move(arm.pos+np.array([.002,0.,0.]),arm.quat,.01)
        self.assertEqual(link.sent,[])
        self.assertEqual(arm.frames_sent,0)

    def test_regular_pose_six_directions_are_real_frames_not_just_targets(self):
        for axis in range(3):
            for sign in (-1,1):
                with self.subTest(axis=axis,sign=sign):
                    native,arm,link=self.make()
                    arm.arm()
                    start=arm.pos.copy()
                    delta=np.zeros(3);delta[axis]=sign*.002
                    for _ in range(20):
                        native.fetch_arm_state()
                        arm.move(start+delta,arm.quat,.01)
                        self.assertEqual(arm.last_ret,0,arm.motion_reason)
                    native.fetch_arm_state()
                    measured=arm.measured_tcp
                    self.assertGreater(arm.frames_sent,0)
                    self.assertLess(np.linalg.norm(arm.pos-start-delta),.00006)
                    self.assertLess(np.linalg.norm(measured-arm.pos),.00002)

    def test_worker_persists_failure_before_requesting_shutdown(self):
        from types import SimpleNamespace
        native,arm,link=self.make()
        servo=self.servo(native,arm)
        events=[]
        def broken_sample():
            raise TimeoutError('injected input error')
        servo.source=SimpleNamespace(sample=broken_sample,close=lambda:events.append('source_close'))
        servo.recorder=SimpleNamespace(event=lambda *args:events.append('error_event'),close=lambda:events.append('recorder_close'))
        with patch.object(servo,'on_error',side_effect=lambda exc:events.append('persist')):
            with patch('vdo_pico._thread.interrupt_main',side_effect=lambda:events.append('interrupt')):
                servo._loop()
        self.assertEqual(events[:3],['persist','error_event','interrupt'])
        self.assertEqual(servo.error,'injected input error')
        self.assertEqual(link.sent,[])


if __name__ == "__main__":
    unittest.main()
