"""Native EG2 scene/API integration; never initializes XR SDK or real-arm transport."""
import time
from pathlib import Path
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from launch_vdo_pico import bootstrap, check_backend, ROOT
from sim_control import PoseTeleop, CTRL_DT
from xr_input import ControllerSample


def sample(pos, grip=0., click=False):
    return ControllerSample(np.asarray(pos, dtype=float), np.array([1.,0.,0.,0.]),
                            grip, 0., False, time.time(), True, axis_click=click)


class NativePicoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            bootstrap()
        except FileNotFoundError:
            raise unittest.SkipTest("Adjacent private vdo source checkout is not installed")

    def make(self):
        box, native, arm = check_backend()
        arm._last_grip = 1000
        return native, arm

    def test_rate_limited_rotation_preserves_requested_tcp(self):
        from kinemetics.pose import Pose
        from sim_control import quat_matrix
        native, arm = self.make()
        arm.preserve_tip_on_rotation = True
        current, current_q = arm.command_world()
        tip = arm.tcp_pos.copy()
        desired_q = np.empty(4)
        mujoco.mju_mulQuat(desired_q, np.array([np.cos(.3),np.sin(.3),0.,0.]), current_q)
        wrist_target = tip - quat_matrix(desired_q) @ arm.tcp_offset
        with patch.object(native, 'Movep_CANFD', return_value=0) as send:
            arm.move(wrist_target, desired_q, .3)
        pose = send.call_args.args[0]
        actual_q = Pose.rpy_to_quat(pose.rx, pose.ry, pose.rz)
        base_tip = np.array([pose.px,pose.py,pose.pz]) + quat_matrix(actual_q) @ arm.tcp_offset
        actual_tip = arm.base_position + arm.base_rotation @ base_tip
        np.testing.assert_allclose(actual_tip, tip, atol=1e-6)

    def test_tcp_and_base_are_read_from_latest_model(self):
        native, arm = self.make()
        self.assertEqual(arm.tcp_site, "eg2_tcp")
        np.testing.assert_allclose(arm.tcp_offset, [0.,0.,.158164], atol=1e-6)
        np.testing.assert_allclose(arm.base_position, [0.,0.,.7], atol=1e-9)
        np.testing.assert_allclose(arm.tcp_pos, arm.measured_tcp, atol=1e-8)
        self.assertEqual(native.model.actuator("eg2_ctrl").ctrlrange[1], .07)

    def test_six_directions_use_native_movep_and_keep_wrist(self):
        for index, sign in ((0,1),(0,-1),(1,1),(1,-1),(2,1),(2,-1)):
            native, arm = self.make()
            # Let the native plant settle before measuring the incremental
            # physical path, rather than counting the initial gravity transient.
            for _ in range(500):
                mujoco.mj_step(native.model, native.data)
            mujoco.mj_forward(native.model, native.data)
            measured_start = arm.measured_tcp
            measured_path = []
            control = PoseTeleop(arm)
            s = sample([0.,1.,0.], click=True)
            control.update(s, 0.)
            s.axis_click, s.grip = False, 1.
            control.update(s, 0.)
            start, qstart = arm.command_world()
            original = native.Movep_CANFD
            with patch.object(native, "Movep_CANFD", wraps=original) as calls:
                # XR axes: -Z forward, X right, Y up.
                raw = np.array([[0.,0.,-1.],[1.,0.,0.],[0.,1.,0.]])[index]
                for k in range(110):
                    s.pos = np.array([0.,1.,0.]) + raw * sign * .02 * min(1., k/40.)
                    s.stamp = time.time()
                    control.update(s, 0.)
                    for _ in range(10):
                        mujoco.mj_step(native.model, native.data)
                    mujoco.mj_forward(native.model, native.data)
                    measured_path.append(arm.measured_tcp-measured_start)
                self.assertGreater(calls.call_count, 100)
            delta = arm.pos-start
            axis = np.eye(3)[index] * (sign if index != 1 else -sign)
            along = float(delta @ axis)
            lateral = float(np.linalg.norm(delta-along*axis))
            print(f"NATIVE axis={index} sign={sign} along={along*1000:.2f}mm lateral={lateral*1000:.2f}mm", flush=True)
            self.assertGreater(along, .015)
            self.assertLess(lateral, .003)
            measured = measured_path[-1]
            measured_along = float(measured @ axis)
            measured_lateral = float(np.linalg.norm(measured-measured_along*axis))
            path = np.asarray(measured_path)
            max_lateral = float(np.max(np.linalg.norm(path-np.outer(path@axis, axis), axis=1)))
            print(f"  PHYSICS along={measured_along*1000:.2f}mm lateral={measured_lateral*1000:.2f}mm max_lateral={max_lateral*1000:.2f}mm", flush=True)
            self.assertGreater(measured_along, .015)
            self.assertLess(measured_along, .025)
            self.assertLess(max_lateral, .006)
            dot = abs(float(arm.quat @ qstart))
            self.assertGreater(dot, .999)
            s.grip = 0.
            control.update(s, 0.)
            self.assertFalse(control.active)
            self.assertTrue(arm.holding)

    def test_gripper_uses_native_units_and_busy_does_not_write(self):
        native, arm = self.make()
        arm._last_grip = None
        with patch.object(native, "Set_Gripper_Position", return_value=0) as send:
            arm.gripper(0.)
            send.assert_called_with(1000, block=False)
            arm.gripper(1.)
            send.assert_called_with(0, block=False)
        q = native.q_cmd_deg.copy()
        native._busy = True
        arm.move(arm.pos+np.array([.01,0.,0.]), arm.quat, .06)
        self.assertNotEqual(arm.last_ret, 0)
        np.testing.assert_array_equal(native.q_cmd_deg, q)
        native._busy = False


if __name__ == "__main__":
    unittest.main()
