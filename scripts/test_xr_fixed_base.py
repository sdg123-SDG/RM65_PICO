"""Conformance with downloaded PiperXR heading code and input replay checks."""
import ast
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from sim_control import PoseTeleop, SimArm, quat_matrix
from test_teleop_direction import FakeArm, sample
from xr_heading import heading_basis, world_mapping
from xr_recording import ReplayInput, SessionRecorder, jsonable

ROOT = Path(__file__).resolve().parents[1]


def quat(yaw=0., pitch=0., roll=0.):
    # Compose OpenXR yaw(Y), pitch(X), roll(Z).
    def axis(angle, index):
        q = np.zeros(4)
        q[0], q[index] = np.cos(angle/2), np.sin(angle/2)
        return q
    q = axis(yaw, 2)
    tmp = np.empty(4)
    mujoco.mju_mulQuat(tmp, q, axis(pitch, 1))
    mujoco.mju_mulQuat(q, tmp, axis(roll, 3))
    return q


class FixedBaseTests(unittest.TestCase):
    def test_conforms_to_actual_piperxr_capture_function(self):
        source = ROOT / "refs/PiperXR/piper_xr/common/pose_mapping.py"
        if not source.is_file():
            self.skipTest("Optional pinned PiperXR checkout absent; see docs/DEVELOPMENT.md")
        tree = ast.parse(source.read_text(encoding="utf-8"))
        selected = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in ("_shortest_arc_matrix", "_as_proper_rotation")]
        klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CorrectedPoseMixin")
        klass.body = [node for node in klass.body if isinstance(node, ast.FunctionDef) and node.name == "_capture_yaw_align"]
        def matrix(q):
            m = np.eye(4)
            m[:3,:3] = quat_matrix(q)
            return m
        def rotation(angle, axis):
            axis = np.asarray(axis) / np.linalg.norm(axis)
            return matrix(np.r_[np.cos(angle/2), axis*np.sin(angle/2)])
        ns = {"np": np, "tf": SimpleNamespace(quaternion_matrix=matrix, rotation_matrix=rotation)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=selected+[klass], type_ignores=[])), str(source), "exec"), ns)
        fixed = np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
        ref = ns["CorrectedPoseMixin"]()
        ref.enable_yaw_align = True
        ref.R_headset_world = fixed
        for yaw in np.deg2rad([-170., -90., 0., 45., 179.]):
            for pitch in [-.6, 0., .6]:
                q = quat(yaw, pitch, .3)
                pose = np.r_[np.zeros(3), q[1:], q[0]]
                ref.xr_client = SimpleNamespace(get_pose_by_name=lambda name: pose)
                ref._capture_yaw_align("left")
                reference = fixed @ ref._yaw_align["left"]
                ours = world_mapping(heading_basis(q), np.eye(3))
                # Upstream shortest-arc adds eps to norms; near 180 degrees
                # that produces a slightly nonorthogonal result. Compare the
                # closest rotation, not its incidental scale error.
                u, _, vt = np.linalg.svd(reference)
                np.testing.assert_allclose(ours, u @ vt, atol=1e-5)
                np.testing.assert_allclose(ours @ [0.,1.,0.], [0.,0.,1.], atol=1e-12)

    def test_missing_head_blocks_until_explicit_stick_reset(self):
        arm = FakeArm()
        control = PoseTeleop(arm)
        s = sample([.1,1.,.2], grip=1.)
        control.update(s, 145.)
        self.assertFalse(control.active)
        self.assertIsNone(arm.target)
        s.grip, s.axis_click = 0., True
        control.update(s, 145.)
        s.grip, s.axis_click = 1., False
        control.update(s, 145.)
        self.assertTrue(control.active)
        self.assertEqual(control.heading_source, "manual forward")

    def test_frozen_head_and_camera_with_rotated_base(self):
        arm = FakeArm()
        arm.base_rotation = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        control = PoseTeleop(arm)
        origin = np.array([.2,1.2,-.3])
        s = sample(origin, grip=1.)
        s.headset_quat_wxyz = quat(.8,.5,.2)
        basis = heading_basis(s.headset_quat_wxyz)
        control.update(s, 145.)
        mapping = control.mapping.copy()
        s = sample(origin + basis[:,0]*.03, grip=1.)
        s.headset_quat_wxyz = quat(-1.2,-.4,-.2)
        control.update(s, -70.)
        np.testing.assert_allclose(control.mapping, mapping, atol=1e-12)
        np.testing.assert_allclose(arm.target-arm.pos, arm.base_rotation @ [.03,0.,0.], atol=1e-12)

    def test_record_replay_preserves_head_and_disarms_eof_and_gaps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"roundtrip.jsonl"
            arm = SimArm()
            control = PoseTeleop(arm)
            s = sample([.1,1.,.2], grip=1.)
            s.headset_quat_wxyz = quat(.7,.2)
            s.headset_stamp = time.time()
            control.update(s, 0.)
            recorder = SessionRecorder(path, {}, source="synthetic-test")
            recorder.write(s, control, arm)
            recorder.close()
            replay = ReplayInput(path)
            restored = replay.sample_at(0.)
            self.assertTrue(restored.valid)
            np.testing.assert_allclose(restored.headset_quat_wxyz, s.headset_quat_wxyz)
            stopped = replay.sample_at(.5)
            self.assertFalse(stopped.valid)
            self.assertEqual(stopped.grip, 0.)
            self.assertTrue(replay.finished)
            self.assertEqual(replay.metadata["source"], "synthetic-test")

    def test_pose_rotation_is_about_tcp(self):
        arm = SimArm()
        control = PoseTeleop(arm, mode="pose")
        s = sample([.1,1.,.2], grip=1.)
        s.headset_quat_wxyz = quat()
        control.update(s,0.)
        tcp = control.target_tcp.copy()
        s.quat_wxyz = quat(0., .1)
        s.stamp = time.time()
        control.update(s,90.)
        np.testing.assert_allclose(control.target_tcp, tcp, atol=1e-12)
        # Full physics tracking is validated separately by the six-axis test.
        self.assertGreater(np.linalg.norm(control.target_world-control.ref_ee), .005)


if __name__ == "__main__":
    unittest.main(verbosity=2)
