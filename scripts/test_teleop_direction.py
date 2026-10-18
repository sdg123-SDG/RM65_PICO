"""Headless checks of real MuJoCo camera axes, XR calibration and straight moves."""
from pathlib import Path
import tempfile
import time
import unittest

import mujoco
import numpy as np

from sim_control import PoseTeleop, SimArm, SCENE, quat_matrix, view_axes
from xr_direction_calibration import DirectionCalibration, calibrated_map, direction_basis
from xr_input import ControllerSample


def sample(pos, *, grip=0., button=False):
    return ControllerSample(np.asarray(pos, dtype=float), np.array([1., 0., 0., 0.]),
                            grip, 0., False, time.time(), True, primary_button=button)


class FakeArm:
    def __init__(self):
        self.pos = np.array([0.25, 0., 1.1])
        self.quat = np.array([1., 0., 0., 0.])
        self.target = None

    def hold(self):
        pass

    def gripper(self, value):
        pass

    def move(self, target, quat, speed):
        self.target = target.copy()


class DirectionTests(unittest.TestCase):
    def test_screen_axes_match_mujoco_camera(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        data = mujoco.MjData(model)
        scene = mujoco.MjvScene(model, maxgeom=500)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.distance, camera.elevation = 2., -18.
        for angle in [0., 45., 90., 145., 270.]:
            camera.azimuth = angle
            mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            f = scene.camera[0].forward.copy()
            r = np.cross(f, scene.camera[0].up)
            f[2] = 0.
            f /= np.linalg.norm(f)
            expected_f, expected_r = view_axes(angle)
            np.testing.assert_allclose(expected_f, f, atol=1e-6)
            np.testing.assert_allclose(expected_r, r, atol=1e-6)

    def test_room_yaw_ignores_vertical_drift(self):
        origin = np.array([0.25, 1.2, -0.3])
        canonical = np.column_stack([[0., 0., -1.], [1., 0., 0.], [0., 1., 0.]])
        for yaw in [-160., -30., 0., 75., 179.]:
            half = np.deg2rad(yaw) / 2.
            rotation = quat_matrix(np.array([np.cos(half), 0., np.sin(half), 0.]))
            raw_basis = rotation @ canonical
            # Forward capture contains ordinary hand-height drift, not a frame tilt.
            points = [origin, origin + 0.15 * raw_basis[:, 0] + [0., 0.04, 0.]]
            basis = direction_basis(points)
            for angle in [0., 90., 145.]:
                mapping = calibrated_map(basis, "view", angle)
                f, r = view_axes(angle)
                target = np.column_stack([f, r, [0., 0., 1.]])
                np.testing.assert_allclose(mapping @ raw_basis, target, atol=1e-12)

    def test_recorded_up_stroke_cannot_tilt_vertical(self):
        points = [np.array(p) for p in [
            [-0.025282193, -0.323484123, -0.177268922],
            [0.00117028784, -0.286647171, -0.317854077],
            [0.1915551, -0.280897856, -0.173121229],
            [-0.0303064175, -0.06032417, -0.251431823],
        ]]
        basis = direction_basis(points)
        mapping = calibrated_map(basis, "view", 145.)
        np.testing.assert_allclose(mapping @ [0., 0.10, 0.], [0., 0., 0.10], atol=1e-12)
        forward_stroke = points[1] - points[0]
        forward_stroke[1] = 0.
        mapped = mapping @ forward_stroke
        expected_f, expected_r = view_axes(145.)
        self.assertAlmostEqual(float(np.dot(mapped, expected_r)), 0., places=12)
        self.assertAlmostEqual(float(mapped[2]), 0., places=12)
        self.assertGreater(np.dot(mapped, expected_f), 0.14)

    def test_capture_rejects_bad_axes_and_accepts_two_taps(self):
        with tempfile.TemporaryDirectory() as folder:
            calibration = DirectionCalibration(Path(folder) / "calibration.json")
            points = [np.zeros(3), [0., 0.02, -0.15]]
            calibration.update(sample(points[0], grip=1., button=True))
            self.assertEqual(len(calibration.points), 0)
            calibration.update(sample(points[0]))
            for index, point in enumerate(points):
                calibration.update(sample(point, button=True))
                calibration.update(sample(point))
                self.assertEqual(calibration.ready, index == 1)
            self.assertTrue(calibration.report_path.exists())
            calibration.update(sample(points[0], button=True))
            self.assertFalse(calibration.ready)
            self.assertEqual(len(calibration.points), 1)
        with self.assertRaises(ValueError):
            direction_basis([np.zeros(3), np.array([0., 0.2, 0.])])

    def test_mapping_frozen_during_grip_and_release_reanchors(self):
        arm = FakeArm()
        control = PoseTeleop(arm, frame="view")
        control.set_direction_basis(np.eye(3))
        origin = np.array([0.2, 1.3, -0.5])
        control.update(sample(origin, grip=1.), 145.)
        target = arm.pos.copy()
        for axis in range(3):
            control.update(sample(origin + np.eye(3)[axis] * 0.03, grip=1.), 30.)
            expected = calibrated_map(np.eye(3), "view", 145.) @ (np.eye(3)[axis] * 0.03)
            np.testing.assert_allclose(arm.target - target, expected, atol=1e-12)
        control.update(sample(origin), 30.)
        control.update(sample(origin + 0.1, grip=1.), 30.)
        np.testing.assert_allclose(arm.target, arm.pos, atol=1e-12)

    def test_six_straight_mujoco_moves(self):
        f, r = view_axes(145.)
        for label, direction in [("forward", f), ("back", -f), ("right", r), ("left", -r),
                                 ("up", np.array([0., 0., 1.])), ("down", np.array([0., 0., -1.]))]:
            with self.subTest(direction=label):
                arm = SimArm()
                start, quat = arm.pos, arm.quat
                target = start + 0.02 * direction
                sideways = []
                for _ in range(100):
                    arm.move(target, quat, 0.06)
                    arm.step()
                    delta = arm.pos - start
                    sideways.append(np.linalg.norm(delta - np.dot(delta, direction) * direction))
                delta = arm.pos - start
                along = np.dot(delta, direction)
                lateral = np.linalg.norm(delta - along * direction)
                print(f"{label}: along={along*1000:.2f}mm lateral={lateral*1000:.2f}mm max_lateral={max(sideways)*1000:.2f}mm", flush=True)
                self.assertGreater(along, 0.017)
                self.assertLess(abs(along - 0.02), 0.003)
                self.assertLess(lateral, 0.001)
                self.assertLess(max(sideways), 0.002)
                arm.hold()
                held = arm.pos
                for _ in range(10):
                    arm.step()
                self.assertLess(np.linalg.norm(arm.pos - held), 0.0005)


if __name__ == "__main__":
    unittest.main(verbosity=2)
