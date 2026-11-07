"""Offline supervisor checks; the transport integration uses loopback only."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PROJECT / "scripts"))
from bridge_mapping import JOINT_AXIS_SIGN
from bridge_supervisor import BridgeSupervisor


class FakeTransport:
    def __init__(self, raw_deg):
        self.joint = np.asarray(raw_deg, float) * JOINT_AXIS_SIGN
        self.stop_confirmed, self.sent, self.stop_pending = True, False, False
        self.stop_deadline = None
        self.operator_paused = self.protocol_pause = self.rearm_required = False
        self.fault = None
        self.stale = False
        self.targets = []
        self.cancellations = 0

    def state(self):
        if self.stale:
            raise RuntimeError("feedback stale")
        return SimpleNamespace(joint=self.joint.copy())

    def submit(self, target, active):
        assert active
        self.targets.append(np.asarray(target).copy())
        self.sent, self.stop_confirmed = True, False

    def cancel_input(self):
        self.cancellations += 1
        if self.sent:
            self.stop_pending = True
            self.rearm_required = True

    def acknowledge_release(self, value, valid):
        if not valid or value > .1 or not self.stop_confirmed or self.sent or self.stop_pending:
            return False
        self.rearm_required = False
        self.protocol_pause = None
        return True

    def finish_hold(self):
        self.stop_confirmed, self.sent, self.stop_pending = True, False, False


class SupervisorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        report = json.loads((HERE / "real_readonly_inventory.json").read_text(encoding="utf-8"))["responses"]
        cls.raw = np.asarray(report["get_current_arm_state"]["arm_state"]["joint"], float) / 1000
        cls.low = np.asarray(report["get_joint_min_pos"]["min_pos"], float) / 1000
        cls.high = np.asarray(report["get_joint_max_pos"]["max_pos"], float) / 1000
        cls.model = mujoco.MjModel.from_xml_path(str(HERE / "vr_teleop/example/scene/scene_rm65.xml"))

    def make(self, transport=None):
        t = transport or FakeTransport(self.raw)
        s = BridgeSupervisor(t, self.model, self.low, self.high, clutch_source="grip",
                             max_input_age_s=.25, activation_delta_deg=.02)
        return s, t

    def tick(self, supervisor, q=None, grip=0., **kw):
        args = dict(clutch_source="grip", clutch_value=grip, controller_age_s=.01,
                    head_age_s=.01, target_age_s=.01)
        args.update(kw)
        return supervisor.update(np.deg2rad(self.raw if q is None else q), **args)

    def arm_and_move(self):
        s, t = self.make()
        self.assertEqual(self.tick(s, grip=1).state, "WAIT_RELEASE")
        self.assertEqual(self.tick(s, grip=0).state, "READY")
        self.assertEqual(self.tick(s, grip=1).state, "ARMED")
        moved = self.raw.copy(); moved[1] += .1
        self.assertEqual(self.tick(s, moved, 1).state, "ACTIVE")
        return s, t, moved

    def test_release_hold_and_rearm(self):
        s, t, moved = self.arm_and_move()
        np.testing.assert_allclose(t.targets[-1], moved * JOINT_AXIS_SIGN)
        self.assertEqual(self.tick(s, moved, 0).reason, "clutch_released")
        self.assertEqual(self.tick(s, moved, 1).reason, "transport_not_ready")
        t.finish_hold()
        self.assertEqual(self.tick(s, moved, 1).state, "WAIT_RELEASE")
        self.assertEqual(self.tick(s, moved, 0).state, "READY")

    def test_head_controller_and_target_staleness_cancel(self):
        for field in ("head_age_s", "controller_age_s", "target_age_s"):
            with self.subTest(field=field):
                s, t, moved = self.arm_and_move()
                result = self.tick(s, moved, 1, **{field: .3})
                self.assertEqual(result.reason, "controller_or_head_stale")
                self.assertTrue(t.stop_pending)
                self.assertEqual(s.state, "WAIT_RELEASE")

    def test_protocol_pause_requires_confirmed_hold_and_fresh_release(self):
        s, t, moved = self.arm_and_move()
        t.protocol_pause = "synthetic controller error"
        self.assertEqual(self.tick(s, moved, 1).reason, "transport_not_ready")
        self.assertEqual(self.tick(s, moved, 0).state, "WAIT_RELEASE")
        t.finish_hold()
        self.assertEqual(self.tick(s, moved, 1).state, "WAIT_RELEASE")
        self.assertIsNotNone(t.protocol_pause)
        self.assertEqual(self.tick(s, moved, 0, head_age_s=.3).state, "WAIT_RELEASE")
        self.assertIsNotNone(t.protocol_pause)
        self.assertEqual(self.tick(s, moved, 0).state, "READY")
        self.assertIsNone(t.protocol_pause)
        self.assertEqual(self.tick(s, moved, 1).state, "ARMED")

    def test_stale_feedback_faults_and_cancels(self):
        s, t, moved = self.arm_and_move(); t.stale = True
        self.assertEqual(self.tick(s, moved, 1).state, "FAULT")
        self.assertTrue(t.stop_pending)
        self.assertEqual(self.tick(s, moved, 0).state, "FAULT")

    def test_wrong_clutch_source_and_unbounded_targets_fault(self):
        s, t = self.make()
        self.assertEqual(self.tick(s, clutch_source="trigger").reason, "unexpected_clutch_source")
        s, t = self.make(); self.tick(s); self.tick(s, grip=1)
        bad = self.raw.copy(); bad[0] = 200
        self.assertEqual(self.tick(s, bad, 1).state, "FAULT")
        self.assertFalse(t.targets)

    def test_malformed_input_faults_and_requests_cancel(self):
        s, t, moved = self.arm_and_move()
        result = self.tick(s, moved, 1, head_age_s=None)
        self.assertEqual(result.state, "FAULT")
        self.assertTrue(t.stop_pending)

    def test_cancel_failure_is_reported_and_latched(self):
        s, t, moved = self.arm_and_move()
        def fail_cancel(): raise RuntimeError("synthetic stop failure")
        t.cancel_input = fail_cancel
        result = self.tick(s, moved, 0)
        self.assertEqual(result.state, "FAULT")
        self.assertIn("cancel_failed", result.reason)
        self.assertEqual(self.tick(s, moved, 1).state, "FAULT")

    def test_preexisting_transport_fault_never_sends(self):
        s, t = self.make()
        t.fault = "synthetic transport fault"
        result = self.tick(s, grip=1)
        self.assertEqual(result.state, "FAULT")
        self.assertFalse(t.targets)

    def test_first_press_anchors_measured_pose_and_sender_handles_next_target(self):
        s, t = self.make(); self.tick(s)
        bad = self.raw.copy(); bad[0] += .5
        result = self.tick(s, bad, 1)
        self.assertEqual(result.reason, "clutch_anchored")
        np.testing.assert_allclose(result.sync_sim_q_rad, np.deg2rad(self.raw))
        self.assertFalse(t.targets)
        bad = self.raw.copy(); bad[0] += 1.1
        self.assertEqual(self.tick(s, bad, 1).state, "ACTIVE")
        self.assertEqual(len(t.targets), 1)

    def test_existing_transport_loopback_only(self):
        from test_latest_bridge import LatestTests
        helper = LatestTests(methodName="runTest")
        t, calls = helper.make()
        self.addCleanup(helper.doCleanups)
        s, _ = self.make(t)
        q = np.zeros(6)
        args = dict(clutch_source="grip", controller_age_s=.01, head_age_s=.01, target_age_s=.01)
        t.start()
        self.assertEqual(s.update(q, clutch_value=0, **args).state, "READY")
        self.assertEqual(s.update(q, clutch_value=1, **args).state, "ARMED")
        q[1] = np.deg2rad(-.2)
        for _ in range(8):
            self.assertEqual(s.update(q, clutch_value=1, **args).state, "ACTIVE")
            time.sleep(.02)
        self.assertTrue(any(c.get("command") == "movej_canfd" for c in calls))
        self.assertEqual(s.update(q, clutch_value=0, **args).state, "WAIT_RELEASE")
        deadline = time.monotonic() + 2
        while not t.stop_confirmed and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(t.stop_confirmed)
        self.assertTrue(any(c.get("command") == "movej" for c in calls))


if __name__ == "__main__":
    unittest.main()
