"""Exercise the vr_teleop loop and legacy sender against a localhost mock."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "vr_teleop"))
import example.teleop_sim as sim
import util.xrtoolkit_input as xr
import loopback_bridge as loopback


class Viewer:
    def __init__(self, count):
        self.remaining = count
        self.cam = SimpleNamespace(azimuth=0, elevation=0, distance=0, lookat=np.zeros(3))
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def is_running(self):
        self.remaining -= 1
        return self.remaining >= 0
    def sync(self): time.sleep(.02)


class SyntheticInput:
    def __init__(self, frames):
        self.frames, self.n, self.frame = frames, -1, None
    def poll(self):
        self.n += 1; self.frame = self.frames[self.n]
        return self.frame["controller"]
    def clutch_channel(self, source, threshold):
        return "grip" if self.frame["grip"] >= threshold else None
    def begin_clutch(self, axis):
        return {"forward_axis": axis, "reference_mode": "head_view", "robot_forward_in_pico": [0, 0, -1]}
    def end_clutch(self): pass
    def get_wrist_pose(self): return ((self.frame["x"], 0, 0), (0, 0, 0, 1))
    def get_pinch_distance(self): return .1
    def sample_status(self):
        return {"controller_valid": self.frame["controller"], "head_valid": self.frame["head"],
                "controller_age_s": .01 if self.frame["controller"] else float("inf"),
                "head_age_s": .01 if self.frame["head"] else float("inf"),
                "grip": self.frame["grip"], "trigger": 0.}
    def diagnostics(self): return "synthetic test input"
    def stop(self): pass


def frames(grip, head, count, start=0., controller=True):
    return [{"grip": grip, "head": head, "controller": controller, "x": start + i * .0005}
            for i in range(count)]


class SimulationLoopbackTests(unittest.TestCase):
    def run_trial(self, samples, viewer_delay=0.):
        with tempfile.TemporaryDirectory(prefix="vr-rm65-loopback-") as tmp:
            log = Path(tmp) / "decisions.jsonl"
            argv = ["teleop_sim.py", "--robot", "rm65", "--input-source", "xrtoolkit",
                    "--xrtoolkit-clutch", "grip", "--bridge-loopback-clutch", "grip",
                    "--position-scale", ".5", "--bridge-loopback-log", str(log)]
            def launch(model, data):
                time.sleep(viewer_delay)
                return Viewer(len(samples))
            with patch.object(sim.viewer, "launch_passive", launch), \
                 patch.object(xr, "XrToolkitPicoInput", lambda hand: SyntheticInput(samples)), \
                 patch.object(sys, "argv", argv):
                sim.main()
            rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(all(row["robot_connection"] is False for row in rows))
            return rows

    def test_full_simulation_to_localhost_sender_and_release(self):
        rows = self.run_trial(frames(0, True, 4) + frames(1, True, 16) + frames(0, True, 12))
        states = [row.get("state") for row in rows]
        self.assertIn("READY", states); self.assertIn("ARMED", states); self.assertIn("ACTIVE", states)
        self.assertIn("WAIT_RELEASE", states)
        self.assertGreater(max(row.get("loopback_frames", 0) for row in rows), 0)

    def test_regrip_after_extended_motion_anchors_again(self):
        samples = (frames(0, True, 8) + frames(1, True, 80) + frames(0, True, 25, .04) +
                   frames(1, True, 40, .04) + frames(0, True, 15, .06))
        rows = self.run_trial(samples)
        decisions = [r for r in rows if r.get("event") == "decision"]
        self.assertNotIn("startup_mismatch_resync_required", [r.get("reason") for r in decisions])
        active = [r for r in decisions if r.get("state") == "ACTIVE"]
        self.assertTrue(active)
        releases = [i for i, r in enumerate(decisions) if r.get("reason") == "clutch_released"]
        self.assertTrue(releases)
        self.assertIn("ACTIVE", [r.get("state") for r in decisions[releases[0] + 1:]])

    def test_one_bad_ik_frame_stops_then_release_and_regrip_recovers(self):
        samples = (frames(0, True, 8) + frames(1, True, 25) + frames(0, True, 25, .0125) +
                   frames(1, True, 25, .0125))
        original = sim.solve_pose_ik
        count = 0
        def fail_once(*args, **kwargs):
            nonlocal count
            count += 1
            if count == 10:
                return np.full_like(args[5], np.nan)
            return original(*args, **kwargs)
        with patch.object(sim, "solve_pose_ik", fail_once):
            rows = self.run_trial(samples)
        decisions = [r for r in rows if r.get("event") == "decision"]
        bad = next(i for i, r in enumerate(decisions) if r.get("reason") == "ik_invalid_or_rejected")
        self.assertEqual(decisions[bad]["state"], "WAIT_RELEASE")
        self.assertNotIn("FAULT", [r.get("state") for r in decisions])
        self.assertIn("ACTIVE", [r.get("state") for r in decisions[bad + 1:]])

    def test_slow_viewer_start_does_not_expire_synthetic_feedback(self):
        rows = self.run_trial(frames(0, True, 8), viewer_delay=.8)
        states = [row.get("state") for row in rows]
        self.assertIn("READY", states)
        self.assertNotIn("FAULT", states)

    def test_lost_head_requires_release_before_reengagement(self):
        samples = (frames(0, True, 4) + frames(1, True, 14) + frames(1, False, 5, .007) +
                   frames(1, True, 8, .01) + frames(0, True, 12) + frames(1, True, 12))
        rows = self.run_trial(samples)
        decisions = [(r.get("state"), r.get("reason")) for r in rows if r.get("event") == "decision"]
        stale = next(i for i, (_, reason) in enumerate(decisions) if reason == "controller_or_head_stale")
        ready = next(i for i in range(stale + 1, len(decisions)) if decisions[i][0] == "READY")
        self.assertNotIn("ACTIVE", [state for state, _ in decisions[stale:ready]])
        self.assertIn("ACTIVE", [state for state, _ in decisions[ready + 1:]])

    def test_lost_controller_requires_release_before_reengagement(self):
        samples = (frames(0, True, 4) + frames(1, True, 14) + frames(1, True, 5, .007, False) +
                   frames(1, True, 8, .01) + frames(0, True, 12) + frames(1, True, 12))
        rows = self.run_trial(samples)
        decisions = [(r.get("state"), r.get("reason")) for r in rows if r.get("event") == "decision"]
        stale = next(i for i, (_, reason) in enumerate(decisions) if reason == "controller_or_head_stale")
        ready = next(i for i in range(stale + 1, len(decisions)) if decisions[i][0] == "READY")
        self.assertNotIn("ACTIVE", [state for state, _ in decisions[stale:ready]])
        self.assertIn("ACTIVE", [state for state, _ in decisions[ready + 1:]])

    def test_viewer_exception_closes_loopback_worker(self):
        made = []
        class TrackingLoopback(loopback.LoopbackBridge):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                made.append(self)
        class FailingViewer(Viewer):
            def sync(self): raise RuntimeError("synthetic viewer failure")
        samples = frames(0, True, 4)
        with tempfile.TemporaryDirectory(prefix="vr-rm65-loopback-") as tmp:
            log = Path(tmp) / "decisions.jsonl"
            argv = ["teleop_sim.py", "--robot", "rm65", "--input-source", "xrtoolkit",
                    "--xrtoolkit-clutch", "grip", "--bridge-loopback-log", str(log)]
            with patch.object(sim.viewer, "launch_passive", lambda model, data: FailingViewer(len(samples))), \
                 patch.object(xr, "XrToolkitPicoInput", lambda hand: SyntheticInput(samples)), \
                 patch.object(loopback, "LoopbackBridge", TrackingLoopback), \
                 patch.object(sys, "argv", argv):
                with self.assertRaisesRegex(RuntimeError, "synthetic viewer failure"):
                    sim.main()
            self.assertTrue(made[0]._closed)
            self.assertFalse(made[0].transport.thread.is_alive())
            self.assertTrue(made[0].output.closed)

    def test_invalid_ik_stops_loopback_instead_of_reusing_old_target(self):
        samples = frames(0, True, 4) + frames(1, True, 8)
        with tempfile.TemporaryDirectory(prefix="vr-rm65-loopback-") as tmp:
            log = Path(tmp) / "decisions.jsonl"
            argv = ["teleop_sim.py", "--robot", "rm65", "--input-source", "xrtoolkit",
                    "--xrtoolkit-clutch", "grip", "--bridge-loopback-log", str(log)]
            with patch.object(sim.viewer, "launch_passive", lambda model, data: Viewer(len(samples))), \
                 patch.object(xr, "XrToolkitPicoInput", lambda hand: SyntheticInput(samples)), \
                 patch.object(sim, "solve_pose_ik", lambda *args, **kw: np.full_like(args[5], np.nan)), \
                 patch.object(sys, "argv", argv):
                sim.main()
            rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(any(row.get("reason") == "ik_invalid_or_rejected" for row in rows))
            self.assertNotIn("FAULT", [row.get("state") for row in rows])
            self.assertEqual(rows[-1]["event"], "end")
            self.assertTrue(all(row["robot_connection"] is False for row in rows))


if __name__ == "__main__":
    unittest.main()
