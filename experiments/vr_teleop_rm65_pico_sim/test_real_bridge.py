"""Exercise the real bridge factory against localhost only, never hardware."""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from real_bridge import RealBridge, gripper_command_for_trigger, trigger_to_gripper_position
from latest_bridge import LatestTransport


def wait_guard_log_closed(path):
    """Windows cannot remove an open guard log; wait for the child to exit."""
    renamed = path.with_name(path.name + ".closed")
    deadline = time.monotonic() + 5
    while True:
        try:
            path.rename(renamed)
            renamed.rename(path)
            return
        except PermissionError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.05)


class LocalController:
    def __init__(self, *, arm_error=0):
        self.report = json.loads((HERE / "real_readonly_inventory.json").read_text(encoding="utf-8"))["responses"]
        self.report["get_current_arm_state"]["arm_state"]["arm_err"] = arm_error
        self.joint = self.report["get_current_arm_state"]["arm_state"]["joint"][:]
        self.calls = []
        self.drop_feedback = False
        self.drop_gripper_ack = False
        self.inject_protocol_error = threading.Event()
        self.done = threading.Event()
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0)); self.listener.listen(1)
        host, port = self.listener.getsockname()
        self.address = f"{host}:{port}"
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            conn, peer = self.listener.accept()
        except OSError:
            return
        assert peer[0] == "127.0.0.1"
        conn.settimeout(.05); buf = b""
        try:
            while not self.done.is_set():
                try: chunk = conn.recv(8192)
                except socket.timeout: continue
                if not chunk: break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    msg = json.loads(line); command = msg["command"]; self.calls.append(msg)
                    if self.inject_protocol_error.is_set():
                        self.inject_protocol_error.clear()
                        conn.sendall(b"1, string or '}' expected near '{'\r\n")
                    reply = None
                    if command == "get_arm_software_info":
                        reply = {"state": "arm_software_info", "Ctrl_version": 7014144}
                    elif command == "get_gripper_state":
                        reply = {"state": "gripper_state", "enable": 0}
                    elif command == "get_current_arm_state":
                        if not self.drop_feedback:
                            reply = copy.deepcopy(self.report[command]); reply["arm_state"]["joint"] = self.joint[:]
                    elif command in self.report:
                        reply = self.report[command]
                    elif command == "movej_canfd":
                        self.joint = msg["joint"][:]
                        if not self.drop_feedback:
                            reply = {"state": "joint_state", "joint": self.joint, "arm_err": 0}
                    elif command == "movej":
                        self.joint = msg["joint"][:]
                        if not self.drop_feedback:
                            reply = {"command": "movej", "receive_state": True}
                    elif command in ("set_gripper_position", "set_gripper_release", "set_gripper_pick", "set_gripper_pick_on"):
                        if not self.drop_gripper_ack:
                            reply = {"command": "set_gripper", "state": True}
                    if reply is not None:
                        conn.sendall((json.dumps(reply) + "\r\n").encode())
        except OSError:
            pass
        finally:
            conn.close()

    def close(self):
        self.done.set(); self.listener.close(); self.thread.join(1)


class FakeGuard:
    def __init__(self, arm, address, *, fail=False):
        self.installed = self.closed = False; self.fail = fail
    def install(self):
        if self.fail: raise RuntimeError("synthetic guard failure")
        self.installed = True
    def close(self): self.closed = True


class RealBridgeLoopbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(HERE / "vr_teleop/example/scene/scene_rm65.xml"))

    def test_interleaved_gripper_ack_is_recovered_without_losing_state(self):
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close); self.addCleanup(receiver.close)
        t = LatestTransport.__new__(LatestTransport)
        t._sock = receiver; t.buf = bytearray(); t.lock = threading.Lock()
        t.gripper_inflight = ({'command': 'set_gripper_release', 'speed': 500}, time.monotonic())
        t.gripper_error = None; t.gripper_target = None
        t.pending_query = ('get_current_arm_state', time.monotonic())
        t.protocol_errors = []; t.protocol_pause = None
        t.hold_accepted = False; t.hold_target = None
        events = []; feedback = []
        t.record = lambda event, **fields: events.append((event, fields))
        t._publish_feedback = lambda state: feedback.append(state)
        state = json.dumps({'state': 'current_arm_state', 'arm_state': {
            'joint': [-6421, -29915, -31696, 826, -96679, -4906],
            'pose': [363575, -38845, 428548, 3121, -378, -18], 'arm_err': 0, 'sys_err': 0}})
        cut = state.index('[', state.index('"joint"')) + 1
        wire = state[:cut] + json.dumps({'command': 'set_gripper', 'state': True}) + '\r\n' + state[cut:] + '\r\n'
        sender.sendall(wire.encode())
        t._read()
        self.assertIsNone(t.gripper_inflight)
        self.assertIsNone(t.gripper_error)
        self.assertIsNone(t.pending_query)
        self.assertEqual(len(feedback), 1)
        self.assertIn('interleaved_gripper_ack_recovered', [event for event, _ in events])
        self.assertIsNone(t.protocol_pause)

    def test_interleaved_gripper_ack_in_joint_feedback(self):
        from real.json_codec import decode_arm_state
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close); self.addCleanup(receiver.close)
        t = LatestTransport.__new__(LatestTransport)
        t._sock = receiver; t.buf = bytearray(); t.lock = threading.Lock()
        t.gripper_inflight = ({'command': 'set_gripper_release', 'speed': 500}, time.monotonic())
        t.gripper_error = None; t.gripper_target = None; t.pending_query = None
        t.protocol_errors = []; t.protocol_pause = None
        events = []; feedback = []
        t.record = lambda event, **fields: events.append((event, fields))
        t._publish_feedback = lambda state: feedback.append(state)
        cached = decode_arm_state({'state': 'current_arm_state', 'arm_state': {
            'joint': [0, 0, 0, 0, 0, 0], 'pose': [0, 0, 0, 0, 0, 0], 'arm_err': 0, 'sys_err': 0}})
        t.cached_state = lambda: copy.copy(cached)
        wire = (b'{"state": "joint_state", "joint"'
                b'{"command": "set_gripper", "state": true}\r\n'
                b': [-2216, -56337, -18764, 4777, -95887, -11013], "arm_err": 0}\r\n')
        sender.sendall(wire)
        t._read()
        self.assertIsNone(t.gripper_inflight)
        self.assertIsNone(t.gripper_error)
        self.assertEqual(len(feedback), 1)
        self.assertIn('interleaved_gripper_ack_recovered', [event for event, _ in events])
        self.assertIsNone(t.protocol_pause)

    def test_start_release_follow_and_hold_use_only_localhost(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
            guards = []
            def guard_factory(arm, address):
                guard = FakeGuard(arm, address); guards.append(guard); return guard
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip", guard_factory=guard_factory)
            self.addCleanup(lambda: None if bridge._closed else bridge.close())
            self.assertTrue(guards[0].installed)
            self.assertFalse(any(c["command"] in ("movej", "movej_canfd") for c in fake.calls))
            q = bridge.initial_sim_q_rad.copy()
            ages = dict(controller_age_s=.01, head_age_s=.01, target_age_s=.01)
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "WAIT_RELEASE")
            self.assertEqual(bridge.observe(q, clutch_value=0., **ages).state, "READY")
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "ARMED")
            self.assertFalse(any(c["command"] in ("movej", "movej_canfd") for c in fake.calls))
            q[1] += np.deg2rad(.1)
            for _ in range(10):
                self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "ACTIVE")
                time.sleep(.02)
            motion = [c for c in fake.calls if c["command"] == "movej_canfd"]
            self.assertTrue(motion)
            self.assertLess(motion[-1]["joint"][1], 0)  # raw controller sign retained
            self.assertEqual(bridge.observe(q, clutch_value=0., **ages).state, "WAIT_RELEASE")
            deadline = time.monotonic() + 2
            while not bridge.transport.stop_confirmed and time.monotonic() < deadline: time.sleep(.01)
            self.assertTrue(bridge.transport.stop_confirmed)
            self.assertTrue(any(c["command"] == "movej" for c in fake.calls))
            bridge.close(); self.assertTrue(guards[0].closed)

    def test_trigger_opening_scale(self):
        self.assertEqual(trigger_to_gripper_position(0.), 1000)
        self.assertEqual(trigger_to_gripper_position(.02), 1000)
        self.assertEqual(trigger_to_gripper_position(.25), 750)
        self.assertEqual(trigger_to_gripper_position(.50), 500)
        self.assertEqual(trigger_to_gripper_position(1.), 1)

    def test_deep_trigger_switches_to_force_grasp_with_hysteresis(self):
        self.assertEqual(gripper_command_for_trigger(.5, False), ('position', 500))
        self.assertEqual(gripper_command_for_trigger(.89, False), ('position', 110))
        self.assertEqual(gripper_command_for_trigger(.90, False), ('force', None))
        self.assertEqual(gripper_command_for_trigger(.82, True), ('force', None))
        self.assertEqual(gripper_command_for_trigger(.79, True), ('position', 210))
        self.assertEqual(gripper_command_for_trigger(0., True), ('position', 1000))

    def test_right_trigger_closes_in_proportion_and_release_opens(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-gripper-loopback-") as tmp:
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                guard_factory=lambda arm, addr: FakeGuard(arm, addr))
            self.addCleanup(lambda: None if bridge._closed else bridge.close())
            grip_cmds = ('set_gripper_position', 'set_gripper_pick_on')
            def wait_commands(count):
                deadline = time.monotonic() + 3
                while len([c for c in fake.calls if c['command'] in grip_cmds]) < count and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertEqual(len([c for c in fake.calls if c['command'] in grip_cmds]), count)
                while bridge.transport.gripper_inflight is not None and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertIsNone(bridge.transport.gripper_error)

            bridge.observe_gripper(0., True)
            bridge.observe_gripper(.01, True)
            wait_commands(1)
            self.assertEqual(fake.calls[-1], {
                'command': 'set_gripper_position', 'position': 1000, 'block': False})
            for count, trigger, position in ((2, .25, 750), (3, .50, 500), (4, .85, 150)):
                bridge.observe_gripper(trigger, True)
                wait_commands(count)
                self.assertEqual(fake.calls[-1], {
                    'command': 'set_gripper_position', 'position': position, 'block': False})
            # Fully pressed: one force-controlled grasp, held while the trigger stays deep.
            bridge.observe_gripper(1., True)
            wait_commands(5)
            self.assertEqual(fake.calls[-1], {
                'command': 'set_gripper_pick_on', 'speed': 500, 'force': 500, 'block': False})
            # teleop_sim mirrors this onto the MuJoCo gripper: must stay a number.
            self.assertEqual(bridge.gripper_position / 1000.0, 0.001)
            bridge.observe_gripper(.85, True)
            time.sleep(.25)
            wait_commands(5)
            bridge.observe_gripper(0., True)
            wait_commands(6)
            self.assertEqual(fake.calls[-1], {
                'command': 'set_gripper_position', 'position': 1000, 'block': False})
            bridge.observe_gripper(.01, False)
            time.sleep(.25)
            self.assertEqual(len([c for c in fake.calls if c['command'] in grip_cmds]), 6)
            bridge.close()

    def test_missing_gripper_ack_allows_later_press(self):
        fake = LocalController(); fake.drop_gripper_ack = True; self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-gripper-loopback-") as tmp:
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                guard_factory=lambda arm, addr: FakeGuard(arm, addr))
            self.addCleanup(lambda: None if bridge._closed else bridge.close())
            bridge.observe_gripper(.7, True)
            deadline = time.monotonic() + 3
            while bridge.transport.gripper_error is None and time.monotonic() < deadline: time.sleep(.02)
            self.assertEqual(bridge.transport.gripper_error, 'gripper command not acknowledged')
            fake.drop_gripper_ack = False
            bridge.observe_gripper(.22, True)
            deadline = time.monotonic() + 2
            while len([c for c in fake.calls if c['command'] == 'set_gripper_position']) < 2 and time.monotonic() < deadline: time.sleep(.02)
            self.assertEqual([c for c in fake.calls if c['command'] == 'set_gripper_position'], [
                {'command': 'set_gripper_position', 'position': 300, 'block': False},
                {'command': 'set_gripper_position', 'position': 780, 'block': False}])
            self.assertIsNone(bridge.transport.fault)
            bridge.close()

    def test_startup_error_and_guard_failure_never_send_motion(self):
        for arm_error, guard_fail in ((1, False), (0, True)):
            with self.subTest(arm_error=arm_error, guard_fail=guard_fail):
                fake = LocalController(arm_error=arm_error)
                with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
                    try:
                        with self.assertRaises((ValueError, RuntimeError)):
                            RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                       guard_factory=lambda arm, addr: FakeGuard(arm, addr, fail=guard_fail))
                        self.assertFalse(any(c["command"] in ("movej", "movej_canfd") for c in fake.calls))
                    finally:
                        fake.close()

    def test_head_loss_stops_and_requires_release(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                guard_factory=lambda arm, addr: FakeGuard(arm, addr))
            self.addCleanup(lambda: None if bridge._closed else bridge.close())
            q = bridge.initial_sim_q_rad.copy()
            ages = dict(controller_age_s=.01, head_age_s=.01, target_age_s=.01)
            bridge.observe(q, clutch_value=0., **ages); bridge.observe(q, clutch_value=1., **ages)
            q[1] += np.deg2rad(.1)
            for _ in range(8): bridge.observe(q, clutch_value=1., **ages); time.sleep(.02)
            self.assertGreater(bridge.transport.frames, 0)
            stale = dict(ages, head_age_s=.3)
            self.assertEqual(bridge.observe(q, clutch_value=1., **stale).state, "WAIT_RELEASE")
            deadline = time.monotonic() + 2
            while not bridge.transport.stop_confirmed and time.monotonic() < deadline: time.sleep(.01)
            self.assertTrue(bridge.transport.stop_confirmed)
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "WAIT_RELEASE")
            self.assertEqual(bridge.observe(q, clutch_value=0., **ages).state, "READY")
            bridge.close()

    def test_controller_protocol_pause_recovers_only_after_hold_and_release(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                guard_factory=lambda arm, addr: FakeGuard(arm, addr))
            self.addCleanup(lambda: None if bridge._closed else bridge.close())
            q = bridge.initial_sim_q_rad.copy()
            ages = dict(controller_age_s=.01, head_age_s=.01, target_age_s=.01)
            bridge.observe(q, clutch_value=0., **ages); bridge.observe(q, clutch_value=1., **ages)
            q[1] += np.deg2rad(.1)
            for _ in range(8): bridge.observe(q, clutch_value=1., **ages); time.sleep(.02)
            self.assertGreater(bridge.transport.frames, 0)
            fake.inject_protocol_error.set()
            deadline = time.monotonic() + 2
            while not bridge.transport.protocol_pause and time.monotonic() < deadline: time.sleep(.01)
            self.assertIsNotNone(bridge.transport.protocol_pause)
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "WAIT_RELEASE")
            deadline = time.monotonic() + 2
            while not bridge.transport.stop_confirmed and time.monotonic() < deadline: time.sleep(.01)
            self.assertTrue(bridge.transport.stop_confirmed)
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "WAIT_RELEASE")
            self.assertIsNotNone(bridge.transport.protocol_pause)
            released = bridge.observe(q, clutch_value=0., **ages)
            self.assertEqual(released.state, "READY")
            self.assertIsNone(bridge.transport.protocol_pause)
            resumed_q = released.sync_sim_q_rad.copy()
            self.assertEqual(bridge.observe(resumed_q, clutch_value=1., **ages).state, "ARMED")
            frames_before = bridge.transport.frames
            resumed_q[1] += np.deg2rad(.1)
            for _ in range(8):
                self.assertEqual(bridge.observe(resumed_q, clutch_value=1., **ages).state, "ACTIVE")
                time.sleep(.02)
            self.assertGreater(bridge.transport.frames, frames_before)
            bridge.close()

    def test_lost_feedback_faults_and_does_not_auto_resume(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
            bridge = RealBridge(self.model, fake.address, Path(tmp) / "real.jsonl", "grip",
                                guard_factory=lambda arm, addr: FakeGuard(arm, addr))
            q = bridge.initial_sim_q_rad.copy()
            ages = dict(controller_age_s=.01, head_age_s=.01, target_age_s=.01)
            bridge.observe(q, clutch_value=0., **ages); bridge.observe(q, clutch_value=1., **ages)
            q[1] += np.deg2rad(.1)
            for _ in range(5): bridge.observe(q, clutch_value=1., **ages); time.sleep(.02)
            self.assertGreater(bridge.transport.frames, 0)
            fake.drop_feedback = True
            deadline = time.monotonic() + 1.5
            while not bridge.transport.fault and time.monotonic() < deadline:
                bridge.observe(q, clutch_value=1., **ages); time.sleep(.02)
            self.assertIsNotNone(bridge.transport.fault)
            self.assertEqual(bridge.observe(q, clutch_value=1., **ages).state, "FAULT")
            with self.assertRaisesRegex(RuntimeError, "shutdown not confirmed"):
                bridge.close()

    def test_actual_simulation_loop_with_real_factory_against_localhost(self):
        from test_sim_loopback import Viewer, SyntheticInput, frames
        import example.teleop_sim as sim
        import util.xrtoolkit_input as xr
        import real_bridge as real_module
        fake = LocalController(); self.addCleanup(fake.close)
        samples = (frames(0, True, 8) + frames(1, True, 5) + frames(0, True, 60, .0025) +
                   frames(1, True, 5, .0325) + frames(0, True, 3, .035) +
                   frames(1, True, 5, .0475) + frames(0, True, 30, .05) + frames(1, True, 5, .065))
        guards = []
        def make_bridge(model, address, output, clutch):
            def guard_factory(arm, addr):
                guard = FakeGuard(arm, addr); guards.append(guard); return guard
            return RealBridge(model, address, output, clutch, guard_factory=guard_factory)
        with tempfile.TemporaryDirectory(prefix="rm65-real-bridge-loopback-") as tmp:
            log = Path(tmp) / "real.jsonl"
            argv = ["teleop_sim.py", "--robot", "rm65", "--input-source", "xrtoolkit",
                    "--xrtoolkit-clutch", "grip", "--view-forward-axis", "+x",
                    "--bridge-real", fake.address, "--bridge-real-log", str(log), "--position-scale", ".5"]
            with patch.object(sim.viewer, "launch_passive", lambda model, data: Viewer(len(samples))), \
                 patch.object(xr, "XrToolkitPicoInput", lambda hand: SyntheticInput(samples)), \
                 patch.object(real_module, "RealBridge", make_bridge), \
                 patch.object(sys, "argv", argv):
                sim.main()
            rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(any(row.get("state") == "ACTIVE" for row in rows))
            released = next(i for i, row in enumerate(rows) if row.get("reason") == "clutch_released")
            self.assertTrue(any(row.get("state") == "ACTIVE" for row in rows[released + 1:]))
            self.assertFalse(any(row.get("reason") == "startup_mismatch_resync_required" for row in rows))
            self.assertTrue(any(c["command"] == "movej_canfd" for c in fake.calls))
            self.assertTrue(guards[0].installed and guards[0].closed)
            self.assertTrue(rows[-1]["hold_confirmed"])

    @unittest.skipUnless(os.name == "nt", "Windows guard integration")
    def test_default_guard_clean_exit_against_localhost(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-guard-loopback-") as tmp:
            tmp_path = Path(tmp)
            scripts = tmp_path / "scripts"; scripts.mkdir()
            original = PROJECT / "scripts" / "real_pico_guard.py"
            (scripts / "real_pico_guard.py").write_text(
                "import runpy\nrunpy.run_path(" + repr(str(original)) + ", run_name='__main__')\n",
                encoding="utf-8",
            )
            code = (
                "import sys; from pathlib import Path; import mujoco; "
                "sys.path.insert(0,sys.argv[1]); sys.path.insert(0,sys.argv[2]); "
                "import launch_vdo_pico_real as launch; launch.ROOT=Path(sys.argv[3]); "
                "from real_bridge import RealBridge; "
                "m=mujoco.MjModel.from_xml_path(sys.argv[4]); "
                "b=RealBridge(m,sys.argv[5],Path(sys.argv[3])/'bridge.jsonl','grip'); "
                "b.close(); print('clean-close',flush=True)"
            )
            proc = subprocess.run(
                [sys.executable, "-c", code, str(HERE), str(PROJECT / "scripts"), str(tmp_path),
                 str(HERE / "vr_teleop/example/scene/scene_rm65.xml"), fake.address],
                capture_output=True, text=True, timeout=20, creationflags=subprocess.CREATE_NO_WINDOW,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            deadline = time.monotonic() + 5
            guard_text = ""
            while time.monotonic() < deadline:
                logs = list((tmp_path / "logs/real").glob("guard_*.log"))
                if logs:
                    guard_text = logs[0].read_text(encoding="utf-8", errors="replace")
                    if "confirmed clean exit" in guard_text: break
                time.sleep(.05)
            self.assertIn("confirmed clean exit", guard_text)
            wait_guard_log_closed(logs[0])
            self.assertFalse(any(c["command"] in ("movej", "movej_canfd") for c in fake.calls))

    @unittest.skipUnless(os.name == "nt", "Windows guard integration")
    def test_default_guard_does_not_repeat_confirmed_hold(self):
        fake = LocalController(); self.addCleanup(fake.close)
        with tempfile.TemporaryDirectory(prefix="rm65-real-guard-loopback-") as tmp:
            tmp_path = Path(tmp); scripts = tmp_path / "scripts"; scripts.mkdir()
            original = PROJECT / "scripts" / "real_pico_guard.py"
            (scripts / "real_pico_guard.py").write_text(
                "import runpy\nrunpy.run_path(" + repr(str(original)) + ", run_name='__main__')\n", encoding="utf-8")
            code = '''
import sys,time
from pathlib import Path
import mujoco,numpy as np
sys.path.insert(0,sys.argv[1]); sys.path.insert(0,sys.argv[2])
import launch_vdo_pico_real as launch
launch.ROOT=Path(sys.argv[3])
from real_bridge import RealBridge
m=mujoco.MjModel.from_xml_path(sys.argv[4])
b=RealBridge(m,sys.argv[5],Path(sys.argv[3])/'bridge.jsonl','grip')
q=b.initial_sim_q_rad.copy()
a=dict(controller_age_s=.01,head_age_s=.01,target_age_s=.01)
b.observe(q,clutch_value=0.,**a); b.observe(q,clutch_value=1.,**a)
q[1]+=np.deg2rad(.1)
for _ in range(10):
    b.observe(q,clutch_value=1.,**a); time.sleep(.02)
b.observe(q,clutch_value=0.,**a)
deadline=time.monotonic()+2
while not b.transport.stop_confirmed and time.monotonic()<deadline: time.sleep(.01)
assert b.transport.stop_confirmed
b.close()
'''
            proc = subprocess.run(
                [sys.executable, "-c", code, str(HERE), str(PROJECT / "scripts"), str(tmp_path),
                 str(HERE / "vr_teleop/example/scene/scene_rm65.xml"), fake.address],
                capture_output=True, text=True, timeout=20, creationflags=subprocess.CREATE_NO_WINDOW,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            deadline = time.monotonic() + 5; guard_text = ""
            while time.monotonic() < deadline:
                logs = list((tmp_path / "logs/real").glob("guard_*.log"))
                if logs:
                    guard_text = logs[0].read_text(encoding="utf-8", errors="replace")
                    if "confirmed clean exit" in guard_text: break
                time.sleep(.05)
            self.assertIn("confirmed clean exit", guard_text)
            wait_guard_log_closed(logs[0])
            self.assertTrue(any(c["command"] == "movej_canfd" for c in fake.calls))
            self.assertEqual(sum(c["command"] == "movej" for c in fake.calls), 1)


if __name__ == "__main__":
    unittest.main()
