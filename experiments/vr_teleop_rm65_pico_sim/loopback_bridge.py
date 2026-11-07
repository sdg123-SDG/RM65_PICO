"""Connect the trial supervisor to the old sender and a localhost mock only."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

from bridge_mapping import JOINT_AXIS_SIGN, verify_codec_source
from bridge_supervisor import BridgeSupervisor

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(PROJECT / "scripts"))
sys.path.insert(0, str(PROJECT.parent / "vdo-rm65b"))
from latest_bridge import LatestTransport
from simu.mailbox import LatestBox


class LoopbackBridge:
    """Simulated feedback mirrors commands instantly; never connects to RM65."""

    def __init__(self, model, initial_q_rad, inventory: Path, output: Path, clutch_source: str):
        verify_codec_source()
        report = json.loads(inventory.read_text(encoding="utf-8-sig"))
        if report.get("read_only") is not True:
            raise ValueError("loopback requires a read-only inventory for limits")
        replies = report["responses"]
        low = np.asarray(replies["get_joint_min_pos"]["min_pos"], dtype=float) / 1000
        high = np.asarray(replies["get_joint_max_pos"]["max_pos"], dtype=float) / 1000
        initial_raw = np.rad2deg(np.asarray(initial_q_rad, dtype=float))
        if initial_raw.shape != (6,) or not np.isfinite(initial_raw).all():
            raise ValueError("invalid loopback starting joints")
        self.done = threading.Event()
        self.mock_error = None
        self.mock_commands = 0
        self._mock_joint = [int(round(float(x) * 1000)) for x in initial_raw]
        self._closed = False
        output.parent.mkdir(parents=True, exist_ok=True)
        self.output = output.open("x", encoding="utf-8")

        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                listener.bind(("127.0.0.1", 0))
                listener.listen(1)
                client = socket.create_connection(listener.getsockname(), timeout=1.0)
                self.server, peer = listener.accept()
            finally:
                listener.close()
        except Exception:
            self.output.close()
            raise
        if peer[0] != "127.0.0.1" or client.getpeername()[0] != "127.0.0.1":
            client.close(); self.server.close()
            self.output.close()
            raise RuntimeError("loopback bridge accepts localhost only")
        self.client = client
        self.server.settimeout(.05)
        self.mock_thread = threading.Thread(target=self._mock_controller, name="rm65-loopback-mock", daemon=True)
        self.mock_thread.start()

        box = LatestBox()
        box.put(SimpleNamespace(joint=initial_raw * JOINT_AXIS_SIGN, pose=np.zeros(6), arm_err=0, sys_err=0))
        real = SimpleNamespace(link=SimpleNamespace(_sock=client, last_arm_state_received=time.monotonic()),
                               plant=SimpleNamespace(), arm_state=box, _busy=False)
        cfg = {"trace_dir": output.parent / "loopback_transport_logs", "state_timeout_s": .5,
               "max_stream_gap_s": .25, "max_following_error_deg": 2.}
        try:
            self.transport = LatestTransport(real, cfg)
            self.supervisor = BridgeSupervisor(self.transport, model, low, high, clutch_source=clutch_source,
                max_input_age_s=.25, activation_delta_deg=.02)
            self.last_record, self.last_decision = 0., None
            self._write({"event": "start", "controller": "synthetic localhost mirror", "real_feedback": False,
                         "robot_connection": False, "clutch_source": clutch_source})
        except Exception:
            self.done.set()
            self.client.close(); self.server.close(); self.mock_thread.join(.5)
            self.output.close()
            raise

    def start(self):
        # The mock's initial sample is synthetic. Timestamp it only when the
        # viewer is ready, so GUI creation cannot age a not-yet-running test.
        with self.transport.lock:
            self.transport.last_arm_state_received = time.monotonic()
        self.transport.start()

    def _mock_controller(self):
        buf = b""
        try:
            while not self.done.is_set():
                try:
                    chunk = self.server.recv(8192)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    msg = json.loads(line)
                    command = msg.get("command")
                    self.mock_commands += 1
                    reply = None
                    if command == "movej_canfd":
                        self._mock_joint = msg["joint"]
                        reply = {"state": "joint_state", "joint": self._mock_joint, "arm_err": 0}
                    elif command == "movej":
                        self._mock_joint = msg["joint"]
                        reply = {"command": "movej", "receive_state": True}
                    elif command == "get_current_arm_state":
                        reply = {"state": "current_arm_state", "arm_state": {"joint": self._mock_joint,
                                 "pose": [0] * 6, "arm_err": 0, "sys_err": 0}}
                    elif command == "get_joint_en_state":
                        reply = {"state": "joint_en_state", "en_state": [1] * 6}
                    elif command == "get_joint_err_flag":
                        reply = {"state": "joint_err_flag", "err_flag": [0] * 6}
                    if reply is not None:
                        self.server.sendall((json.dumps(reply) + "\r\n").encode())
        except Exception as exc:
            if not self.done.is_set():
                self.mock_error = repr(exc)

    def _write(self, item):
        self.output.write(json.dumps({"monotonic_s": time.monotonic(), **item}) + "\n")
        self.output.flush()

    def observe(self, q_rad, *, clutch_value, controller_age_s, head_age_s, target_age_s, ik_valid=True):
        if self.mock_error:
            raise RuntimeError(f"loopback mock failed: {self.mock_error}")
        if not ik_valid:
            result = self.supervisor.suspend("ik_invalid_or_rejected")
        else:
            result = self.supervisor.update(q_rad, clutch_source=self.supervisor.clutch_source,
                clutch_value=clutch_value, controller_age_s=controller_age_s,
                head_age_s=head_age_s, target_age_s=target_age_s)
        now = time.monotonic()
        key = (result.state, result.reason)
        if key != self.last_decision or now - self.last_record >= .1:
            self._write({"event": "decision", "state": result.state, "reason": result.reason,
                         "loopback_frames": self.transport.frames, "real_feedback": False,
                         "robot_connection": False})
            self.last_decision, self.last_record = key, now
        return result

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.transport.stop()
            self._write({"event": "end", "loopback_frames": self.transport.frames,
                         "robot_connection": False, "real_feedback": False})
        finally:
            self.done.set()
            self.client.close()
            self.server.close()
            self.mock_thread.join(.5)
            self.output.close()
