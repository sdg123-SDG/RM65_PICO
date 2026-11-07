"""Guarded RM65 joint bridge for the isolated vr_teleop trial.

Creation requires an explicit address. It reads controller state first and
does not submit motion until the supervisor observes release, matching start
pose, and a fresh clutch press. Hardware behavior still requires commissioning.
"""
from __future__ import annotations

import json
import ipaddress
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import uuid

import numpy as np

from bridge_supervisor import BridgeSupervisor
from bridge_mapping import preview_target
from hardware_preflight import check_inventory

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
VDO = PROJECT.parent / "vdo-rm65b"
sys.path.insert(0, str(PROJECT / "scripts"))
sys.path.insert(0, str(VDO))

from latest_bridge import LatestTransport, TraceWriter
from probe_real_robot import ReadOnlyClient, collect
from real.json_codec import apply_joint_axis_sign, decode_arm_state
from real.json_link import JsonArmLink, parse_real_addr
from real_robot_profile import hardware_profile
from simu.mailbox import LatestBox


def trigger_to_gripper_position(value):
    """Map a right-trigger reading to an EG2 opening command.

    ``set_gripper_position`` uses opening units: 1 is fully closed and 1000 is
    fully open. Trigger travel closes in proportion; a released trigger,
    including the small sensor floor, commands fully open.
    """
    if float(value) <= .02:
        return 1000
    return max(1, min(1000, int(round((1. - float(value)) * 1000))))


# Trigger pressed this deep switches to force-controlled grasping
# (set_gripper_pick_on); releasing below the lower value returns to position
# control. Position mode stops at contact and does not keep squeezing.
GRIPPER_FORCE_ON = .90
GRIPPER_FORCE_OFF = .80
GRIPPER_PICK_SPEED = 500
GRIPPER_PICK_FORCE = 500


def gripper_command_for_trigger(value, force_mode):
    """Return ('force', None) or ('position', opening) for a trigger value."""
    value = float(value)
    if value >= GRIPPER_FORCE_ON or (force_mode and value >= GRIPPER_FORCE_OFF):
        return 'force', None
    return 'position', trigger_to_gripper_position(value)


def _default_guard_factory(arm, address):
    from launch_vdo_pico_real import install_guard_wrapper
    install_guard_wrapper(VDO)
    from real.protect import RealRuntimeGuard
    return RealRuntimeGuard(arm, addr=address)


class RealBridge:
    """Own a single controller link through the existing guarded sender."""

    def __init__(self, model, address: str, output: Path, clutch_source: str, *,
                 link_factory=None, guard_factory=None):
        if not address or clutch_source not in ("grip", "trigger"):
            raise ValueError("real bridge needs an explicit address and one clutch source")
        self.address = address
        self.transport = self.guard = self.trace = self.link = None
        self._closed = False
        output.parent.mkdir(parents=True, exist_ok=True)
        self.trace = TraceWriter(output)
        self.last_record, self.last_decision = 0., None
        self.gripper_position = None
        self.gripper_input_log_at = 0.
        try:
            host, port = parse_real_addr(address)
            self.physical_session = not ipaddress.ip_address(host).is_loopback
            self.link = (link_factory or JsonArmLink.connect)(host, port)
            reader = ReadOnlyClient(self.link._sock)
            reader.buffer = self.link._buf
            report = collect(reader)
            self.link._buf = reader.buffer
            initial = report.get("responses", {}).get("get_current_arm_state", {}).get("arm_state", {})
            profile = hardware_profile(report)
            inventory_dir = output.parent / "inventories"
            inventory_dir.mkdir(parents=True, exist_ok=True)
            inventory_path = inventory_dir / f"inventory_{uuid.uuid4().hex}.json"
            inventory_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            check_inventory(inventory_path)

            # Recheck the newest pose against both models, then obtain one more
            # fresh sample immediately before transferring socket ownership.
            reader.buffer = self.link._buf
            latest = reader.query("get_current_arm_state")
            self.link._buf = reader.buffer
            previous = np.asarray(initial.get("joint"), dtype=float) / 1000.
            checked_raw = np.asarray(latest["arm_state"]["joint"], dtype=float) / 1000.
            if checked_raw.shape != (6,) or previous.shape != (6,) or not np.isfinite(checked_raw).all():
                raise ValueError("invalid initial controller joints")
            if np.max(np.abs(checked_raw - previous)) > .2:
                raise RuntimeError("controller moved during startup; retry with a stationary arm")
            report["responses"]["get_current_arm_state"] = latest
            inventory_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            check_inventory(inventory_path)
            reader.buffer = self.link._buf
            latest = reader.query("get_current_arm_state")
            received = time.monotonic()
            self.link._buf = reader.buffer
            raw = np.asarray(latest["arm_state"]["joint"], dtype=float) / 1000.
            if raw.shape != (6,) or not np.isfinite(raw).all() or np.max(np.abs(raw - checked_raw)) > .2:
                raise RuntimeError("controller moved during startup; retry with a stationary arm")
            preview_target(np.deg2rad(raw), model, profile["joint_min_deg"], profile["joint_max_deg"])
            state = decode_arm_state(latest)
            if state.arm_err or state.sys_err:
                raise RuntimeError("controller reports an error at startup")
            state.joint = apply_joint_axis_sign(state.joint)
            self.link.last_arm_state_received = received
            box = LatestBox(); box.put(state)
            arm = SimpleNamespace(link=self.link, plant=SimpleNamespace(), arm_state=box, _busy=False)
            cfg = {"trace_dir": output.parent / "transport_logs", "state_timeout_s": .5,
                   "max_stream_gap_s": .25, "gripper_enabled": True}
            self.transport = LatestTransport(arm, cfg)
            self.supervisor = BridgeSupervisor(self.transport, model, profile["joint_min_deg"], profile["joint_max_deg"],
                clutch_source=clutch_source, max_input_age_s=.25, activation_delta_deg=.02)
            # Install the independent hold guard before any startup reposition.
            self.guard = (guard_factory or _default_guard_factory)(arm, address)
            self.guard.install()
            if self.physical_session:
                raw = self._move_to_taught_pose(raw, model, profile)
            else:
                print("[startup pose] loopback controller; physical taught-pose motion skipped", flush=True)
            self.initial_sim_q_rad = np.deg2rad(raw)
            self.transport.start()  # PICO cannot submit motion until __init__ completes.
            if self.transport.fault:
                raise RuntimeError(f"sender failed during guard startup: {self.transport.fault}")
            self.transport.state()
            self._record("start", inventory=str(inventory_path), clutch_source=clutch_source,
                         initial_controller_joint_deg=raw.tolist(), gripper="right_trigger_analog_position")
        except BaseException:
            self.close()
            raise

    def _move_to_taught_pose(self, current_raw_deg, model, profile):
        """Move to the last hand-guided pose before PICO control is enabled."""
        from bridge_mapping import preview_target
        from real.json_codec import apply_joint_axis_sign, decode_arm_state, encode_joint_deg

        pose_path = PROJECT / "logs" / "real" / "taught_pose.json"
        if not pose_path.is_file():
            raise RuntimeError(f"No taught startup pose at {pose_path}; finish hand-guiding with dragstop first")
        try:
            saved = json.loads(pose_path.read_text(encoding="utf-8"))
            target_mdh = np.asarray(saved["joint_mdh_deg"], dtype=float)
        except Exception as exc:
            raise RuntimeError(f"Invalid taught startup pose file: {exc}") from exc
        if target_mdh.shape != (6,) or not np.isfinite(target_mdh).all():
            raise RuntimeError("Taught startup pose must contain six finite joint angles")

        # The hand-guide tool stores MDH angles. The controller JSON endpoint
        # expects raw controller signs (J2/J3/J5 reversed).
        target_raw = apply_joint_axis_sign(target_mdh)
        low = np.asarray(profile["joint_min_deg"], dtype=float)
        high = np.asarray(profile["joint_max_deg"], dtype=float)
        if np.any(target_raw < low) or np.any(target_raw > high):
            raise RuntimeError("Taught startup pose exceeds the controller's joint limits")
        preview_target(np.deg2rad(target_raw), model, low, high)

        delta = target_raw - np.asarray(current_raw_deg, dtype=float)
        if delta.shape != (6,) or not np.isfinite(delta).all():
            raise RuntimeError("Invalid current joints before startup reposition")
        if np.max(np.abs(delta)) <= .3:
            print("[startup pose] already at taught pose; PICO will wait for clutch release", flush=True)
            return np.asarray(current_raw_deg, dtype=float)

        print("[startup pose] moving to taught hand-guided posture at joint speed 10%; PICO motion remains disabled", flush=True)
        reply = self.link.rpc({
            "command": "movej",
            "joint": encode_joint_deg(target_raw),
            "v": 10,
            "r": 0,
            "trajectory_connect": 0,
        }, timeout_s=5.)
        rejected = any(reply.get(k) is False for k in ("receive_state", "movej", "state")) or reply.get("status") in (False, "false", "error")
        accepted = (reply.get("receive_state") is True or reply.get("movej") is True or
                    reply.get("command") == "movej" or
                    (reply.get("state") == "current_trajectory_state" and reply.get("trajectory_state") is True))
        if rejected or not accepted:
            raise RuntimeError(f"Controller did not accept taught-pose movej: {reply}")

        deadline = time.monotonic() + 90.
        settled = 0
        last_report = 0.
        measured_raw = np.asarray(current_raw_deg, dtype=float)
        while time.monotonic() < deadline:
            msg = self.link.rpc({"command": "get_current_arm_state"}, timeout_s=2.)
            state = decode_arm_state(msg)
            if state.arm_err or state.sys_err or not np.isfinite(state.joint).all():
                raise RuntimeError(f"Controller error during taught-pose move: arm={state.arm_err} sys={state.sys_err}")
            measured_raw = state.joint.copy()
            error = float(np.max(np.abs(target_raw - measured_raw)))
            if error <= .3:
                settled += 1
                if settled >= 3:
                    mdh_state = state
                    mdh_state.joint = apply_joint_axis_sign(mdh_state.joint)
                    self.transport._publish_feedback(mdh_state)
                    self.transport.initial_joint = mdh_state.joint.copy()
                    self.transport.raw_state = msg
                    print("[startup pose] taught pose reached and verified; enabling PICO session", flush=True)
                    self._record("startup_pose_reached", target_controller_deg=target_raw.tolist(),
                                 measured_controller_deg=measured_raw.tolist(), max_error_deg=error)
                    return measured_raw
            else:
                settled = 0
            now = time.monotonic()
            if now - last_report >= 2.:
                print(f"[startup pose] max joint error {error:.2f} deg", flush=True)
                last_report = now
            time.sleep(.1)
        raise TimeoutError("Taught startup pose was not reached within 90 seconds; PICO session will not start")

    def _record(self, event, **fields):
        if self.trace is not None:
            if self.trace.error:
                raise RuntimeError(self.trace.error)
            self.trace.enqueue({"event": event, "monotonic_s": time.monotonic(), **fields})

    def observe(self, q_rad, *, clutch_value, controller_age_s, head_age_s, target_age_s, ik_valid=True):
        if self.trace.error:
            return self.supervisor.fail(self.trace.error)
        if not ik_valid:
            result = self.supervisor.suspend("ik_invalid_or_rejected")
        else:
            result = self.supervisor.update(q_rad, clutch_source=self.supervisor.clutch_source,
                clutch_value=clutch_value, controller_age_s=controller_age_s,
                head_age_s=head_age_s, target_age_s=target_age_s)
        now = time.monotonic()
        key = (result.state, result.reason)
        if key != self.last_decision:
            print(f"[real bridge] {result.state}: {result.reason}", flush=True)
        if key != self.last_decision or now - self.last_record >= .1:
            self._record("decision", state=result.state, reason=result.reason,
                         locally_sent_frames=self.transport.frames)
            self.last_decision, self.last_record = key, now
        return result

    def observe_gripper(self, value, valid):
        now = time.monotonic()
        if now - self.gripper_input_log_at >= 1.:
            print(f"[gripper input] right_trigger={float(value):.2f} valid={bool(valid)}", flush=True)
            self.gripper_input_log_at = now
        if not valid or not np.isfinite(value) or not 0 <= value <= 1:
            return
        mode, position = gripper_command_for_trigger(value, self.gripper_position == 'force')
        if mode == 'force':
            if self.gripper_position == 'force':
                return
            self.gripper_position = 'force'
            self.transport.submit_gripper_pick_on(GRIPPER_PICK_SPEED, GRIPPER_PICK_FORCE)
            print(f"[gripper] force grasp (set_gripper_pick_on speed={GRIPPER_PICK_SPEED} "
                  f"force={GRIPPER_PICK_FORCE})", flush=True)
            return
        if position == self.gripper_position:
            return
        self.gripper_position = position
        self.transport.submit_gripper_position(position)

    def close(self):
        if self._closed:
            return
        self._closed = True
        error = None
        try:
            if self.transport is not None:
                try:
                    self.transport.halt_for_exit()
                except Exception as exc:
                    error = exc
            if self.guard is not None:
                try:
                    self.guard.close()
                except Exception as exc:
                    error = error or exc
            if self.transport is not None:
                try:
                    self.transport.close()
                except Exception as exc:
                    error = error or exc
            elif self.link is not None:
                self.link.close()
            if self.trace is not None:
                try:
                    self._record("end", hold_confirmed=bool(self.transport and self.transport.exit_confirmed),
                                 error=None if error is None else str(error))
                    self.trace.close()
                except Exception as exc:
                    error = error or exc
        finally:
            self.trace = None
        if error is not None:
            raise RuntimeError(f"real bridge shutdown not confirmed: {error}") from error
