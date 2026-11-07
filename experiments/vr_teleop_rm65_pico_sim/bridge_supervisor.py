"""Supervise simulated RM65 targets before an injected LatestTransport.

This module does not create sockets, connect to a robot, or start a worker.
It is exercised with a fake/loopback transport until the complete real entry
and independent guard are commissioned.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from bridge_mapping import JOINT_AXIS_SIGN, preview_target, verify_codec_source


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    sync_sim_q_rad: np.ndarray | None = None


class BridgeSupervisor:
    """Fail-closed clutch and target boundary for the existing transport API."""

    def __init__(self, transport, model, controller_min_deg, controller_max_deg, *,
                 clutch_source: str, max_input_age_s: float, activation_delta_deg: float):
        verify_codec_source()
        if clutch_source not in ("grip", "trigger"):
            raise ValueError("choose exactly one clutch source")
        for name, value in (("max_input_age_s", max_input_age_s),
                            ("activation_delta_deg", activation_delta_deg)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.transport, self.model = transport, model
        self.low, self.high = controller_min_deg, controller_max_deg
        self.clutch_source = clutch_source
        self.max_input_age_s = max_input_age_s
        self.activation_delta_deg = activation_delta_deg
        self.state = "WAIT_RELEASE"
        self.fault = None
        self.anchor = None

    def _cancel(self):
        self.transport.cancel_input()
        self.anchor = None

    def _fault(self, reason: str) -> Decision:
        try:
            self._cancel()
        except Exception as exc:
            reason = f"{reason}; cancel_failed: {exc}"
        self.state, self.fault = "FAULT", reason
        return Decision(self.state, reason)

    def fail(self, reason: str) -> Decision:
        if self.fault:
            return Decision("FAULT", self.fault)
        return self._fault(reason)

    def suspend(self, reason: str) -> Decision:
        """Cancel an invalid target without terminating the control session."""
        if self.fault:
            return Decision("FAULT", self.fault)
        try:
            if self.state != "WAIT_RELEASE" or self.transport.sent:
                self._cancel()
        except Exception as exc:
            return self._fault(f"{reason}; cancel_failed: {exc}")
        self.state = "WAIT_RELEASE"
        return Decision(self.state, reason)

    def _held(self) -> bool:
        t = self.transport
        return bool(t.stop_confirmed and not t.sent and not t.stop_pending and t.stop_deadline is None)

    def measured_sim_q_rad(self):
        measured = np.asarray(self.transport.state().joint, dtype=float)
        if measured.shape != (6,) or not np.isfinite(measured).all():
            raise ValueError("invalid_real_feedback")
        return np.deg2rad(measured * JOINT_AXIS_SIGN)

    def update(self, sim_q_rad, *, clutch_source: str, clutch_value: float,
               controller_age_s: float, head_age_s: float, target_age_s: float) -> Decision:
        if self.fault:
            return Decision("FAULT", self.fault)
        if getattr(self.transport, "fault", None):
            return self._fault(f"transport fault: {self.transport.fault}")
        try:
            if clutch_source != self.clutch_source:
                return self._fault("unexpected_clutch_source")
            values = (clutch_value, controller_age_s, head_age_s, target_age_s)
            fresh = all(np.isfinite(x) for x in values) and 0 <= clutch_value <= 1
            fresh = fresh and all(0 <= age <= self.max_input_age_s for age in values[1:])
            if not fresh:
                if self.state != "WAIT_RELEASE" or self.transport.sent:
                    self._cancel()
                self.state = "WAIT_RELEASE"
                return Decision(self.state, "controller_or_head_stale")
            feedback = self.transport.state()
            measured = np.asarray(feedback.joint, dtype=float)
            if measured.shape != (6,) or not np.isfinite(measured).all():
                return self._fault("invalid_real_feedback")
            # Transport feedback is in its internal sign convention; the
            # simulated scene uses the observed controller-raw convention.
            sync_q = np.deg2rad(measured * JOINT_AXIS_SIGN)
            blocked = bool(self.transport.operator_paused or
                           self.transport.stop_pending or self.transport.stop_deadline is not None or
                           (self.state != "WAIT_RELEASE" and
                            (self.transport.rearm_required or self.transport.protocol_pause)))
            if blocked:
                if self.state != "WAIT_RELEASE":
                    self._cancel()
                self.state = "WAIT_RELEASE"
                return Decision(self.state, "transport_not_ready")
            if self.state == "WAIT_RELEASE":
                if clutch_value <= .1 and self._held() and self.transport.acknowledge_release(clutch_value, True):
                    self.state = "READY"
                    return Decision(self.state, "released_and_hold_confirmed", sync_q)
                return Decision(self.state, "release_clutch_and_confirm_hold", sync_q if self._held() else None)
            if clutch_value <= .1:
                if self.state in ("ARMED", "ACTIVE"):
                    self._cancel()
                    self.state = "WAIT_RELEASE"
                    return Decision(self.state, "clutch_released")
                return Decision(self.state, "ready", sync_q if self._held() else None)
            if self.state == "READY" and clutch_value < .55:
                return Decision(self.state, "awaiting_clutch", sync_q if self._held() else None)

            candidate = preview_target(sim_q_rad, self.model, self.low, self.high)
            target = np.asarray(candidate["latest_transport_internal_deg"], dtype=float)
            if self.state == "READY":
                # The first press reanchors to measured joints; it never sends
                # the possibly stale simulated target as an initial command.
                self.anchor = measured.copy()
                self.state = "ARMED"
                return Decision(self.state, "clutch_anchored", sync_q)
            if self.state == "ARMED" and np.max(np.abs(target - self.anchor)) < self.activation_delta_deg:
                return Decision(self.state, "waiting_for_motion")
            self.transport.submit(target, True)
            self.state = "ACTIVE"
            return Decision(self.state, "target_submitted_to_transport")
        except Exception as exc:
            return self._fault(f"boundary_error: {exc}")
