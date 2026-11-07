"""XRoboToolkit adapter for the isolated vr_teleop RM65 MuJoCo trial.

Only the left/right controller and headset tracking samples are read. No RM65
SDK, socket, or physical-arm command path is imported here.
"""
from __future__ import annotations

import os
from pathlib import Path
import sys
import time

import numpy as np

from util.quaternion import matrix_to_quaternion, quaternion_to_matrix


def _rm65_pico_root() -> Path:
    configured = os.environ.get("RM65_PICO_ROOT")
    root = Path(configured).expanduser().resolve() if configured else Path(__file__).resolve().parents[4]
    if not (root / "scripts" / "xr_input.py").is_file():
        raise FileNotFoundError(
            f"Cannot find RM65_PICO/scripts/xr_input.py under {root}; "
            "set RM65_PICO_ROOT to the existing project directory."
        )
    return root


class XrToolkitPicoInput:
    """Expose XRoboToolkit controller data using a clutch-anchored view frame."""

    def __init__(self, hand: str = "left") -> None:
        root = _rm65_pico_root()
        sys.path.insert(0, str(root / "scripts"))
        from xr_input import XRoboToolkitInput

        self.hand = hand
        self._source = XRoboToolkitInput(hand=hand)
        self._sample = None
        self._anchor_pos: np.ndarray | None = None
        self._anchor_rot: np.ndarray | None = None
        self._robot_from_pico: np.ndarray | None = None

    def poll(self) -> bool:
        self._sample = self._source.sample()
        return bool(self._sample.valid)

    def sample_status(self) -> dict:
        sample = self._sample
        now = time.time()
        controller_age = now - sample.stamp if sample is not None and sample.valid else float("inf")
        head_age = now - sample.headset_stamp if sample is not None and sample.headset_quat_wxyz is not None else float("inf")
        controller_valid = bool(sample is not None and sample.valid and 0 <= controller_age < .25)
        head_valid = bool(sample is not None and sample.headset_quat_wxyz is not None and 0 <= head_age < .25)
        return {
            "controller_valid": controller_valid,
            "head_valid": head_valid,
            "controller_age_s": controller_age,
            "head_age_s": head_age,
            "grip": float(sample.grip) if controller_valid else 0.,
            "trigger": float(sample.trigger) if controller_valid else 0.,
            "right_trigger": float(getattr(sample, "right_trigger", 0.)),
            "right_trigger_valid": bool(getattr(sample, "right_trigger_valid", False)),
        }

    def clutch_value(self, source: str = "grip") -> float:
        if self._sample is None or not self._sample.valid:
            return 0.0
        return float(np.clip(getattr(self._sample, source), 0.0, 1.0))

    def clutch_channel(self, source: str = "auto", threshold: float = 0.55) -> str | None:
        if source == "auto":
            if self.clutch_value("grip") >= threshold:
                return "grip"
            if self.clutch_value("trigger") >= threshold:
                return "trigger"
            return None
        return source if self.clutch_value(source) >= threshold else None

    def clutch_pressed(self, source: str = "auto", threshold: float = 0.55) -> bool:
        return self.clutch_channel(source, threshold) is not None

    @staticmethod
    def _controller_rotation(sample) -> np.ndarray:
        q_wxyz = np.asarray(sample.quat_wxyz, dtype=np.float64)
        q_xyzw = q_wxyz[[1, 2, 3, 0]]
        q_norm = float(np.linalg.norm(q_xyzw))
        if not np.all(np.isfinite(q_xyzw)) or q_norm < 1e-8:
            raise ValueError("invalid controller quaternion")
        return np.asarray(quaternion_to_matrix(q_xyzw / q_norm), dtype=np.float64)

    def begin_clutch(self, forward_axis: str = "+x") -> dict[str, list[float] | str]:
        """Anchor hand pose and map the current horizontal head-forward to base ±X."""
        sample = self._sample
        if sample is None or not sample.valid or sample.pos is None or sample.quat_wxyz is None:
            raise ValueError("controller tracking is not fresh")
        if forward_axis not in ("+x", "-x"):
            raise ValueError("forward_axis must be '+x' or '-x'")

        if sample.headset_quat_wxyz is None:
            raise ValueError(
                "Head pose missing; enable Tracking > Head and Send on XRoboToolkit"
            )
        head_q_wxyz = np.asarray(sample.headset_quat_wxyz, dtype=np.float64)
        head_q_xyzw = head_q_wxyz[[1, 2, 3, 0]]
        head_norm = float(np.linalg.norm(head_q_xyzw))
        if not np.all(np.isfinite(head_q_xyzw)) or head_norm < 1e-8:
            raise ValueError("invalid headset quaternion")
        head_rot = np.asarray(quaternion_to_matrix(head_q_xyzw / head_norm), dtype=np.float64)

        # XR coordinates are X-right, Y-up, Z-backward. Remove head pitch/roll
        # so a nod does not tilt the robot's horizontal base frame.
        up_pico = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        forward_pico = head_rot @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
        forward_pico[1] = 0.0
        forward_norm = float(np.linalg.norm(forward_pico))
        if forward_norm < 1e-6:
            raise ValueError("head is too close to vertical to calibrate forward")
        forward_pico /= forward_norm
        right_pico = np.cross(forward_pico, up_pico)
        right_pico /= np.linalg.norm(right_pico)

        sign = 1.0 if forward_axis == "+x" else -1.0
        self._robot_from_pico = np.vstack(
            [sign * forward_pico, -sign * right_pico, up_pico]
        )
        self._anchor_pos = np.asarray(sample.pos, dtype=np.float64).copy()
        self._anchor_rot = self._controller_rotation(sample)
        return {
            "forward_axis": forward_axis,
            "reference_mode": "head_view",
            "robot_forward_in_pico": forward_pico.tolist(),
            "robot_left_in_pico": (-right_pico).tolist(),
        }

    def end_clutch(self) -> None:
        self._anchor_pos = None
        self._anchor_rot = None
        self._robot_from_pico = None

    def get_wrist_pose(self, side: str = "left"):
        del side
        sample = self._sample
        if (
            sample is None
            or not sample.valid
            or sample.pos is None
            or sample.quat_wxyz is None
            or self._anchor_pos is None
            or self._anchor_rot is None
            or self._robot_from_pico is None
        ):
            return None

        controller_rot = self._controller_rotation(sample)
        delta_pos = self._robot_from_pico @ (
            np.asarray(sample.pos, dtype=np.float64) - self._anchor_pos
        )
        # Re-express the wrist's spatial rotation delta in the calibrated robot
        # base frame. The start orientation maps to identity at clutch-down.
        delta_rot_pico = controller_rot @ self._anchor_rot.T
        delta_rot_robot = self._robot_from_pico @ delta_rot_pico @ self._robot_from_pico.T
        delta_quat = matrix_to_quaternion(
            tuple(tuple(float(v) for v in row) for row in delta_rot_robot)
        )
        return (
            tuple(float(v) for v in delta_pos),
            tuple(float(v) for v in delta_quat),
        )

    def get_pinch_distance(self, side: str = "left"):
        del side
        sample = self._sample
        if sample is None or not sample.valid:
            return None
        # Keep the existing trigger -> simulated gripper mapping.
        return float((1.0 - np.clip(sample.trigger, 0.0, 1.0)) * 0.10)

    def get_landmarks_mediapipe(self, side: str = "left"):
        del side
        return None

    def diagnostics(self) -> str:
        base = self._source.diagnostics_summary()
        if self._sample is None:
            return base
        return (
            f"{base} hand={self.hand} grip={self._sample.grip:.2f} "
            f"trigger={self._sample.trigger:.2f} "
            f"buttons={int(self._sample.primary_button)}/{int(self._sample.secondary_button)} "
            f"axis_click={int(self._sample.axis_click)} "
            f"head={'yes' if self._sample.headset_quat_wxyz is not None else 'no'}"
        )

    def stop(self) -> None:
        self._source.close()
