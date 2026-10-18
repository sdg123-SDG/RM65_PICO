"""Left joystick Cartesian jogging with held wrist orientation; MuJoCo only."""
from __future__ import annotations

import argparse
import time

import mujoco.viewer
import numpy as np

from sim_control import CTRL_DT, SimArm, fresh, view_axes
from xr_input import XRoboToolkitInput, start_service_hint

DEFAULT_SPEED_MPS = 0.06
DEADZONE = 0.15


def stick_axis(v: float) -> float:
    if not np.isfinite(v) or abs(v) <= DEADZONE:
        return 0.
    return float(np.sign(v) * (min(abs(v), 1.) - DEADZONE) / (1. - DEADZONE))


def stick_velocity(axis_xy: np.ndarray, azimuth_deg: float, up: float, speed_mps: float) -> np.ndarray:
    ax, ay = (stick_axis(float(x)) for x in axis_xy)
    forward, right = view_axes(azimuth_deg)
    vel = ay * speed_mps * forward + ax * speed_mps * right + np.array([0., 0., up])
    return vel * min(1., speed_mps / max(np.linalg.norm(vel), 1e-12))


class JogControl:
    def __init__(self, arm: SimArm, speed=DEFAULT_SPEED_MPS, frame="view"):
        self.arm, self.speed, self.frame = arm, speed, frame
        self.active = False
        self.vel = np.zeros(3)
        self.need_neutral = False

    def update(self, sample, azimuth: float) -> None:
        self.vel[:] = 0.
        if not fresh(sample) or sample.axis_xy is None or sample.menu:
            self.arm.hold()
            self.active = False
            self.need_neutral = True
            return
        self.arm.gripper(sample.trigger)
        up = self.speed * (float(sample.secondary_button) - float(sample.primary_button))
        # Freeze screen axes for each continuous jog, even if the camera rotates.
        az = azimuth if self.frame == "view" else 0.
        if self.active:
            az = self.azimuth
        if self.frame == "view":
            vel = stick_velocity(sample.axis_xy, az, up, self.speed)
        else:
            # Robot base convention: +X is forward, +Y is left.
            ax, ay = (stick_axis(float(x)) for x in sample.axis_xy)
            vel = ay * self.speed * np.array([1., 0., 0.])
            vel += ax * self.speed * np.array([0., -1., 0.])
            vel += np.array([0., 0., up])
            vel *= min(1., self.speed / max(np.linalg.norm(vel), 1e-12))
        if np.linalg.norm(vel) < 1e-9:
            self.arm.hold()
            self.active = False
            self.need_neutral = False
            return
        if self.need_neutral:
            return
        if not self.active:
            self.quat = self.arm.quat
            self.target = self.arm.pos
            self.azimuth = az
            self.active = True
        self.vel = vel
        self.target += vel * CTRL_DT
        # Avoid windup against the table, joint limits or unreachable targets.
        gap = self.target - self.arm.pos
        self.target = self.arm.pos + gap * min(1., self.speed * 0.12 / max(np.linalg.norm(gap), 1e-12))
        self.arm.move(self.target, self.quat, self.speed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS, help="Cartesian speed limit in m/s")
    parser.add_argument("--frame", choices=["view", "robot"], default="view")
    args = parser.parse_args()
    if not 0.01 <= args.speed <= 0.30:
        parser.error("speed must be 0.01..0.30 m/s")
    print(start_service_hint())
    print(f"SIMULATION: LEFT stick up=forward, right=right ({args.frame}); "
          f"Y=up, X=down. Neutral=stop. Trigger=gripper. Speed cap {args.speed} m/s.")
    arm = SimArm()
    control = JogControl(arm, speed=args.speed, frame=args.frame)
    src = XRoboToolkitInput(hand="left")
    last_log = last_diag = 0.
    try:
        with mujoco.viewer.launch_passive(arm.model, arm.data) as viewer:
            while viewer.is_running():
                tick = time.perf_counter()
                sample = src.sample()
                control.update(sample, viewer.cam.azimuth)
                arm.step()
                viewer.sync()
                if tick - last_log > 0.5:
                    print(f"valid={sample.valid} stick={sample.axis_xy} "
                          f"active={control.active} xyz={arm.pos.round(3)} "
                          f"neutral_required={control.need_neutral}")
                    last_log = tick
                if tick - last_diag > 1.0:
                    print(src.diagnostics_summary())
                    last_diag = tick
                remaining = CTRL_DT - (time.perf_counter() - tick)
                if remaining > 0.:
                    time.sleep(remaining)
    finally:
        src.close()


if __name__ == "__main__":
    main()
