"""Export one existing transport trace without connecting to the robot."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(PROJECT.parent / "vdo-rm65b"))
from kinemetics.mujoco_solver import Rm65MujocoKinematics
from kinemetics.pose import Pose
from real.json_codec import apply_joint_axis_sign

COLUMNS = (["timestamp_unix_ns", "elapsed_s", "event", "state_age_s"]
           + [f"joint_{n}_deg" for n in range(1, 7)]
           + ["x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad",
              "pose_source", "gripper_command"])


def export_trace(source: Path, destination: Path) -> tuple[int, int]:
    """Use measured joint replies; gripper 1 means a command was sent locally."""
    kin = Rm65MujocoKinematics()
    destination.parent.mkdir(parents=True, exist_ok=True)
    count = commands = 0
    last_state = None
    origin = None
    with source.open(encoding="utf-8") as inp, destination.open("x", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=COLUMNS)
        writer.writeheader()
        for line_number, line in enumerate(inp, 1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid trace JSON at line {line_number}") from exc
            kind = event.get("type")
            if kind not in ("feedback", "joint_feedback", "gripper_sent"):
                continue
            stamp = float(event["monotonic_s"])
            if origin is None:
                origin = stamp
            if kind in ("feedback", "joint_feedback"):
                internal = np.asarray(event["joint_deg"], dtype=float)
                if internal.shape != (6,) or not np.isfinite(internal).all():
                    raise ValueError(f"Invalid joint feedback at line {line_number}")
                joint = apply_joint_axis_sign(internal)
                fk = kin.fk_at(np.deg2rad(joint))
                rpy = Pose.quat_to_rpy(fk.quat)
                last_state = (stamp, joint.tolist(), np.concatenate((fk.pos, rpy)).tolist())
            row = {"timestamp_unix_ns": event["wall_time_ns"],
                   "elapsed_s": round(stamp - origin, 6), "event": kind,
                   "gripper_command": int(kind == "gripper_sent")}
            if last_state is not None:
                state_stamp, joint, pose = last_state
                row["state_age_s"] = round(max(0., stamp - state_stamp), 6)
                row.update(zip([f"joint_{n}_deg" for n in range(1, 7)], joint))
                row.update(zip(["x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad"], pose))
                row["pose_source"] = "fk_measured_joints"
            writer.writerow(row)
            count += 1
            commands += row["gripper_command"]
    return count, commands


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.trace.parent.parent / "datasets" / (args.trace.stem + ".csv")
    rows, commands = export_trace(args.trace, output)
    print(f"[teleop data] {output} ({rows} rows; {commands} gripper sends)")


if __name__ == "__main__":
    main()
