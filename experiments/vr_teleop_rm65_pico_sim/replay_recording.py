"""Replay recorded controller joints in MuJoCo; never connects to hardware."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
SCENE = HERE / "vr_teleop/example/scene/scene_rm65.xml"


def load_recording(path):
    times, joints = [], []
    with path.open(encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            if row.get("event") not in ("feedback", "joint_feedback"):
                continue
            q = [float(row[f"joint_{j}_deg"]) for j in range(1, 7)]
            t = float(row["elapsed_s"])
            if not np.isfinite(q).all() or not np.isfinite(t) or (times and t < times[-1]):
                raise ValueError("Invalid or unordered recorded feedback")
            times.append(t)
            joints.append(q)
    if len(times) < 2:
        raise ValueError("Recording needs at least two joint feedback samples")
    return np.asarray(times), np.deg2rad(np.asarray(joints))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--skip-initial-idle", action="store_true")
    parser.add_argument("--check", action="store_true", help="load model and recording without opening a window")
    args = parser.parse_args()
    times, joints = load_recording(args.csv)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    addresses = [int(model.jnt_qposadr[model.joint(f"joint_{i}").id]) for i in range(1, 7)]
    start = times[0]
    if args.skip_initial_idle:
        moving = np.flatnonzero(np.max(np.abs(joints - joints[0]), axis=1) > np.deg2rad(.2))
        if len(moving):
            start = max(times[0], times[moving[0]] - 1.)
    data.qpos[addresses] = [np.interp(start, times, joints[:, i]) for i in range(6)]
    mujoco.mj_forward(model, data)
    print(f"Simulation only. Start {start:.2f}s; duration {times[-1] - start:.2f}s. Gripper position not recorded.", flush=True)
    if args.check:
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        began = time.monotonic()
        while viewer.is_running():
            t = min(times[-1], start + time.monotonic() - began)
            with viewer.lock():
                data.qpos[addresses] = [np.interp(t, times, joints[:, i]) for i in range(6)]
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(.02)


if __name__ == "__main__":
    main()
