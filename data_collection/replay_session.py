"""Replay one collected session (aligned.csv, the saved 10 fps dataset) in MuJoCo.

Simulation only: never connects to the robot and records nothing.
Usage: replay_session.py <session folder> [--camera d405|d435i] [--speed 1.0] [--check]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

SCENE = Path(r"E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim\vr_teleop\example\scene\scene_rm65.xml")


def load(session: Path, camera: str):
    path = session / ("aligned.csv" if camera == "d405" else "d435i/aligned.csv")
    with path.open(encoding="utf-8", newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["robot_valid"] == "1"]
    if len(rows) < 2:
        raise ValueError(f"{path} has fewer than two frames with robot data")
    t = np.array([int(r["camera_host_timestamp_unix_ns"]) for r in rows], dtype=np.float64) / 1e9
    q = np.deg2rad([[float(r[f"joint_{j}_deg"]) for j in range(1, 7)] for r in rows])
    ep = np.array([int(r["episode"] or 0) for r in rows])
    grip = np.array([int(r["gripper_command"]) for r in rows])
    return t, q, ep, grip, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    ap.add_argument("--camera", choices=["d405", "d435i"], default="d405")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--check", action="store_true", help="print a summary without opening a window")
    args = ap.parse_args()
    t, q, ep, grip, rows = load(args.session, args.camera)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    adr = [int(model.jnt_qposadr[model.joint(f"joint_{i}").id]) for i in range(1, 7)]
    rng = np.array([model.jnt_range[model.joint(f"joint_{i}").id] for i in range(1, 7)])
    dt = np.diff(t)
    jump = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    print(f"Simulation only. {len(t)} frames, {t[-1] - t[0]:.1f} s, episodes {sorted(set(ep.tolist()))}", flush=True)
    print(f"frame interval median {np.median(dt) * 1000:.0f} ms, max {dt.max() * 1000:.0f} ms", flush=True)
    print("joint range (deg):", "  ".join(f"J{i + 1} {np.rad2deg(q[:, i].min()):.0f}~{np.rad2deg(q[:, i].max()):.0f}" for i in range(6)), flush=True)
    out = np.any((q < rng[:, 0]) | (q > rng[:, 1]), axis=1).sum()
    print(f"frames outside model joint limits: {out}; largest per-frame joint step {np.rad2deg(jump.max()):.1f} deg", flush=True)
    print(f"frames with gripper command: {int(grip.sum())}", flush=True)
    if args.check:
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            began = time.monotonic()
            last_i = -1
            while viewer.is_running():
                now = t[0] + (time.monotonic() - began) * args.speed
                if now > t[-1]:
                    break
                i = int(np.searchsorted(t, now, side="right") - 1)
                with viewer.lock():
                    data.qpos[adr] = [np.interp(now, t, q[:, k]) for k in range(6)]
                    mujoco.mj_forward(model, data)
                if i != last_i and (i % 10 == 0 or grip[i]):
                    print(f"t={t[i] - t[0]:6.1f}s frame {rows[i]['frame_id']} episode {ep[i]}" + ("  [gripper cmd]" if grip[i] else ""), flush=True)
                last_i = i
                viewer.sync()
                time.sleep(.02)
            print("-- end of session, replaying again (close the window to stop) --", flush=True)
            time.sleep(1.)


if __name__ == "__main__":
    main()
