"""Fixed-base RM65 teleoperation: HMD yaw alignment, clutch, record and replay."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time
import mujoco
import mujoco.viewer
import numpy as np
from sim_control import CTRL_DT, PoseTeleop, SimArm
from xr_input import XRoboToolkitInput, start_service_hint
from xr_recording import ReplayInput, SessionRecorder, new_log_path


def draw_axes(viewer, arm, control):
    with viewer.lock():
        scene = viewer.user_scn
        scene.ngeom = 0
        origin = arm.tcp_pos
        basis = arm.base_rotation @ np.diag([1., -1., 1.])
        for axis, color in zip(basis.T, ([1., .15, .15, 1.], [.1, .9, .2, 1.], [.15, .4, 1., 1.])):
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), origin, np.eye(3).ravel(), np.array(color))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .003, origin, origin + .10 * axis)
            scene.ngeom += 1
        if control.active:
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.full(3, .008), control.target_tcp, np.eye(3).ravel(), np.array([1., .8, .1, .55]))
            scene.ngeom += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", choices=["robot", "view"], help="robot base is fixed; view is legacy camera-relative mode")
    parser.add_argument("--mode", choices=["position", "pose"])
    parser.add_argument("--scale", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--yaw-deg", type=float)
    parser.add_argument("--replay", type=Path, help="recorded input; does not load the XR SDK")
    parser.add_argument("--headless", action="store_true", help="replay simulation without a window")
    parser.add_argument("--record", type=Path, help="new JSONL output; live input is recorded automatically")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--duration", type=float, default=0., help="optional run limit in seconds")
    args = parser.parse_args()
    replay = ReplayInput(args.replay) if args.replay else None
    saved = replay.metadata.get("config", {}) if replay else {}
    for name, default in (("frame", "robot"), ("mode", "position"), ("scale", 1.), ("speed", .06), ("yaw_deg", 0.)):
        if getattr(args, name) is None:
            setattr(args, name, saved.get(name, default))
    if not (.01 <= args.speed <= .30 and .05 <= args.scale <= 3.):
        parser.error("speed must be .01.. .30 m/s and scale .05..3")
    if args.headless and not args.replay:
        parser.error("--headless requires --replay")
    arm = SimArm()
    control = PoseTeleop(arm, frame=args.frame, scale=args.scale, speed=args.speed, mode=args.mode, yaw_deg=args.yaw_deg)
    src = replay if replay is not None else XRoboToolkitInput(hand="left")
    recorder = None
    metrics = {"source": "replay" if args.replay else "live", "original_input_source": replay.metadata.get("source") if replay else "live", "frames": 0, "valid_frames": 0,
               "head_frames": 0, "active_frames": 0, "heading_sources": {}, "max_target_error_mm": 0.}
    try:
        if not args.no_record and (not args.replay or args.record):
            config = {"frame": args.frame, "mode": args.mode, "scale": args.scale, "speed": args.speed,
                      "base_xyz": arm.data.xpos[arm.base].tolist(), "base_rotation": arm.base_rotation.tolist(),
                      "tcp_offset": arm.tcp_offset.tolist(), "yaw_deg": args.yaw_deg}
            recorder = SessionRecorder(args.record or new_log_path(), config, "replay" if args.replay else "live")
            print(f"RECORD: {recorder.path}", flush=True)
        if not args.replay:
            print(start_service_hint())
        print("SIMULATION ONLY. LEFT Grip=follow, release=stop; trigger=gripper.\n"
              "Enable Head Tracking + Controller Tracking + Send. Look toward the workspace when pressing Grip.\n"
              "No movement calibration. Missing Head: release Grip, point controller forward and CLICK STICK.\n"
              "X with Grip released returns to automatic HMD heading.\n"
              "RED=base +X forward, GREEN=base -Y right, BLUE=base +Z up. Yellow dot=TCP target.\n"
              f"frame={args.frame}; scale={args.scale}; speed={args.speed} m/s; wrist={args.mode}", flush=True)
        context = nullcontext(None) if args.headless else mujoco.viewer.launch_passive(arm.model, arm.data)
        with context as viewer:
            if viewer is not None:
                with viewer.lock():
                    viewer.cam.azimuth, viewer.cam.elevation = 0., -25.
                    viewer.cam.lookat[:] = [.22, 0., 1.]
                    viewer.cam.distance = 1.45
            last_log = last_diag = 0.
            last_reason = ""
            start = time.perf_counter()
            step_index = 0
            while viewer is None or viewer.is_running():
                tick = time.perf_counter()
                elapsed = step_index * CTRL_DT if args.headless else tick - start
                sample = src.sample_at(elapsed) if args.headless else src.sample()
                azimuth = viewer.cam.azimuth if viewer is not None else 0.
                control.update(sample, azimuth)
                arm.step()
                hand, tool = control.motion_summary()
                if recorder is not None:
                    recorder.write(sample, control, arm)
                metrics["frames"] += 1
                metrics["valid_frames"] += int(sample.valid)
                metrics["head_frames"] += int(sample.headset_quat_wxyz is not None)
                if control.active:
                    metrics["active_frames"] += 1
                    source = control.heading_source
                    metrics["heading_sources"][source] = metrics["heading_sources"].get(source, 0) + 1
                    metrics["max_target_error_mm"] = max(metrics["max_target_error_mm"], arm.position_error * 1000.)
                head = "OK" if sample.headset_quat_wxyz is not None else "MISSING"
                if viewer is not None:
                    draw_axes(viewer, arm, control)
                    if hasattr(viewer, "set_texts"):
                        viewer.set_texts((None, None,
                            f"RM65 FIXED BASE | Head: {head}\n{control.reason}\nHeading: {control.heading_source}\n"
                            "Grip=follow; release=stop; stick click=set forward; X=auto Head\n"
                            "RED forward | GREEN right | BLUE up | yellow target\n"
                            f"Hand F/R/U mm: {np.round(hand*1000,1)}\nTool F/R/U mm: {np.round(tool*1000,1)}", ""))
                    viewer.sync()
                if not args.headless and (tick - last_log > 2. or control.reason != last_reason):
                    print(f"{control.reason} Head={head} heading={control.heading_source} grip={sample.grip:.2f} "
                          f"hand_FRU_mm={np.round(hand*1000,1)} tool_FRU_mm={np.round(tool*1000,1)}", flush=True)
                    last_log, last_reason = tick, control.reason
                if not args.headless and tick - last_diag > 5.:
                    print(src.diagnostics_summary(), flush=True)
                    last_diag = tick
                if args.replay and src.finished:
                    break
                if args.duration and elapsed >= args.duration:
                    break
                step_index += 1
                remaining = CTRL_DT - (time.perf_counter() - tick)
                if not args.headless and remaining > 0.:
                    time.sleep(remaining)
    finally:
        src.close()
        if recorder is not None:
            recorder.close()
            print(f"RECORDED {recorder.count} frames: {recorder.path}", flush=True)
        if args.replay:
            report = args.replay.with_suffix(".replay_report.json")
            report.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
            print(json.dumps(metrics, indent=2), flush=True)
            print(f"REPORT: {report}", flush=True)


if __name__ == "__main__":
    main()
