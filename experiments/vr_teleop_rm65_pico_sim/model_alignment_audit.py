"""Compare saved read-only RM65 feedback with both MuJoCo models; no network I/O."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(PROJECT.parent / "vdo-rm65b"))

from kinemetics.mujoco_solver import Rm65MujocoKinematics
from kinemetics.pose import Pose
from real.json_codec import apply_joint_axis_sign


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.rad2deg(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1))))


def evaluate(path: Path, model, vdo) -> dict:
    report = json.loads(path.read_text(encoding="utf-8-sig"))
    if report.get("read_only") is not True:
        raise ValueError(f"Not a read-only inventory: {path}")
    replies = report["responses"]
    for name in ("get_current_tool_frame", "get_current_work_frame"):
        frame = np.asarray(replies[name]["pose"], dtype=float)
        if frame.shape != (6,) or not np.all(np.isfinite(frame)) or np.any(frame != 0):
            raise ValueError(f"{name} is nonzero or invalid; frame conversion needs commissioning")
    state = replies["get_current_arm_state"]["arm_state"]
    raw = np.asarray(state["joint"], dtype=float) / 1000.0
    pose = np.asarray(state["pose"], dtype=float)
    if raw.shape != (6,) or pose.shape != (6,) or not np.isfinite(raw).all() or not np.isfinite(pose).all():
        raise ValueError(f"Invalid current-arm state: {path}")
    target_p = pose[:3] / 1e6
    target_r = Pose.quat_to_mat(Pose.rpy_to_quat(*(pose[3:] / 1000.0)))
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[model.joint(f"joint_{i}").id] for i in range(1, 7)]
    data.qpos[qadr] = np.deg2rad(raw)
    mujoco.mj_forward(model, data)
    base = model.body("base_link").id
    flange = model.body("link_6").id
    tip = model.site("rm65_ee_site").id
    rb = data.xmat[base].reshape(3, 3)
    flange_p = rb.T @ (data.xpos[flange] - data.xpos[base])
    flange_r = rb.T @ data.xmat[flange].reshape(3, 3)
    tip_p = rb.T @ (data.site_xpos[tip] - data.xpos[base])

    vdo_results = {}
    for name, q in (("controller_raw", raw), ("axis_sign_applied", apply_joint_axis_sign(raw))):
        fk = vdo.fk_at(np.deg2rad(q))
        vdo_results[name] = {
            "position_error_mm": float(np.linalg.norm(fk.pos - target_p) * 1000),
            "rotation_error_deg": rotation_error_deg(Pose.quat_to_mat(fk.quat), target_r),
        }
    return {
        "source": str(path), "created_utc": report.get("created_utc"),
        "controller_joint_deg": raw.tolist(), "controller_tcp_m": target_p.tolist(),
        "controller_rpy_rad": (pose[3:] / 1000.0).tolist(),
        "tool_and_work_frames_zero": True,
        "upstream_link6_position_error_mm": float(np.linalg.norm(flange_p - target_p) * 1000),
        "upstream_link6_rotation_error_deg": rotation_error_deg(flange_r, target_r),
        "controller_tcp_in_upstream_link6_mm": (flange_r.T @ (target_p - flange_p) * 1000).tolist(),
        "upstream_pinch_site_to_controller_tcp_mm": float(np.linalg.norm(tip_p - target_p) * 1000),
        "vdo_fk_candidates": vdo_results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, action="append")
    parser.add_argument("--output", type=Path, default=HERE / "model_alignment_audit.json")
    args = parser.parse_args()
    inputs = args.inventory or [HERE / "real_readonly_inventory.json", PROJECT / "logs/real/exit_readonly_20261031.json"]
    model = mujoco.MjModel.from_xml_path(str(HERE / "vr_teleop/example/scene/scene_rm65.xml"))
    vdo = Rm65MujocoKinematics()
    results = [evaluate(p, model, vdo) for p in inputs]
    offsets = np.asarray([r["controller_tcp_in_upstream_link6_mm"] for r in results])
    result = {
        "network_io": False,
        "motion_commands": False,
        "note": "Comparison of recorded poses only; this does not establish a safe live conversion or a calibrated gripper TCP.",
        "max_offset_spread_mm": float(np.max(np.linalg.norm(offsets - offsets[0], axis=1))),
        "snapshots": results,
    }
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
