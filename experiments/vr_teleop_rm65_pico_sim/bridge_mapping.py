"""Offline RM65 simulation-to-LatestTransport mapping preview.

This file opens no socket and cannot send robot commands. The sign mapping is
exposed here so it can be checked against recorded physical feedback before
any live bridge is connected.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
CODEC_PATH = PROJECT.parent / "vdo-rm65b" / "real" / "json_codec.py"
CODEC_SHA256 = "3dd9031f1cd54ae051dc6a143ab79cd964bbfe142e66757ff14fa6bc1dfdc21a"
JOINT_AXIS_SIGN = np.array([1., -1., -1., 1., -1., 1.])


def verify_codec_source() -> None:
    if hashlib.sha256(CODEC_PATH.read_bytes()).hexdigest() != CODEC_SHA256:
        raise RuntimeError("The old bridge codec changed; re-audit the offline mapping before preview")


def _vector(values, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (6,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain exactly six finite values")
    return array


def preview_target(sim_q_rad, model, controller_min_deg, controller_max_deg) -> dict:
    """Preview the two boundaries without constructing a hardware transport."""
    q = _vector(sim_q_rad, "simulated joints")
    low = _vector(controller_min_deg, "controller minimum")
    high = _vector(controller_max_deg, "controller maximum")
    if np.any(low >= high):
        raise ValueError("invalid controller joint limits")
    ranges = np.array([model.jnt_range[model.joint(f"joint_{i}").id] for i in range(1, 7)])
    if np.any(q < ranges[:, 0]) or np.any(q > ranges[:, 1]):
        raise ValueError("simulated target exceeds MuJoCo joint limits")

    # The two recorded physical snapshots matched the upstream model when
    # controller joint angles were used directly. This assumption is an audit
    # candidate, not authorization to send a target to hardware.
    controller_deg = np.rad2deg(q)
    if np.any(controller_deg < low) or np.any(controller_deg > high):
        raise ValueError("target exceeds the recorded controller joint limits")
    latest_target = controller_deg * JOINT_AXIS_SIGN
    wire_deg = latest_target * JOINT_AXIS_SIGN
    return {
        "sim_joint_rad": q.tolist(),
        "controller_candidate_deg": controller_deg.tolist(),
        "latest_transport_internal_deg": latest_target.tolist(),
        "wire_candidate_millideg": [int(round(float(v) * 1000.0)) for v in wire_deg],
        "network_io": False,
        "motion_commands": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=HERE / "real_readonly_inventory.json")
    parser.add_argument("--output", type=Path, default=HERE / "bridge_mapping_preview.json")
    args = parser.parse_args()
    verify_codec_source()
    report = json.loads(args.inventory.read_text(encoding="utf-8-sig"))
    if report.get("read_only") is not True:
        raise ValueError("inventory must be from a read-only query")
    replies = report["responses"]
    raw = _vector(replies["get_current_arm_state"]["arm_state"]["joint"], "recorded controller joints") / 1000
    low = _vector(replies["get_joint_min_pos"]["min_pos"], "controller minimum") / 1000
    high = _vector(replies["get_joint_max_pos"]["max_pos"], "controller maximum") / 1000
    model = mujoco.MjModel.from_xml_path(str(HERE / "vr_teleop/example/scene/scene_rm65.xml"))
    preview = preview_target(np.deg2rad(raw), model, low, high)
    preview["inventory_created_utc"] = report.get("created_utc")
    preview["assumption"] = "Upstream simulated joint coordinates equal controller raw joints; observed in two recorded poses, not verified for hardware motion."
    args.output.write_text(json.dumps(preview, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(preview, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
