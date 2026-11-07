"""Check a recorded read-only RM65 inventory against both local models.

No sockets or motion commands are used. Passing this offline check does not
assert current feedback freshness or authorize physical motion.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from bridge_mapping import preview_target, verify_codec_source
from model_alignment_audit import evaluate

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(PROJECT / "scripts"))
from real_robot_profile import hardware_profile
from kinemetics.mujoco_solver import Rm65MujocoKinematics


def check_inventory(path: Path) -> dict:
    verify_codec_source()
    report = json.loads(path.read_text(encoding="utf-8-sig"))
    if report.get("read_only") is not True:
        raise ValueError("inventory must originate from a read-only query")
    profile = hardware_profile(report)
    state = report["responses"]["get_current_arm_state"]["arm_state"]
    if state.get("arm_err") != 0 or state.get("sys_err") != 0:
        raise ValueError("controller reported arm or system error")
    q_deg = np.asarray(state["joint"], dtype=float) / 1000
    model = mujoco.MjModel.from_xml_path(str(HERE / "vr_teleop/example/scene/scene_rm65.xml"))
    mapping = preview_target(np.deg2rad(q_deg), model, profile["joint_min_deg"], profile["joint_max_deg"])
    alignment = evaluate(path, model, Rm65MujocoKinematics())
    vdo = alignment["vdo_fk_candidates"]["controller_raw"]
    if vdo["position_error_mm"] > .5 or vdo["rotation_error_deg"] > .2:
        raise ValueError("recorded controller pose disagrees with VDO FK")
    offset = np.asarray(alignment["controller_tcp_in_upstream_link6_mm"])
    if np.linalg.norm(offset - np.array([0., 0., -28.6])) > .5 or alignment["upstream_link6_rotation_error_deg"] > .2:
        raise ValueError("recorded controller pose disagrees with the observed upstream link-6 transform")
    return {
        "source": str(path), "inventory_created_utc": report.get("created_utc"),
        "offline_geometry_and_joint_checks_passed": True,
        "live_feedback_freshness_checked": False,
        "real_motion_tested": False,
        "offline_validator_opened_robot_connection": False,
        "joint_deg_controller_raw": q_deg.tolist(),
        "candidate_latest_transport_internal_deg": mapping["latest_transport_internal_deg"],
        "vdo_fk_position_error_mm": vdo["position_error_mm"],
        "vdo_fk_rotation_error_deg": vdo["rotation_error_deg"],
        "upstream_link6_to_controller_tcp_mm": offset.tolist(),
        "controller_tool": profile["tool"]["tool_name"],
        "collision_stage": profile["collision_stage"],
        "note": "Recorded-state check only; fingertip calibration, live freshness, independent guard and physical validation remain outstanding.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=HERE / "real_readonly_inventory.json")
    parser.add_argument("--output", type=Path, default=HERE / "hardware_preflight_report.json")
    args = parser.parse_args()
    result = check_inventory(args.inventory)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
