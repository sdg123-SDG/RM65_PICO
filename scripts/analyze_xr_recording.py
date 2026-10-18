"""Summarize real input availability, clutch strokes and frame consistency."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import numpy as np


def analyze(path):
    meta, frames = {}, []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("type") == "meta":
                meta = row
            elif row.get("type") == "frame":
                frames.append(row)
    result = {"path": str(path), "input_source": meta.get("source"), "frames": len(frames),
              "valid_controller_frames": 0, "fresh_head_frames": 0, "active_frames": 0,
              "max_vertical_mapping_error": 0., "strokes": []}
    current = []
    base = np.asarray(meta.get("config", {}).get("base_rotation", np.eye(3)))
    def finish():
        if not current:
            return
        hand = np.asarray([r["control"]["hand_fru"] for r in current])
        tool = np.asarray([r["control"]["tool_fru"] for r in current])
        peak = np.argmax(np.linalg.norm(hand, axis=1))
        result["strokes"].append({"start_s": current[0]["t"], "end_s": current[-1]["t"],
                                  "heading_source": current[0]["control"]["heading_source"],
                                  "hand_peak_fru_mm": (hand[peak]*1000).round(2).tolist(),
                                  "tool_at_hand_peak_fru_mm": (tool[peak]*1000).round(2).tolist(),
                                  "max_follow_error_mm": float(np.max(np.linalg.norm(hand-tool, axis=1))*1000)})
        current.clear()
    for row in frames:
        sample, control = row["sample"], row["control"]
        result["valid_controller_frames"] += int(sample["valid"])
        result["fresh_head_frames"] += int(sample.get("headset_quat_wxyz") is not None)
        if control["active"]:
            result["active_frames"] += 1
            current.append(row)
            mapping = np.asarray(control["mapping"])
            error = np.linalg.norm(mapping @ [0., 1., 0.] - base @ [0., 0., 1.])
            result["max_vertical_mapping_error"] = max(result["max_vertical_mapping_error"], float(error))
        else:
            finish()
    finish()
    if not frames:
        result["status"] = "No live controller packets were received; not a motion test."
    elif not result["fresh_head_frames"]:
        result["status"] = "No fresh Head pose recorded. Auto heading cannot be validated; manual heading is labelled separately."
    else:
        result["status"] = "Input available. Stroke values describe measured motion, not independently verified human-intended directions."
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    result = analyze(args.recording)
    output = args.recording.with_suffix(".analysis.json")
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"REPORT: {output}")


if __name__ == "__main__":
    main()
