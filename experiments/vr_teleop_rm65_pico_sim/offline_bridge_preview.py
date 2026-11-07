"""Record simulated bridge candidates; never opens a robot connection."""
from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np

from bridge_mapping import preview_target, verify_codec_source


class OfflineBridgePreview:
    def __init__(self, model, inventory: Path, output: Path):
        verify_codec_source()
        report = json.loads(inventory.read_text(encoding="utf-8-sig"))
        if report.get("read_only") is not True:
            raise ValueError("bridge preview requires a read-only inventory")
        replies = report["responses"]
        self.low = np.asarray(replies["get_joint_min_pos"]["min_pos"], dtype=float) / 1000
        self.high = np.asarray(replies["get_joint_max_pos"]["max_pos"], dtype=float) / 1000
        self.model = model
        output.parent.mkdir(parents=True, exist_ok=True)
        self.output = output.open("x", encoding="utf-8")
        self._closed = False
        self.last_status = None
        self.last_record = 0.0
        self._write({"event": "start", "inventory_created_utc": report.get("created_utc"),
                     "real_feedback": False, "network_io": False, "motion_commands": False})

    def _write(self, item):
        self.output.write(json.dumps({"monotonic_s": time.monotonic(), **item}) + "\n")
        self.output.flush()

    def observe(self, q_rad, *, clutch_active, controller_valid, head_valid, channel, target_valid=True):
        now = time.monotonic()
        reason = None
        candidate = None
        if not controller_valid:
            reason = "controller_missing_or_stale"
        elif not head_valid:
            reason = "head_missing_or_stale"
        elif not clutch_active:
            reason = "clutch_released"
        elif not target_valid:
            reason = "ik_invalid_or_rejected"
        else:
            try:
                candidate = preview_target(q_rad, self.model, self.low, self.high)
            except ValueError as exc:
                reason = f"invalid_target: {exc}"
        status = reason or "candidate_only"
        if status != self.last_status or now - self.last_record >= 0.1:
            self._write({"event": "preview", "status": status, "clutch_channel": channel,
                         "candidate": candidate, "real_feedback": False,
                         "network_io": False, "motion_commands": False})
            self.last_status = status
            self.last_record = now
        return status

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._write({"event": "end", "network_io": False, "motion_commands": False})
        self.output.close()
