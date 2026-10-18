"""Record raw controller/head samples and simulation responses; SDK-free replay."""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import asdict, fields
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import time
import numpy as np
from xr_input import ControllerSample

ROOT = Path(__file__).resolve().parents[1]


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def new_log_path():
    name = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    return ROOT / "logs" / "xr" / (name + ".jsonl")


class SessionRecorder:
    def __init__(self, path, config, source="live"):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("x", encoding="utf-8")
        self.start = self.last_valid = None
        self.count = 0
        self.last_flush = time.monotonic()
        self.file.write(json.dumps({"type": "meta", "format": "rm65-xr-v1", "source": source,
                                    "created_utc": datetime.now(timezone.utc).isoformat(), "config": config}, default=jsonable) + "\n")
        self.file.flush()

    def write(self, sample, control, arm):
        now = time.monotonic()
        if sample.valid:
            self.last_valid = now
        if self.last_valid is None or now - self.last_valid > 1.:
            return
        if self.start is None:
            self.start = now
        payload = asdict(sample)
        payload["age"] = max(0., time.time() - sample.stamp) if sample.stamp else None
        payload["head_age"] = max(0., time.time() - sample.headset_stamp) if sample.headset_stamp else None
        hand, actual = control.motion_summary()
        row = {"type": "frame", "t": now - self.start, "sample": payload,
               "control": {"active": control.active, "reason": control.reason,
                           "heading_source": control.heading_source, "mapping": getattr(control, "mapping", None),
                           "hand_fru": hand, "tool_fru": actual, "target_tcp": control.target_tcp},
               "robot": {"xyz": arm.pos, "tcp": arm.tcp_pos, "q": arm.data.qpos[arm.qadr]}}
        self.file.write(json.dumps(row, default=jsonable, separators=(",", ":")) + "\n")
        self.count += 1
        if now - self.last_flush > 1.:
            self.file.flush()
            self.last_flush = now

    def close(self):
        self.file.close()


class ReplayInput:
    def __init__(self, path):
        self.path = Path(path)
        self.metadata, self.frames = {}, []
        with self.path.open(encoding="utf-8") as stream:
            for line in stream:
                if not line.strip():
                    continue
                record = json.loads(line)
                if record.get("type") == "meta":
                    self.metadata = record
                elif record.get("type") == "frame":
                    self.frames.append(record)
        if not self.frames:
            raise ValueError(f"No controller frames recorded in {path}")
        self.times = [r["t"] for r in self.frames]
        if any(b < a for a, b in zip(self.times, self.times[1:])):
            raise ValueError("Recording time runs backwards")
        self.duration = self.times[-1]
        self.start = time.monotonic()
        self.finished = False

    def sample_at(self, elapsed):
        index = max(0, bisect_right(self.times, elapsed) - 1)
        rec = self.frames[index]
        row = rec["sample"].copy()
        gap = max(0., elapsed - rec["t"])
        age, head_age = row.pop("age", None), row.pop("head_age", None)
        allowed = {f.name for f in fields(ControllerSample)}
        row = {k: v for k, v in row.items() if k in allowed}
        for name in ("pos", "quat_wxyz", "axis_xy", "headset_pos", "headset_quat_wxyz", "left_pos", "left_quat_wxyz"):
            if row.get(name) is not None:
                row[name] = np.asarray(row[name], dtype=float)
        row["stamp"] = time.time() - (age + gap if age is not None else 1e6)
        row["headset_stamp"] = time.time() - (head_age + gap if head_age is not None else 1e6)
        if head_age is None or head_age + gap >= .25:
            row["headset_pos"] = row["headset_quat_wxyz"] = None
        self.finished = elapsed > self.duration + .02
        if self.finished or elapsed < self.times[0] or age is None or age + gap >= .25:
            row["valid"] = False
            row["grip"] = row["trigger"] = 0.
            for key in ("axis_click", "primary_button", "secondary_button", "menu"):
                row[key] = False
        return ControllerSample(**row)

    def sample(self):
        return self.sample_at(time.monotonic() - self.start)

    def diagnostics_summary(self):
        return f"REPLAY frames={len(self.frames)} duration={self.duration:.2f}s original_source={self.metadata.get('source')}"

    def close(self):
        pass
