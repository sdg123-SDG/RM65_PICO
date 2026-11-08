"""Record D405 (session root) and D435i (d435i/ subfolder) alongside the existing teleop, without opening a robot connection."""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "_deps"))
import cv2
import numpy as np
import pyrealsense2 as rs

TRIAL = Path(r"E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim")
DATASETS = TRIAL / "logs/real/datasets"
TRACE = TRIAL / "logs/real/transport_logs"
WRITERS_PER_CAMERA = 4
CAMERA_FPS = 30
SAVE_EVERY = 3  # 30 fps stream -> save 10 fps; color only, no depth
SENSOR_COLUMNS = ["joint_1_deg", "joint_2_deg", "joint_3_deg", "joint_4_deg", "joint_5_deg", "joint_6_deg", "x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad", "pose_source"]


def save_png(path: Path, array: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", array, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError(f"Unable to encode {path}")
    path.write_bytes(encoded.tobytes())


def intrinsics(v):
    return {"width": v.width, "height": v.height, "fx": v.fx, "fy": v.fy,
            "ppx": v.ppx, "ppy": v.ppy, "model": str(v.model), "coeffs": list(v.coeffs)}


def align_records(frames_path: Path, robot_path: Path, output: Path, max_gap_ms=150):
    with frames_path.open(encoding="utf-8", newline="") as f:
        frames = list(csv.DictReader(f))
    with robot_path.open(encoding="utf-8-sig", newline="") as f:
        robot = list(csv.DictReader(f))
    states = sorted(((int(x["timestamp_unix_ns"]), x) for x in robot if x.get("event") in ("feedback", "joint_feedback") and x.get("joint_1_deg")), key=lambda x: x[0])
    events = sorted(int(x["timestamp_unix_ns"]) for x in robot if x.get("gripper_command") == "1")
    stamps = [x[0] for x in states]
    fields = ["frame_id", "episode", "camera_host_timestamp_unix_ns", "color_path", "robot_timestamp_unix_ns", "robot_minus_camera_ms", "robot_valid"] + SENSOR_COLUMNS + ["gripper_command", "gripper_command_count"]
    paired = 0
    frame_times = [int(f["host_timestamp_unix_ns"]) for f in frames]
    if any(b < a for a, b in zip(frame_times, frame_times[1:])):
        raise ValueError("Camera host timestamps moved backwards; alignment is ambiguous")
    prev = frame_times[0] if frame_times else 0
    event_index = bisect.bisect_left(events, prev)
    before = event_index
    with output.open("x", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for index, (frame, t) in enumerate(zip(frames, frame_times)):
            pos = bisect.bisect_left(stamps, t)
            candidates = [j for j in (pos - 1, pos) if 0 <= j < len(states)]
            best = min(candidates, key=lambda j: abs(stamps[j] - t)) if candidates else None
            valid = best is not None and abs(stamps[best] - t) <= max_gap_ms * 1_000_000
            row = {"frame_id": frame["frame_id"], "episode": frame.get("episode", ""), "camera_host_timestamp_unix_ns": t,
                   "color_path": frame["color_path"], "robot_valid": int(valid)}
            if valid:
                ts, state = states[best]
                row.update({"robot_timestamp_unix_ns": ts, "robot_minus_camera_ms": round((ts - t) / 1e6, 3)})
                row.update({key: state.get(key, "") for key in SENSOR_COLUMNS})
                paired += 1
            count = 0
            while event_index < len(events) and events[event_index] <= t:
                if (index == 0 and events[event_index] == t) or (index > 0 and events[event_index] > prev):
                    count += 1
                event_index += 1
            row.update({"gripper_command": int(count > 0), "gripper_command_count": count})
            writer.writerow(row)
            prev = t
    return {"frames": len(frames), "paired_frames": paired, "gripper_events_total": len(events),
            "gripper_events_outside_camera_interval": before + len(events) - event_index, "max_gap_ms": max_gap_ms}


class CameraCapture:
    def __init__(self, folder: Path, model: str = "D405", record_event: threading.Event | None = None, episode: list | None = None):
        self.folder = folder
        self.episode = episode if episode is not None else [0]
        self.record_event = record_event or threading.Event()
        self.model = model
        folder.mkdir(exist_ok=True)
        self.stop_event = threading.Event()
        self.capture_done = threading.Event()
        self.error = None
        self.frames = 0
        self.dropped_queue_frames = 0
        self.camera_frame_gaps = 0
        self.duplicate_color_frames = 0
        self.first_frame_ns = self.last_frame_ns = self.last_camera_frame_number = None
        self.frame_queue = queue.Queue(maxsize=120)
        self.episode_spans = {}  # episode -> [first_ns, last_ns, frames]
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        # Pin the device by model so the recorded camera never depends on enumeration order.
        found = [d for d in rs.context().query_devices() if model.upper() in d.get_info(rs.camera_info.name).upper()]
        if len(found) != 1:
            raise RuntimeError(f"Expected exactly one {model}, found {len(found)}")
        cfg.enable_device(found[0].get_info(rs.camera_info.serial_number))
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, CAMERA_FPS)
        profile = self.pipeline.start(cfg)
        device = profile.get_device()
        self.info = {"name": device.get_info(rs.camera_info.name), "serial": device.get_info(rs.camera_info.serial_number),
                     "color_intrinsics": intrinsics(profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()),
                     "streams": "color only (no depth)", "host_timestamp": "time.time_ns immediately after frameset arrival; not hardware clock synchronization",
                     "requested_camera_fps": CAMERA_FPS, "save_every_n_frames": SAVE_EVERY,
                     "saved_fps_nominal": CAMERA_FPS / SAVE_EVERY, "sampling": f"save every {SAVE_EVERY}th received color frame"}
        (folder / "color").mkdir()
        self.csv_file = (folder / "camera.csv").open("x", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=["frame_id", "episode", "camera_frame_number", "host_timestamp_unix_ns", "camera_timestamp_ms", "camera_timestamp_domain", "color_path"])
        self.csv_writer.writeheader()
        self.thread = threading.Thread(target=self.run, name=f"camera-recorder-{model}")
        # PNG encoding releases the GIL, so several writers keep up with 30 fps x 2 cameras.
        self.writer_threads = [threading.Thread(target=self.write_frames, name=f"camera-writer-{model}-{i}") for i in range(WRITERS_PER_CAMERA)]

    def start(self):
        for t in self.writer_threads:
            t.start()
        self.thread.start()

    def saved_fps(self):
        frames = sum(v[2] for v in self.episode_spans.values())
        span = sum(v[1] - v[0] for v in self.episode_spans.values())
        intervals = frames - len(self.episode_spans)
        return round(intervals * 1e9 / span, 3) if span > 0 and intervals > 0 else None

    def run(self):
        try:
            last_received_frame = None
            received = 0
            last_flush = time.monotonic()
            while not self.stop_event.is_set():
                frames = self.pipeline.wait_for_frames(1000)
                host_ns = time.time_ns()
                if not self.record_event.is_set():
                    continue  # preview only: nothing is saved until recording starts
                color = frames.get_color_frame()
                if not color:
                    continue
                frame_number = color.get_frame_number()
                if frame_number == last_received_frame:
                    self.duplicate_color_frames += 1
                    continue
                last_received_frame = frame_number
                episode = self.episode[0]
                # camera-side drops are counted on every received frame, before downsampling
                if self.last_camera_frame_number is not None and episode == getattr(self, "last_episode", episode):
                    self.camera_frame_gaps += max(0, frame_number - self.last_camera_frame_number - 1)
                    received += 1
                else:
                    received = 0
                self.last_camera_frame_number = frame_number
                self.last_episode = episode
                if received % SAVE_EVERY:
                    continue
                frame_id = f"{self.frames:06d}"
                cp = f"color/{frame_id}.png"
                item = (cp, np.asanyarray(color.get_data()).copy())
                try:
                    self.frame_queue.put_nowait(item)
                except queue.Full:
                    self.dropped_queue_frames += 1
                    continue
                # Row is written in capture order once the frame is queued, so camera.csv stays time-sorted.
                self.csv_writer.writerow({"frame_id": frame_id, "episode": episode, "camera_frame_number": frame_number, "host_timestamp_unix_ns": host_ns,
                                          "camera_timestamp_ms": color.get_timestamp(), "camera_timestamp_domain": str(color.get_frame_timestamp_domain()),
                                          "color_path": cp})
                if self.first_frame_ns is None:
                    self.first_frame_ns = host_ns
                self.last_frame_ns = host_ns
                span = self.episode_spans.setdefault(episode, [host_ns, host_ns, 0])
                span[1] = host_ns
                span[2] += 1
                if time.monotonic() - last_flush >= 1.:
                    self.csv_file.flush()
                    last_flush = time.monotonic()
                self.frames += 1
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            print(f"[camera {self.model}] recording failed: {self.error}", flush=True)
        finally:
            self.stop_event.set()
            self.capture_done.set()

    def write_frames(self):
        try:
            while not self.capture_done.is_set() or not self.frame_queue.empty():
                try:
                    cp, color = self.frame_queue.get(timeout=.1)
                except queue.Empty:
                    continue
                save_png(self.folder / cp, color)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            print(f"[camera {self.model}] recording failed: {self.error}", flush=True)
            self.stop_event.set()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)
        self.pipeline.stop()
        if self.thread.is_alive():
            self.thread.join(timeout=3)
        if self.thread.is_alive():
            self.error = self.error or "camera thread did not stop"
        for t in self.writer_threads:
            t.join(timeout=30)
        if any(t.is_alive() for t in self.writer_threads):
            self.error = self.error or "camera writer did not finish"
        self.csv_file.close()


def wait_for_teleop(process, record_event, manifest, follow_file: Path, episode: list):
    """The teleop writes "1" while the arm follows the hand and "0" when it stops.
    Frames are saved only while following; each follow period is one episode."""
    episodes = manifest.setdefault("episodes", [])
    while True:
        running = process.poll() is None
        try:
            following = running and follow_file.read_text(encoding="ascii").strip() == "1"
        except OSError:
            following = False
        if following and not record_event.is_set():
            episode[0] = len(episodes)
            episodes.append({"episode": episode[0], "start_utc": datetime.now(timezone.utc).isoformat()})
            record_event.set()
            print(f"[data] Arm following: recording episode {episode[0]}.", flush=True)
        elif not following and record_event.is_set():
            record_event.clear()
            episodes[-1]["end_utc"] = datetime.now(timezone.utc).isoformat()
            print(f"[data] Arm stopped following: episode {episode[0]} saved, recording paused.", flush=True)
        if not running:
            return process.returncode
        time.sleep(.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record-now", action="store_true", help="start recording immediately instead of waiting for the left trigger")
    parser.add_argument("--camera-only-seconds", type=float, default=0., help="camera check without starting teleop")
    args = parser.parse_args()
    parent = ROOT / ("_检查" if args.camera_only_seconds else "会话")
    parent.mkdir(parents=True, exist_ok=True)
    folder = parent / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    folder.mkdir()
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "status": "starting", "camera_only": bool(args.camera_only_seconds)}
    camera = camera2 = None
    process = None
    record_event = threading.Event()
    episode = [0]
    if args.camera_only_seconds or args.record_now:
        record_event.set()
    old = {p.name for p in DATASETS.glob("latest_*.csv")} if DATASETS.exists() else set()
    try:
        camera = CameraCapture(folder, "D405", record_event, episode)
        manifest["camera"] = camera.info
        camera2 = CameraCapture(folder / "d435i", "D435i", record_event, episode)
        manifest["camera_d435i"] = camera2.info
        camera.start()
        camera2.start()
        print(f"[data] session: {folder}", flush=True)
        if args.camera_only_seconds:
            time.sleep(args.camera_only_seconds)
        else:
            if not (TRIAL / "start_real_trial.ps1").is_file():
                raise FileNotFoundError("Teleop launcher not found")
            print("[data] Cameras ready. Launching the existing teleop; recording runs only while the arm follows your hand." if not record_event.is_set() else "[data] Camera recording started. Launching the existing teleop.", flush=True)
            trigger_file = folder / "follow_state.flag"
            os.environ["COLLECT_TRIGGER_FILE"] = str(trigger_file)
            process = subprocess.Popen(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(TRIAL / "start_real_trial.ps1")], cwd=str(TRIAL))
            manifest["teleop_exit_code"] = wait_for_teleop(process, record_event, manifest, trigger_file, episode)
        manifest["status"] = "camera_captured"
    except KeyboardInterrupt:
        manifest["status"] = "interrupted"
        print("[data] Interrupted; preserving recorded data.", flush=True)
        if process is not None and process.poll() is None:
            try:
                manifest["teleop_exit_code"] = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                manifest["teleop_still_running"] = True
    except Exception as exc:
        manifest["status"] = "error"
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[data] {manifest['error']}", flush=True)
    finally:
        if not args.camera_only_seconds and not args.record_now and not manifest.get("episodes"):
            manifest["recording_never_started"] = "the arm never started following; no frames saved"
            print("[data] The arm never started following; nothing was recorded.", flush=True)
        for cam, suffix in ((camera, ""), (camera2, "_d435i")):
            if cam is None:
                continue
            cam.close()
            manifest[f"camera{suffix}_frames"] = cam.frames
            manifest[f"camera{suffix}_queue_drops"] = cam.dropped_queue_frames
            manifest[f"camera{suffix}_frame_number_gaps"] = cam.camera_frame_gaps
            manifest[f"camera{suffix}_duplicate_color_frames"] = cam.duplicate_color_frames
            if cam.saved_fps() is not None:
                manifest[f"camera{suffix}_saved_fps"] = cam.saved_fps()
            if cam.error:
                manifest[f"camera{suffix}_error"] = cam.error
                manifest["camera_error"] = manifest.get("camera_error") or f"{cam.model}: {cam.error}"
                if args.camera_only_seconds:
                    manifest["status"] = "error"
        if not args.camera_only_seconds:
            new = [p for p in DATASETS.glob("latest_*.csv") if p.name not in old] if DATASETS.exists() else []
            if len(new) == 1:
                source = new[0]
                shutil.copy2(source, folder / "robot.csv")
                raw = TRACE / (source.stem + ".jsonl")
                if raw.is_file():
                    shutil.copy2(raw, folder / "transport.jsonl")
                try:
                    manifest["alignment"] = align_records(folder / "camera.csv", folder / "robot.csv", folder / "aligned.csv")
                    if camera2 is not None and (folder / "d435i" / "camera.csv").is_file():
                        manifest["alignment_d435i"] = align_records(folder / "d435i" / "camera.csv", folder / "robot.csv", folder / "d435i" / "aligned.csv")
                    if manifest.get("camera_error"):
                        manifest["status"] = "aligned_with_camera_error"
                    elif manifest.get("teleop_exit_code") not in (None, 0):
                        manifest["status"] = "aligned_teleop_failed"
                    else:
                        manifest["status"] = "aligned"
                except Exception as exc:
                    manifest["alignment_error"] = f"{type(exc).__name__}: {exc}"
            else:
                manifest["robot_data_error"] = f"Expected one new CSV; found {len(new)}. Original logs remain in teleop folder."
                if manifest["status"] not in ("error", "interrupted"):
                    manifest["status"] = "teleop_failed_no_robot_data" if manifest.get("teleop_exit_code") not in (None, 0) else "robot_data_missing"
                print("[data] Robot data was not recorded; this is not a synchronized session.", flush=True)
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        (folder / "session.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[data] saved: {folder}; status={manifest['status']}", flush=True)
    return int(manifest["status"] in ("error", "interrupted", "aligned_with_camera_error", "aligned_teleop_failed", "teleop_failed_no_robot_data", "robot_data_missing")
               or bool(manifest.get("robot_data_error") or manifest.get("alignment_error")))


if __name__ == "__main__":
    sys.exit(main())
