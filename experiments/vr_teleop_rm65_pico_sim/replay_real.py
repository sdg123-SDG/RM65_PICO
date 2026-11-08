"""Replay a collected session on the REAL RM65 arm (joints + gripper). Records nothing.

Source: the movej_canfd joint commands and gripper commands that were actually sent
during the session (transport.jsonl), restricted to the recorded episodes.

Steps: check no other controller connection -> read state -> movej (10 %) to the first
replayed pose -> wait for Enter -> stream 50 Hz movej_canfd at the recorded timing
(joint speed capped at 30 deg/s, same as teleop) -> stop and hold.
Ctrl+C at any time stops streaming and holds the arm where it is.

Usage: replay_real.py <session folder> [--speed 1.0] [--no-gripper] [--check]
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
sys.path.insert(0, str(PROJECT.parent / "vdo-rm65b"))
from real.json_codec import apply_joint_axis_sign, decode_arm_state, encode_joint_deg  # noqa: E402
from real.json_link import JsonArmLink, parse_real_addr  # noqa: E402

TICK = .02
MAX_SPEED_DEG_S = 30.
SETTLE_DEG = .3
GAP_S = .1  # longer gaps between sent frames = arm was holding; replay holds too


def utc_ns(text):
    return int(datetime.fromisoformat(text).timestamp() * 1e9)


def load(session: Path):
    meta = json.loads((session / "session.json").read_text(encoding="utf-8"))
    windows = [(utc_ns(e["start_utc"]), utc_ns(e["end_utc"])) for e in meta.get("episodes", []) if e.get("end_utc")]
    if not windows:
        raise ValueError("session has no finished episodes")
    inside = lambda t: any(a <= t <= b for a, b in windows)  # noqa: E731
    joints, grips = [], []
    with (session / "transport.jsonl").open(encoding="utf-8") as f:
        for line in f:
            rec = json.loads(line)
            t = rec.get("wall_time_ns")
            if t is None or not inside(t):
                continue
            if rec.get("type") == "sent":
                # transport logs MDH angles; the controller takes apply_joint_axis_sign(MDH)
                joints.append((t, apply_joint_axis_sign(rec["joint_deg"])))
            elif rec.get("type") == "gripper_sent":
                grips.append((t, rec["payload"]))
    if len(joints) < 2:
        raise ValueError("no joint commands inside the episodes")
    t0 = joints[0][0]
    jt = np.array([(t - t0) / 1e9 for t, _ in joints])
    jq = np.array([q for _, q in joints])
    gt = [((t - t0) / 1e9, p) for t, p in grips]
    return jt, jq, gt


def target_at(jt, jq, t):
    i = int(np.searchsorted(jt, t, side="right") - 1)
    if i < 0:
        return jq[0]
    if i >= len(jt) - 1:
        return jq[-1]
    if jt[i + 1] - jt[i] > GAP_S:
        return jq[i]
    a = (t - jt[i]) / (jt[i + 1] - jt[i])
    return jq[i] + a * (jq[i + 1] - jq[i])


def send_raw(link, payload):
    line = json.dumps(payload, separators=(",", ":")).encode() + b"\r\n"
    with link._lock:
        link._sock.sendall(line)


def read_state(link, retries=5):
    # The controller occasionally emits a truncated/non-JSON line (seen right after gripper
    # commands). That is a link glitch, not an arm error: drop the line and ask again.
    for attempt in range(retries):
        try:
            msg = link.rpc({"command": "get_current_arm_state"}, timeout_s=1.)
            break
        except json.JSONDecodeError:
            if attempt == retries - 1:
                raise
            print("[replay] skipped a malformed controller line", flush=True)
    state = decode_arm_state(msg)
    if state.arm_err or state.sys_err:
        raise RuntimeError(f"controller error arm_err={state.arm_err} sys_err={state.sys_err}")
    return np.asarray(state.joint, dtype=float)


def hold(link):
    try:
        q = read_state(link)
        send_raw(link, {"command": "movej", "joint": encode_joint_deg(q), "v": 10, "r": 0, "trajectory_connect": 0})
        print(f"[replay] holding at {np.round(q, 1).tolist()}", flush=True)
    except Exception as exc:
        print(f"[replay] hold command failed ({exc}); use the teach pendant / e-stop if needed", flush=True)


def busy_connections(host, port):
    cmd = (f"@(Get-NetTCPConnection -State Established -RemoteAddress {host} -RemotePort {port} "
           "-ErrorAction SilentlyContinue).Count")
    out = subprocess.run(["powershell.exe", "-NoProfile", "-Command", cmd], capture_output=True, text=True)
    return int(out.stdout.strip() or 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed, 0.1..1.0")
    ap.add_argument("--no-gripper", action="store_true")
    ap.add_argument("--check", action="store_true", help="load and summarize only; no robot connection")
    ap.add_argument("--address")
    args = ap.parse_args()
    if not .1 <= args.speed <= 1.:
        raise SystemExit("--speed must be between 0.1 and 1.0")
    jt, jq, gt = load(args.session)
    if args.no_gripper:
        gt = []
    steps = np.max(np.abs(np.diff(jq, axis=0)), axis=1)
    print(f"[replay] {len(jt)} joint commands, {jt[-1]:.1f} s recorded ({jt[-1] / args.speed:.1f} s at speed {args.speed}), "
          f"{len(gt)} gripper commands; largest step {steps.max():.2f} deg", flush=True)
    print(f"[replay] first pose (controller deg) {np.round(jq[0], 1).tolist()}", flush=True)
    if args.check:
        return
    address = args.address or json.loads((PROJECT / "config/sim_real_bridge.local.json").read_text(encoding="utf-8"))["real_addr"]
    host, port = parse_real_addr(address)
    if busy_connections(host, port):
        raise SystemExit("Another program is connected to the controller (teleop still running?). Close it first.")
    link = JsonArmLink.connect(host, port)
    streaming = False
    try:
        q = read_state(link)
        delta = float(np.max(np.abs(jq[0] - q)))
        print(f"[replay] current pose {np.round(q, 1).tolist()}; max distance to first pose {delta:.1f} deg", flush=True)
        if delta > SETTLE_DEG:
            input(f"Press Enter to move to the first pose at 10% joint speed (Ctrl+C to cancel) ... ")
            send_raw(link, {"command": "movej", "joint": encode_joint_deg(jq[0]), "v": 10, "r": 0, "trajectory_connect": 0})
            deadline, settled = time.monotonic() + 90., 0
            while settled < 3:
                if time.monotonic() > deadline:
                    raise TimeoutError("first pose not reached within 90 s")
                time.sleep(.1)
                err = float(np.max(np.abs(jq[0] - read_state(link))))
                settled = settled + 1 if err <= SETTLE_DEG else 0
            print("[replay] at first pose", flush=True)
        input(f"Press Enter to START the replay ({jt[-1] / args.speed:.0f} s). Ctrl+C stops and holds ... ")
        streaming = True
        goal = read_state(link)
        gi = 0
        began = time.monotonic()
        next_tick = began
        last_state_check = began
        max_step = MAX_SPEED_DEG_S * TICK
        while True:
            now = time.monotonic()
            t = (now - began) * args.speed
            target = target_at(jt, jq, t)
            step = np.clip(target - goal, -max_step, max_step)
            goal = goal + step
            link.send_stream({"command": "movej_canfd", "joint": encode_joint_deg(goal), "follow": False})
            while gi < len(gt) and gt[gi][0] <= t:
                send_raw(link, gt[gi][1])
                print(f"[replay] t={t:5.1f}s gripper {gt[gi][1]['command']} "
                      + str({k: v for k, v in gt[gi][1].items() if k in ('position', 'force')}), flush=True)
                gi += 1
            if now - last_state_check >= .2:
                read_state(link)  # raises on any controller error
                last_state_check = now
            if t >= jt[-1] and np.max(np.abs(jq[-1] - goal)) < 1e-3:
                break
            next_tick += TICK
            time.sleep(max(0., next_tick - time.monotonic()))
        print("[replay] finished", flush=True)
    except KeyboardInterrupt:
        print("\n[replay] stopped by operator", flush=True)
    finally:
        if streaming:
            hold(link)
        time.sleep(.3)
        link.close()


if __name__ == "__main__":
    main()
