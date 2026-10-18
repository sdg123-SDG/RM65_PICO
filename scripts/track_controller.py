"""Print LEFT PICO controller 6DoF + Grip/Trigger via XRoboToolkit.

Primary teleop hand is left. No PICO Connect / OpenXR.

Usage:
  1. Start root start.bat and choose D (diagnostics; service starts automatically)
  2. On PICO: XRoboToolkit -> PC IP -> WORKING -> Controller Tracking + Send
  3. Read the printed pose and Grip/Trigger values
  4. Press Ctrl+C in the launcher to stop diagnostics and the service
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xr_input import XRoboToolkitInput, start_service_hint  # noqa: E402


def main() -> None:
    print(start_service_hint())
    print("Tracking LEFT controller only. Ctrl+C to stop. No RM65 motion is sent.")
    src = XRoboToolkitInput(hand="left")
    last_diag = 0.0
    last_sample = 0.0
    try:
        while True:
            s = src.sample()
            if s.valid and s.pos is not None and s.quat_wxyz is not None:
                p, q = s.pos, s.quat_wxyz
                line = (
                    f"LEFT: pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                    f"quat_wxyz=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) "
                    f"grip={s.grip:.2f} trig={s.trigger:.2f} menu={int(s.menu)}"
                )
            else:
                line = "LEFT: NO_POSE"
            age = time.time() - s.stamp if s.stamp else -1
            now = time.time()
            if now - last_sample >= 0.5:
                print(f"{line} | age={age:.2f}s svc={src.service_connected}")
                last_sample = now
            if src.last_error:
                print(f"[parse warn] {src.last_error}")
            if now - last_diag >= 1.0:
                print(src.diagnostics_summary())
                last_diag = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        src.close()


if __name__ == "__main__":
    main()
