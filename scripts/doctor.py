"""Read-only dependency/scene check. No service initialization or robot connection."""
import argparse
import os
from pathlib import Path
import platform
import sys

import mujoco
import numpy as np

from sim_control import SimArm, SCENE
from xr_input import DEFAULT_SDK_DIR, DEFAULT_SERVICE_DIR


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true", help="check Python/model without requiring XR binaries")
    args = parser.parse_args()
    print(f"Python={platform.python_version()} {platform.machine()}; MuJoCo={mujoco.__version__}; NumPy={np.__version__}")
    print(f"Scene={SCENE}")
    arm = SimArm()
    print(f"MODEL OK nq={arm.model.nq} nv={arm.model.nv} nu={arm.model.nu}; TCP={arm.tcp_pos.round(4)}")
    if not args.offline:
        for path in (DEFAULT_SERVICE_DIR / "RoboticsServiceProcess.exe", DEFAULT_SDK_DIR / "PXREARobotSDK.dll"):
            if not path.is_file():
                print(f"MISSING: {path}")
                return 1
        print("XR FILES OK (live input is checked separately in diagnostics mode)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
