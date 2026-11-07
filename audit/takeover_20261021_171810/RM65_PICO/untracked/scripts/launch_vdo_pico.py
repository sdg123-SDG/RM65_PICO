"""Run PICO as an input plugin inside the current vdo-rm65b simulation session."""
from __future__ import annotations
import argparse
import inspect
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def bootstrap(path=None):
    path = Path(path or os.environ.get("RM65_VDO_DIR", str(ROOT.parent / "vdo-rm65b"))).resolve()
    if not (path / "launch_rm65_box_cli.py").is_file():
        raise FileNotFoundError(f"Missing vdo-rm65b: {path}. Run setup_vdo.ps1.")
    sys.path.insert(0, str(path))
    return path


def check_backend(path=None, camera="d435i"):
    bootstrap(path)
    import mujoco
    from simu.box_scene import BoxScene, apply_home_keyframe
    from simu.rm65_arm import Rm65ExtArm
    from vdo_pico import VdoArmAdapter
    box = BoxScene(gravcomp=True, camera=camera)
    model = mujoco.MjModel.from_xml_path(str(box.scene_xml))
    data = mujoco.MjData(model)
    apply_home_keyframe(model, data)
    native = Rm65ExtArm(xml_path=box.scene_xml, model=model, data=data)
    arm = VdoArmAdapter(native)
    print(f"VDO OK scene={box.scene_xml}; nq={model.nq} nv={model.nv} nu={model.nu}")
    print(f"Native={type(native).__name__}; TCP={arm.tcp_site}; offset={arm.tcp_offset}; base={arm.base_position}")
    return box, native, arm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vdo-dir")
    parser.add_argument("--camera", choices=("none","gemini335","d435i"), default="d435i")
    parser.add_argument("--control-mode", choices=("teleop","jog"), default="teleop")
    parser.add_argument("--mode", choices=("position","pose"), default="position")
    parser.add_argument("--frame", choices=("robot",), default="robot")
    parser.add_argument("--speed", type=float, default=.06)
    parser.add_argument("--scale", type=float, default=1.)
    parser.add_argument("--yaw-deg", type=float, default=0.)
    parser.add_argument("--check", action="store_true")
    args, native_args = parser.parse_known_args()
    if not .01 <= args.speed <= .30 or not .05 <= args.scale <= 3:
        parser.error("speed .01.. .30 m/s; scale .05..3")
    if any(x.split("=")[0] in ("--real", "--json-sim", "--xbox", "--no-guard") for x in native_args):
        parser.error("This entry supports native MuJoCo simulation only")
    bootstrap(args.vdo_dir)
    if args.check:
        check_backend(args.vdo_dir, args.camera)
        return 0
    import launch_rm65_box_cli as app
    from vdo_pico import PicoEeServo
    if "controller_factory" not in inspect.signature(app.main).parameters:
        raise RuntimeError("Native launcher plugin hook is missing. Run scripts/prepare_vdo.py.")
    sys.argv = [str(Path(app.__file__))] + native_args
    return app.main(xbox_on=True, gravcomp=True, camera=args.camera, controller_name="pico",
        controller_factory=lambda arm: PicoEeServo(arm, mode=args.control_mode, speed=args.speed,
                                                  scale=args.scale, wrist=args.mode, yaw=args.yaw_deg))


if __name__ == "__main__":
    raise SystemExit(main())
