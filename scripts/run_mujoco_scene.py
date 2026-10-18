"""Load and display the RM65 MuJoCo scene (no XR, no real robot)."""

from __future__ import annotations

import time
from pathlib import Path
import os

import mujoco
import mujoco.viewer

SCENE = Path(os.environ.get("RM65_SCENE", str(Path(__file__).resolve().parents[1] / "assets" / "scene" / "custom_rm65_box" / "scene_gemini_gravcomp.xml")))


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    print(f"loaded {SCENE}")
    print(f"nq={model.nq} nu={model.nu} body link_6 id={mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link_6')}")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
