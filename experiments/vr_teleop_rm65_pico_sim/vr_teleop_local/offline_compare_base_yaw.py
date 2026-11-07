"""Offline comparison: original full IK vs hand-azimuth base yaw, same wrist targets."""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(r"E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim\vr_teleop")
sys.path.insert(0, str(ROOT))
from util.ik import solve_pose_ik  # noqa: E402
from util.base_yaw import HandAzimuthBaseYaw  # noqa: E402

model = mujoco.MjModel.from_xml_path(str(ROOT / "example/scene/scene_rm65.xml"))
site = model.site("rm65_ee_site").id
home = np.array([0.0, -0.522, -1.81, 0.0099, 0.782, 0.0])
free = np.array([model.jnt_dofadr[model.joint(f"joint_{i}").id] for i in range(2, 7)])


def ee(data):
    return data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()


from util.quaternion import matrix_to_quaternion  # noqa: E402


def quat(mat):
    return np.asarray(matrix_to_quaternion(mat))


def run(mode, path):
    data, ik = mujoco.MjData(model), mujoco.MjData(model)
    data.qpos[:6] = home
    mujoco.mj_forward(model, data)
    p0, r0 = ee(data)
    tq = quat(r0)
    yaw = None
    if mode == "hand":
        yaw = HandAzimuthBaseYaw(model)
        yaw.engage(data, p0, float(data.qpos[0]))
    q = data.qpos.copy()
    err = []
    for dp in path:
        target = p0 + dp
        if yaw is None:
            q = solve_pose_ik(model, ik, site, target, tq, q, current_q_weight=0.1, home_qpos=home, skip_tail_joints=0)
        else:
            qi = q.copy()
            qi[0] = yaw.target(target)
            q = solve_pose_ik(model, ik, site, target, tq, qi, current_q_weight=0.1,
                              home_qpos=home[free], skip_tail_joints=0, dof_indices=free)
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        err.append(np.linalg.norm(ee(data)[0] - target))
    return np.rad2deg(q[:6] - home), max(err), err[-1]
    return np.rad2deg(q[:6] - home), max(err)


steps = np.vstack([np.linspace(0, 1, 60)[:, None], np.ones((60, 1))])
cases = {
    "左移 15cm": steps * [0, 0.15, 0],
    "右移 15cm": steps * [0, -0.15, 0],
    "前移 10cm": steps * [0.10, 0, 0],
    "左前斜 10+10cm 再抬 5cm": steps * [0.10, 0.10, 0.05],
}
for name, path in cases.items():
    for mode in ("ik", "hand"):
        dq, e, ef = run(mode, path)
        print(f"{name:<22} {mode:<5} 关节变化(°) {np.round(dq, 1).tolist()}  过程最大误差 {e * 1000:.1f} mm  停稳后误差 {ef * 1000:.1f} mm")
    print()

# Edge cases of the mapping itself
data = mujoco.MjData(model)
data.qpos[:6] = home
mujoco.mj_forward(model, data)
y = HandAzimuthBaseYaw(model)
y.engage(data, data.site_xpos[site].copy(), 0.0)
print("贴近底座轴(半径<8cm)保持:", y.target(np.array([0.01, 0.01, 0.5])))
print("转到身后被限位:", round(np.rad2deg(y.target(np.array([-0.5, 0.001, 0.3]))), 1))
y.engage(data, np.array([0.0, 0.0, 0.5]), 0.2)
print("在轴心处接合，之后再移开不跳变:", y.target(np.array([0.0, 0.0, 0.5])), round(y.target(np.array([0.5, 0.0, 0.3])), 3))
