"""Replay recorded 1 Hz hand residuals through scheme-B IK variants. Usage: replay2.py file [file...]"""
import math
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(r"E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim\vr_teleop")
sys.path.insert(0, str(ROOT))
from util.ik import solve_pose_ik  # noqa: E402
from util.base_yaw import HandAzimuthBaseYaw  # noqa: E402
from util.quaternion import matrix_to_quaternion, quaternion_multiply  # noqa: E402

model = mujoco.MjModel.from_xml_path(str(ROOT / "example/scene/scene_rm65.xml"))
site = model.site("rm65_ee_site").id
free = np.array([model.jnt_dofadr[model.joint(f"joint_{i}").id] for i in range(2, 7)])
ranges = np.array([model.jnt_range[model.joint(f"joint_{i}").id] for i in range(1, 7)])
safe = ranges + np.deg2rad([2.0, -2.0])


def euler_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def load(path):
    head = Path(path).read_text(encoding="utf8").splitlines()[0]
    m = re.search(r"start joints deg:\s*([-\d.\s]+)", head)
    start = np.deg2rad([float(v) for v in m.group(1).split()]) if m else np.deg2rad([-6, -30, -32, 1, -97, -5])
    R = np.loadtxt(path)
    fine = [a + k * (b - a) for a, b in zip(R[:-1], R[1:]) for k in np.linspace(0, 1, 50, endpoint=False)]
    fine.append(R[-1])
    return start, fine


def run(path, rot_w=0.3, home_w=0.002, drag=0.0, ws=None, label=""):
    start, fine = load(path)
    data, ik = mujoco.MjData(model), mujoco.MjData(model)
    data.qpos[:6] = start
    mujoco.mj_forward(model, data)
    p0 = data.site_xpos[site].copy()
    q0 = matrix_to_quaternion(data.site_xmat[site].reshape(3, 3))
    yaw = HandAzimuthBaseYaw(model)
    yaw.engage(data, p0, float(start[0]))
    q = data.qpos.copy()
    offset = np.zeros(3)
    xs, j1, err_cmd, err_hand = [], [], [], []
    for r in fine:
        hand_target = p0 + 0.5 * r[:3]
        target = hand_target + offset
        if ws is not None:
            target = yaw.clamp_workspace(target, **ws)
        tq = np.array(quaternion_multiply(euler_quat(*r[3:]), q0))
        qi = q.copy()
        qi[0] = yaw.target(target)
        qn = solve_pose_ik(model, ik, site, target, tq, qi, rot_weight=rot_w, current_q_weight=0.1,
                           home_weight=home_w, home_qpos=start[free], skip_tail_joints=0,
                           dof_indices=free, joint_ranges=safe)
        if np.max(np.abs(np.rad2deg(qn[:6] - q[:6]))) < 30.0:
            q = qn
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        ach = data.site_xpos[site].copy()
        e = target - ach
        if drag > 0 and np.linalg.norm(e) > drag:
            offset -= e * (1 - drag / np.linalg.norm(e))
        xs.append(ach.copy())
        j1.append(math.degrees(q[0]))
        err_cmd.append(np.linalg.norm(e))
        err_hand.append(np.linalg.norm(hand_target - ach))
    X = np.array(xs)
    hx = np.array([p0[0] + 0.5 * r[0] for r in fine])
    # "wrong-way" motion: arm moves forward (+x) while the hand-commanded x moves backward, per 1 s window
    wrong = 0.0
    for i in range(0, len(X) - 50, 50):
        dh, da = hx[i + 50] - hx[i], X[i + 50, 0] - X[i, 0]
        if dh < -0.01 and da > 0.005:
            wrong += da
    return (f"{label:<30} 末端x {X[:, 0].min():.2f}~{X[:, 0].max():.2f} m  J1 {min(j1):5.0f}~{max(j1):4.0f}°  "
            f"对IK目标误差 中位 {np.median(err_cmd) * 1000:3.0f} mm  反向前冲累计 {wrong * 100:4.1f} cm")


if __name__ == "__main__":
    WS = dict(min_radius_m=0.25, max_reach_m=0.68, shoulder_height_m=0.0)
    for f in sys.argv[1:]:
        print(Path(f).name)
        print(run(f, drag=0.05, label="  上次实机版本(拖拽5cm)"))
        print(run(f, label="  去掉拖拽"))
        print(run(f, ws=WS, label="  去掉拖拽+工作空间限制"))
        print(run(f, ws=WS, rot_w=0.15, label="  去掉拖拽+工作空间+rot0.15"))
