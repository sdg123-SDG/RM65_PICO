"""Replay the real 22:37 hand residuals (1 Hz, interpolated to 50 Hz) through the IK pipeline."""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(r"E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim\vr_teleop")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from util.ik import solve_pose_ik  # noqa: E402
from util.base_yaw import HandAzimuthBaseYaw  # noqa: E402
from util.quaternion import matrix_to_quaternion, quaternion_multiply  # noqa: E402

model = mujoco.MjModel.from_xml_path(str(ROOT / "example/scene/scene_rm65.xml"))
site = model.site("rm65_ee_site").id
start = np.deg2rad([-6, -30, -32, 1, -97, -5])  # controller deg == sim q
free = np.array([model.jnt_dofadr[model.joint(f"joint_{i}").id] for i in range(2, 7)])
ranges = np.array([model.jnt_range[model.joint(f"joint_{i}").id] for i in range(1, 7)])
safe = ranges + np.deg2rad([2.0, -2.0])

R = np.loadtxt(HERE / "run_223720_residuals.txt")
fine = []
for a, b in zip(R[:-1], R[1:]):
    for k in np.linspace(0, 1, 50, endpoint=False):
        fine.append(a + k * (b - a))
fine.append(R[-1])


def euler_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def run(mode, rot_w=0.3, home_w=0.01, clamp=False, drag=0.0, soft=0.0, verbose=False):
    data, ik = mujoco.MjData(model), mujoco.MjData(model)
    data.qpos[:6] = start
    mujoco.mj_forward(model, data)
    p0 = data.site_xpos[site].copy()
    q0 = matrix_to_quaternion(data.site_xmat[site].reshape(3, 3))
    yaw = None
    if mode == "hand":
        yaw = HandAzimuthBaseYaw(model)
        yaw.engage(data, p0, float(start[0]))
    q = data.qpos.copy()
    offset = np.zeros(3)
    over = rejected = 0
    errs, joints, first_over, base_dev = [], [], None, []
    for i, r in enumerate(fine):
        target = p0 + 0.5 * r[:3] + offset
        tq = np.array(quaternion_multiply(euler_quat(*r[3:]), q0))
        kw = dict(rot_weight=rot_w, current_q_weight=0.1, home_weight=home_w, skip_tail_joints=0,
                  joint_ranges=safe if clamp else None)
        if yaw is not None and soft > 0:
            q1 = yaw.target(target)
            qn = solve_pose_ik(model, ik, site, target, tq, q, home_qpos=start,
                               joint_prior=(0, q1, soft), **kw)
            base_dev.append(abs(np.rad2deg(qn[0] - q1)))
        elif yaw is None:
            qn = solve_pose_ik(model, ik, site, target, tq, q, home_qpos=start, **kw)
        else:
            qi = q.copy()
            qi[0] = yaw.target(target)
            qn = solve_pose_ik(model, ik, site, target, tq, qi, home_qpos=start[free], dof_indices=free, **kw)
        if np.max(np.abs(np.rad2deg(qn[:6] - q[:6]))) < 30.0:
            q = qn
        else:
            rejected += 1
        if np.any(q[:6] < ranges[:, 0]) or np.any(q[:6] > ranges[:, 1]):
            over += 1
            if first_over is None:
                first_over = (round(i / 50, 1), np.round(np.rad2deg(q[:6]), 1).tolist())
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        ach = data.site_xpos[site].copy()
        e = target - ach
        if drag > 0 and np.linalg.norm(e) > drag:
            offset -= e * (1 - drag / np.linalg.norm(e))
        errs.append(np.linalg.norm(target - ach))
        joints.append(np.rad2deg(q[:6]))
    errs = np.array(errs)
    J = np.array(joints)
    s = (f"越限帧 {over:>4} 首次越限 {first_over}  跳变被拒 {rejected:>3}  "
         f"位置误差 中位 {np.median(errs) * 1000:4.0f} / 最大 {errs.max() * 1000:4.0f} mm")
    if base_dev:
        s += f"  底座偏离方位角 中位 {np.median(base_dev):.1f}° / 最大 {max(base_dev):.1f}°"
    if verbose:
        for t in range(0, len(J), 150):
            s += f"\n   t={t / 50:4.0f}s q={np.round(J[t]).astype(int).tolist()} err={errs[t] * 1000:4.0f}mm"
    return s


if __name__ == "__main__":
    print("实机当时配置(B, rot0.3)        ", run("hand", verbose="-v" in sys.argv))
    print("原方案 IK (rot0.3)             ", run("ik"))
    print("B + 关节限位                   ", run("hand", clamp=True))
    print("B + 限位 + home0.002           ", run("hand", clamp=True, home_w=0.002))
    print("B + 限位 + home0.002 + 拖拽5cm ", run("hand", clamp=True, home_w=0.002, drag=0.05))
    print("原IK + 限位 + home0.002 + 拖拽 ", run("ik", clamp=True, home_w=0.002, drag=0.05))
