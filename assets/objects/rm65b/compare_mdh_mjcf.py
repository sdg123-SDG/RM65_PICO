#!/usr/bin/env python3
"""Compare official RM65-B modified D-H FK against the MJCF/URDF joint chain.

Official MDH: https://develop.realman-robotics.com/robot/robotParameter/RM65OntologyParameters/
  model angle = joint angle + offset
MJCF q is the mechanical joint angle; J2/J3 90° offsets are baked into body quats.

Usage (from any cwd):
  python3 assets/objects/rm65b/compare_mdh_mjcf.py
"""

from __future__ import annotations

import math

PI = math.pi
PI2 = PI / 2
URDF_HALF_PI = 1.5708  # URDF/MJCF wrote π/2 as 1.5708

# Official MDH (m, rad): (a_{i-1}, α_{i-1}, d_i, offset_i)
MDH = (
    (0.0, 0.0, 0.2405, 0.0),
    (0.0, PI2, 0.0, PI2),
    (0.256, 0.0, 0.0, PI2),
    (0.0, PI2, 0.210, 0.0),
    (0.0, -PI2, 0.0, 0.0),
    (0.0, PI2, 0.144, 0.0),
)

# MJCF/URDF parent→child at q=0: (xyz, rpy), then hinge about +Z by q
MJCF_ORIGINS_URDF_PI = (
    ((0.0, 0.0, 0.2405), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (URDF_HALF_PI, -URDF_HALF_PI, 0.0)),
    ((0.256, 0.0, 0.0), (0.0, 0.0, URDF_HALF_PI)),
    ((0.0, -0.21, 0.0), (URDF_HALF_PI, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-URDF_HALF_PI, 0.0, 0.0)),
    ((0.0, -0.144, 0.0), (URDF_HALF_PI, 0.0, 0.0)),
)
MJCF_ORIGINS_EXACT_PI = (
    ((0.0, 0.0, 0.2405), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (PI2, -PI2, 0.0)),
    ((0.256, 0.0, 0.0), (0.0, 0.0, PI2)),
    ((0.0, -0.21, 0.0), (PI2, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-PI2, 0.0, 0.0)),
    ((0.0, -0.144, 0.0), (PI2, 0.0, 0.0)),
)

CONFIGS_DEG = {
    "mechanical zero q=0": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "q2=30deg": (0.0, 30.0, 0.0, 0.0, 0.0, 0.0),
    "q3=45deg": (0.0, 0.0, 45.0, 0.0, 0.0, 0.0),
    "wrist sing [0,60,30,0,0,0]": (0.0, 60.0, 30.0, 0.0, 0.0, 0.0),
    "elbow sing [-90,60,0,0,90,0]": (-90.0, 60.0, 0.0, 0.0, 90.0, 0.0),
    "shoulder [90,43.4,-105.7,0,-30,0]": (90.0, 43.4, -105.7, 0.0, -30.0, 0.0),
}


def rx(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def ry(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rz(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def eye():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def mmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mv(r, v):
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def tmul(a, b):
    ra, ta = a
    rb, tb = b
    return (mmul(ra, rb), tuple(mv(ra, tb)[i] + ta[i] for i in range(3)))


def trans(x, y, z):
    return (eye(), (x, y, z))


def rpy_t(xyz, rpy):
    roll, pitch, yaw = rpy
    rot = mmul(mmul(rz(yaw), ry(pitch)), rx(roll))
    return (rot, xyz)


def rot_err_deg(r1, r2) -> float:
    rt = tuple(tuple(r1[j][i] for j in range(3)) for i in range(3))
    rel = mmul(rt, r2)
    tr = rel[0][0] + rel[1][1] + rel[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def fmt(v) -> tuple:
    return tuple(round(x, 6) for x in v)


def mdh_link(a, alpha, d, theta):
    """Craig MDH: RotX(α)*TransX(a)*RotZ(θ)*TransZ(d)."""
    return tmul(tmul(tmul((rx(alpha), (0.0, 0.0, 0.0)), trans(a, 0.0, 0.0)), (rz(theta), (0.0, 0.0, 0.0))), trans(0.0, 0.0, d))


def fk_mdh(q, use_offset: bool = True):
    acc = (eye(), (0.0, 0.0, 0.0))
    frames = []
    for i, (a, alpha, d, off) in enumerate(MDH):
        theta = q[i] + (off if use_offset else 0.0)
        acc = tmul(acc, mdh_link(a, alpha, d, theta))
        frames.append(acc)
    return frames


def fk_mjcf(q, origins):
    acc = (eye(), (0.0, 0.0, 0.0))
    frames = []
    for (xyz, rpy), qi in zip(origins, q):
        acc = tmul(acc, rpy_t(xyz, rpy))
        acc = tmul(acc, (rz(qi), (0.0, 0.0, 0.0)))
        frames.append(acc)
    return frames


def deg6_to_rad(q_deg):
    return tuple(math.radians(x) for x in q_deg)


def main() -> None:
    print("=== Scalar DH vs MJCF body origins ===")
    print("d1=240.5 mm  MJCF link_1 pos z=0.2405")
    print("a2=256 mm    MJCF link_3 pos x=0.256")
    print("d4=210 mm    MJCF link_4 pos y=-0.21  (Rx(90)*TransZ(d4) => -Y)")
    print("d6=144 mm    MJCF link_6 pos y=-0.144")
    print("alpha baked into body quat; J2/J3 offset=90 baked into body quat (q=0 == mechanical zero)")

    print("\n=== FK: MDH(θ=q+offset) vs MJCF(q)  [mm, deg] ===")
    hdr = f"{'config':<42} {'dpos mm':>10} {'dR deg':>10}  TCP_mdh  TCP_mjcf"
    print(hdr)
    for name, q_deg in CONFIGS_DEG.items():
        q = deg6_to_rad(q_deg)
        mdh = fk_mdh(q, True)
        mj = fk_mjcf(q, MJCF_ORIGINS_URDF_PI)
        mj_e = fk_mjcf(q, MJCF_ORIGINS_EXACT_PI)
        dp = dist(mdh[-1][1], mj[-1][1]) * 1000.0
        dpe = dist(mdh[-1][1], mj_e[-1][1]) * 1000.0
        dr = rot_err_deg(mdh[-1][0], mj[-1][0])
        dre = rot_err_deg(mdh[-1][0], mj_e[-1][0])
        print(f"{name:<42} {dp:10.4f} {dr:10.6f}  {fmt(mdh[-1][1])}  {fmt(mj[-1][1])}")
        print(f"{'  (exact pi/2 origins)':<42} {dpe:10.4f} {dre:10.6f}")

    print("\n=== Per-joint origin at q=0: MDH(offset) vs MJCF ===")
    q0 = (0.0,) * 6
    mdh = fk_mdh(q0, True)
    mj = fk_mjcf(q0, MJCF_ORIGINS_URDF_PI)
    for i in range(6):
        dp = dist(mdh[i][1], mj[i][1]) * 1000.0
        dr = rot_err_deg(mdh[i][0], mj[i][0])
        print(
            f"  after J{i + 1}: dpos={dp:.4f} mm  dR={dr:.6f} deg  "
            f"MDH={fmt(mdh[i][1])} MJCF={fmt(mj[i][1])}"
        )

    print("\n=== MDH WITHOUT offset vs MJCF q=0 (should NOT match) ===")
    mdh0 = fk_mdh(q0, False)
    dp = dist(mdh0[-1][1], mj[-1][1]) * 1000.0
    dr = rot_err_deg(mdh0[-1][0], mj[-1][0])
    print(f"  TCP dpos={dp:.3f} mm dR={dr:.3f} deg  MDH_model0={fmt(mdh0[-1][1])}")

    print("\n=== Reach check ===")
    p = mdh[-1][1]
    reach = math.sqrt(sum(x * x for x in p)) * 1000.0
    print(f"  TCP at mech zero |p|={reach:.1f} mm  d1+a2+d4+d6={240.5 + 256 + 210 + 144}")
    print(f"  workspace R from J2 = a2+d4+d6={256 + 210 + 144} mm (official 610)")

    print("\n=== Joint_2 origin vs MDH Rx(90)Rz(90) ===")
    r_j2 = rpy_t((0.0, 0.0, 0.0), (URDF_HALF_PI, -URDF_HALF_PI, 0.0))[0]
    r_mdh = mmul(rx(PI2), rz(PI2))
    print(f"  dR URDF 1.5708 vs exact MDH offset: {rot_err_deg(r_j2, r_mdh):.6f} deg")
    r_j2e = rpy_t((0.0, 0.0, 0.0), (PI2, -PI2, 0.0))[0]
    print(f"  dR exact pi/2 vs MDH: {rot_err_deg(r_j2e, r_mdh):.6e} deg")

    print("\n=== Flange extra (MJCF only, not in MDH/URDF) ===")
    print("  body flange pos=0.00 -0.01 0.03 quat=1 1 -1 1  relative to link_6")
    w, x, y, z = 1.0, 1.0, -1.0, 1.0
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    rfl = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )
    print(f"  flange local trans=(0,-0.01,0.03) rot angle from I: {rot_err_deg(eye(), rfl):.3f} deg")


if __name__ == "__main__":
    main()
