"""Build UV-mapped visual OBJs + albedo atlases for RM65-B.

Official URDF/STL has no texcoords. Appearance follows the RM65 MDH figure:
white shells, thin black rings as axial slabs at the joints, blue I/O
panel on the same base face as the antenna nub.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
MESH = ROOT / "meshes"
OUT = MESH / "visual"
TEX = ROOT / "textures"

# Antenna nub on base_link (max-radius vertex), +X / slightly -Y.
ANTENNA_XY = np.array([0.0589, -0.0183], dtype=np.float64)
ANTENNA_DIR = np.array([ANTENNA_XY[0], ANTENNA_XY[1], 0.0])
ANTENNA_DIR /= np.linalg.norm(ANTENNA_DIR)

# Ring stripe lives at V≈0.50 across all U (closed loop).
# Default wrap stays in V∈[0.18, 0.42] (white). After OpenGL's V-flip that
# samples PNG rows 0.58–0.82, which must stay white.
# Panel is blitted at the TOP of the PNG; OBJ V is 1-png so MuJoCo hits it.
V_STRIPE = (0.47, 0.53)
V_WHITE = (0.18, 0.42)
U_PANEL = (0.68, 0.98)
V_PANEL_PNG = (0.02, 0.30)
V_PANEL_UV = (1.0 - V_PANEL_PNG[1], 1.0 - V_PANEL_PNG[0])  # (0.70, 0.98)

# Outer +X I/O plate (skip hole bottoms at smaller x). Antenna nub is beside it.
PANEL_X = 0.0450
PANEL_Y = (-0.024, 0.022)
PANEL_Z = (0.017, 0.056)

# rings: (origin, axis, r_min, r_max, axial_lo, axial_hi)
# Painted slab = from the joint origin, translate along `axis` into [axial_lo, axial_hi],
# keep only the outer tube wall (radius in [r_min, r_max], not caps). Distal ends stay white.
MESH_CFG = {
    "base_link": {
        "src": MESH / "base_link.STL",
        "dst": OUT / "base_link.obj",
        "wrap_axis": 2,
        "rings": [
            # J1 parent: existing latitudinal loop on the top tube (keep; do not retune).
            ((0.0, 0.0, 0.134), (0.0, 0.0, 1.0), 0.038, 0.060, -0.006, 0.006),
        ],
        "panel": True,
    },
    "link_1": {
        "src": MESH / "link_1.STL",
        "dst": OUT / "link_1.obj",
        "wrap_axis": 2,
        "rings": [
            # J2: −Y housing rim at the end face (y≈−15 mm, r≈40 mm).
            ((0.0, -0.015, 0.0), (0.0, -1.0, 0.0), 0.038, 0.042, -0.002, 0.002),
        ],
        "panel": False,
    },
    "link_2": {
        "src": MESH / "link_2.STL",
        "dst": OUT / "link_2.obj",
        "wrap_axis": 0,
        "rings": [
            # J3 only; J2 ring lives on link_1 (child housing / distal cap stay white).
            ((0.256, 0.0, 0.011), (0.0, 0.0, 1.0), 0.032, 0.036, -0.004, 0.004),
        ],
        "panel": False,
    },
    "link_3": {
        "src": MESH / "link_3.STL",
        "dst": OUT / "link_3.obj",
        "wrap_axis": 1,
        "rings": [
            # J3 housing is inward from the joint rim; leave white.
            # J4: distal forearm tube rim at the J4 interface (y≈-128 mm, r≈29 mm)
            ((0.0, -0.1275, 0.0), (0.0, -1.0, 0.0), 0.027, 0.031, -0.0015, 0.0015),
        ],
        "panel": False,
    },
    "link_4": {
        "src": MESH / "link_4.STL",
        "dst": OUT / "link_4.obj",
        "wrap_axis": 2,
        "rings": [
            # J4 child: -Z cylinder rim at the J4 interface (z≈-81 mm, r≈29 mm)
            ((0.0, 0.0, -0.081), (0.0, 0.0, 1.0), 0.027, 0.031, -0.0015, 0.0015),
            # J5 interface with link_5: +Y end of the housing (y≈+12 mm, r≈28 mm).
            # Not the opposite −Y cap at y≈−43 mm.
            ((0.0, 0.012, 0.0), (0.0, 1.0, 0.0), 0.026, 0.030, -0.004, 0.004),
        ],
        "panel": False,
    },
    "link_5": {
        "src": MESH / "link_5.STL",
        "dst": OUT / "link_5.obj",
        "wrap_axis": 1,
        "rings": [
            # J6: −Y distal tube rim at the joint-6 end face (y≈−114 mm, r≈28 mm).
            ((0.0, -0.114, 0.0), (0.0, -1.0, 0.0), 0.027, 0.030, -0.004, 0.004),
        ],
        "panel": False,
    },
    "link_6": {
        "src": MESH / "link_6.STL",
        "dst": OUT / "link_6.obj",
        "wrap_axis": 2,
        "rings": [
            # J6 mating: lower −Z tube rim, 5 mm slab (was on link_5 overlap).
            ((0.0, 0.0, -0.055), (0.0, 0.0, 1.0), 0.027, 0.030, -0.0025, 0.0025),
        ],
        "panel": False,
    },
}

GRIPPER_JOBS = [
    (MESH / "gripper" / "base_link.STL", OUT / "gripper_base_link.obj"),
    (MESH / "gripper" / "Link1.STL", OUT / "gripper_link_1.obj"),
    (MESH / "gripper" / "Link2.STL", OUT / "gripper_link_2.obj"),
    (MESH / "gripper" / "Link3.STL", OUT / "gripper_link_3.obj"),
    (MESH / "gripper" / "Link4.STL", OUT / "gripper_link_4.obj"),
]


def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(raw[84 : 84 + n * 50], dtype=dt)
    return tris["v"].astype(np.float64), tris["n"].astype(np.float64)


def _basis(axis: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a = np.zeros(3)
    a[axis] = 1.0
    helper = np.array([1.0, 0.0, 0.0]) if axis != 0 else np.array([0.0, 1.0, 0.0])
    t1 = np.cross(a, helper)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(a, t1)
    return a, t1, t2


def _frame(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ax = np.asarray(axis, dtype=np.float64)
    ax = ax / np.linalg.norm(ax)
    helper = np.array([1.0, 0.0, 0.0]) if abs(ax[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = np.cross(ax, helper)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(ax, t1)
    return ax, t1, t2


def _fix_u_seam(uv: np.ndarray, mask: np.ndarray | None = None) -> None:
    """Shift U by +1 on triangles that wrap the 0/1 cut."""
    span = uv[:, :, 0].max(axis=1) - uv[:, :, 0].min(axis=1)
    seam = span > 0.5
    if mask is not None:
        seam &= mask
    if np.any(seam):
        low = uv[:, :, 0] < 0.5
        uv[seam[:, None] & low, 0] += 1.0


def cylindrical_uv(verts: np.ndarray, axis: int = 2) -> np.ndarray:
    """verts (T,3,3) -> uv (T,3,2), V along axis, U around it."""
    flat = verts.reshape(-1, 3)
    mn, mx = flat.min(0), flat.max(0)
    center = 0.5 * (mn + mx)
    a, t1, t2 = _basis(axis)
    rel = verts - center
    h = rel @ a
    hmin, hmax = float(h.min()), float(h.max())
    v = (h - hmin) / max(hmax - hmin, 1e-9)
    u = np.arctan2(rel @ t2, rel @ t1) / (2.0 * np.pi) + 0.5
    uv = np.stack([u, np.clip(v, 0.0, 1.0)], axis=-1)
    _fix_u_seam(uv)
    return uv


def _face_normals(verts: np.ndarray) -> np.ndarray:
    n = np.cross(verts[:, 1] - verts[:, 0], verts[:, 2] - verts[:, 0])
    n /= np.clip(np.linalg.norm(n, axis=1, keepdims=True), 1e-12, None)
    return n


def split_by_axial_plane(
    verts: np.ndarray, origin, axis, plane_h: float, eps: float = 2e-5
) -> np.ndarray:
    """Cut triangles that cross axial=plane_h so the iso-line is a real edge loop.

    Without this, the black/white boundary follows the original STL diagonals
    and looks like a sawtooth. J1 looks round because its cylinder already has
    a latitudinal edge loop; J2–J6 do not.
    """
    ax, _, _ = _frame(np.asarray(axis, dtype=np.float64))
    o = np.asarray(origin, dtype=np.float64)
    h = (verts - o) @ ax - plane_h
    sgn = np.sign(h)
    sgn[np.abs(h) < eps] = 0.0
    mixed = (sgn.min(axis=1) * sgn.max(axis=1)) < 0
    if not np.any(mixed):
        return verts
    out: list[np.ndarray] = []
    keep = verts[~mixed]
    if len(keep):
        out.append(keep)
    for t in np.nonzero(mixed)[0]:
        V = verts[t]
        ht, st = h[t], sgn[t]
        poly: list[np.ndarray] = []
        for i in range(3):
            j = (i + 1) % 3
            poly.append(V[i])
            if st[i] * st[j] < 0.0:
                a, b = float(ht[i]), float(ht[j])
                w = a / (a - b)
                poly.append(V[i] * (1.0 - w) + V[j] * w)
        p = np.stack(poly, axis=0)
        for k in range(1, len(p) - 1):
            out.append(np.stack([p[0], p[k], p[k + 1]], axis=0)[None])
    return np.concatenate(out, axis=0)


def wrap_white_uv(verts: np.ndarray, axis: int) -> np.ndarray:
    """Project the shell onto a compact white atlas patch.

    A cylindrical U wrap (0↔1) makes seam triangles span half the atlas.
    Mipmaps then average in the V≈0.50 black stripe, which shows up as a
    jagged vertical streak (e.g. link_6 lower-left). Use a non-wrapping
    transverse projection so U stays inside the white columns.
    """
    flat = verts.reshape(-1, 3)
    center = 0.5 * (flat.min(0) + flat.max(0))
    a, t1, _t2 = _basis(axis)
    rel = verts - center
    h = rel @ a
    hmin, hmax = float(h.min()), float(h.max())
    v_loc = (h - hmin) / max(hmax - hmin, 1e-9)
    t = rel @ t1
    tmax = max(float(np.abs(t).max()), 1e-9)
    u_loc = np.clip(0.5 + 0.5 * (t / tmax), 0.0, 1.0)
    # Keep well inside the white band; after MuJoCo's V-flip this is PNG 0.64–0.82.
    u = 0.30 + 0.20 * u_loc
    v = V_WHITE[0] + 0.70 * (V_WHITE[1] - V_WHITE[0]) * np.clip(v_loc, 0.0, 1.0)
    return np.stack([u, np.clip(v, V_WHITE[0], 0.36)], axis=-1)


def _ring_frame(pts: np.ndarray, normals: np.ndarray, ring: tuple) -> dict:
    origin, axis, r_min, r_max, axial_lo, axial_hi = ring
    ax, t1, t2 = _frame(np.asarray(axis, dtype=np.float64))
    rel = pts - np.asarray(origin, dtype=np.float64)
    axial = rel @ ax
    rad = np.linalg.norm(rel - axial[..., None] * ax, axis=-1)
    nn = normals / np.clip(np.linalg.norm(normals, axis=-1, keepdims=True), 1e-12, None)
    cap = np.abs(nn @ ax)
    mid = 0.5 * (axial_lo + axial_hi)
    hw = max(0.5 * (axial_hi - axial_lo), 0.0015)
    r_mid = 0.5 * (r_min + r_max)
    r_sp = max(0.5 * (r_max - r_min), 0.006)
    dist = np.sqrt(((axial - mid) / hw) ** 2 + ((rad - r_mid) / r_sp) ** 2)
    return {
        "ax": ax, "t1": t1, "t2": t2, "rel": rel, "axial": axial, "rad": rad,
        "cap": cap, "mid": mid, "hw": hw, "r_min": r_min, "r_max": r_max, "dist": dist,
    }


def ring_vertex_uv(pts: np.ndarray, normals: np.ndarray, ring: tuple) -> np.ndarray:
    """UV for an axial slab: U around the joint axis, V follows axial translation.

    Black (V=0.50) only inside [axial_lo, axial_hi]. Caps sample white.
    """
    m = _ring_frame(pts, normals, ring)
    u = np.arctan2(m["rel"] @ m["t2"], m["rel"] @ m["t1"]) / (2.0 * np.pi) + 0.5
    ax_n = (m["axial"] - m["mid"]) / m["hw"]
    stripe_hw = 0.5 * (V_STRIPE[1] - V_STRIPE[0])
    abs_n = np.abs(ax_n)
    v = 0.50 + np.sign(ax_n + 1e-9) * stripe_hw * np.clip(abs_n, 0.0, 1.0)
    v = np.where(
        abs_n > 1.0,
        0.50 + np.sign(ax_n + 1e-9) * (stripe_hw + 0.18 * np.clip(abs_n - 1.0, 0.0, 3.0)),
        v,
    )
    v = np.where(m["cap"] > 0.50, 0.30, v)
    return np.stack([u, np.clip(v, 0.08, 0.64)], axis=-1)


def ring_vertex_score(pts: np.ndarray, normals: np.ndarray, ring: tuple) -> np.ndarray:
    """1 inside a thin axial slab at the joint; 0 on distal ends and the link body.

    Region = translate from `origin` along `axis` into [axial_lo, axial_hi],
    keep outer wall radius in [r_min, r_max]. Halo is only ~1.5× the slab so the
    black/white edge is the iso-axial cut, not a wide torus that paints the body.
    """
    origin, axis, r_min, r_max, axial_lo, axial_hi = ring
    m = _ring_frame(pts, normals, ring)
    pad = 0.40 * m["hw"]
    in_ax = (m["axial"] >= (axial_lo - pad)) & (m["axial"] <= (axial_hi + pad))
    in_r = (m["rad"] >= (r_min - 0.003)) & (m["rad"] <= (r_max + 0.003))
    wall = m["cap"] < 0.50
    inside = in_ax & in_r & wall
    return np.where(inside, 1.0 / (np.abs(m["axial"] - m["mid"]) + 0.002), 0.0)


def panel_mask(verts: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """Front +X I/O plate. All verts on that plane; centroid inside the rectangle.

    Port holes poke through (smaller x / side walls) and stay unpainted so the
    3D connectors show; the remaining plate is a solid navy rectangle.
    """
    nn = normals / np.clip(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12, None)
    c = verts.mean(axis=1)
    on_plane = (np.abs(verts[:, :, 0] - PANEL_X) < 0.005).all(axis=1)
    in_rect = (
        (c[:, 1] > PANEL_Y[0] - 0.002)
        & (c[:, 1] < PANEL_Y[1] + 0.002)
        & (c[:, 2] > PANEL_Z[0] - 0.002)
        & (c[:, 2] < PANEL_Z[1] + 0.002)
    )
    return (nn[:, 0] > 0.70) & (np.abs(nn[:, 2]) < 0.40) & on_plane & in_rect


def panel_uv(verts: np.ndarray) -> np.ndarray:
    """YZ projection of the +X plate. Viewed from +X: left=+Y, up=+Z."""
    y = verts[:, :, 1]
    z = verts[:, :, 2]
    u_loc = np.clip((PANEL_Y[1] - y) / (PANEL_Y[1] - PANEL_Y[0]), 0.0, 1.0)
    v_loc = np.clip((z - PANEL_Z[0]) / (PANEL_Z[1] - PANEL_Z[0]), 0.0, 1.0)
    u = U_PANEL[0] + u_loc * (U_PANEL[1] - U_PANEL[0])
    v = V_PANEL_UV[0] + v_loc * (V_PANEL_UV[1] - V_PANEL_UV[0])
    return np.stack([u, v], axis=-1)


def write_obj(path: Path, verts: np.ndarray, normals: np.ndarray, uv: np.ndarray) -> None:
    t = verts.shape[0]
    vn = np.repeat(normals[:, None, :], 3, axis=1)
    lines = [f"# {path.name} RM65 visual UV", "o visual"]
    for p in verts.reshape(-1, 3):
        lines.append(f"v {p[0]:.8f} {p[1]:.8f} {p[2]:.8f}")
    for p in uv.reshape(-1, 2):
        lines.append(f"vt {p[0]:.6f} {p[1]:.6f}")
    for p in vn.reshape(-1, 3):
        nrm = np.linalg.norm(p)
        p = p / nrm if nrm > 1e-12 else np.array([0.0, 0.0, 1.0])
        lines.append(f"vn {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
    for i in range(t):
        a, b, c = 3 * i + 1, 3 * i + 2, 3 * i + 3
        lines.append(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}")
    path.write_text("\n".join(lines) + "\n")


def _grain(size: int, base: tuple[int, int, int], amp: int, seed: int) -> Image.Image:
    rng = np.random.default_rng(seed)
    arr = np.clip(np.asarray(base, dtype=np.float32) + rng.normal(0.0, amp, (size, size, 3)), 0, 255)
    return Image.fromarray(arr.astype(np.uint8), "RGB")


def _soft_band(y: np.ndarray, a: float, b: float, fade: float) -> np.ndarray:
    rise = np.clip((y - (a - fade)) / max(fade, 1e-8), 0.0, 1.0)
    fall = np.clip(((b + fade) - y) / max(fade, 1e-8), 0.0, 1.0)
    rise = rise * rise * (3.0 - 2.0 * rise)
    fall = fall * fall * (3.0 - 2.0 * fall)
    return rise * fall


def _fill_circle(p: np.ndarray, cy: float, cx: float, r: float, rgb: tuple[int, int, int]) -> None:
    yy, xx = np.ogrid[: p.shape[0], : p.shape[1]]
    p[(yy - cy) ** 2 + (xx - cx) ** 2 < r * r] = rgb


def _fill_rect(p: np.ndarray, y0: float, y1: float, x0: float, x1: float, rgb: tuple[int, int, int]) -> None:
    h, w = p.shape[:2]
    p[int(max(0, y0)) : int(min(h, y1)), int(max(0, x0)) : int(min(w, x1))] = rgb


def _make_panel(ph: int, pw: int) -> np.ndarray:
    """Navy I/O plate only. Ports are already 3D in the STL; drawing them here misaligns."""
    p = np.full((ph, pw, 3), (16, 42, 108), dtype=np.uint8)
    _fill_rect(p, ph * 0.03, ph * 0.97, pw * 0.03, pw * 0.97, (22, 52, 124))
    return p


def build_atlases() -> None:
    """White grain with a horizontal black ring-stripe; base also has a blue panel."""
    TEX.mkdir(parents=True, exist_ok=True)
    n = 1024
    white = np.asarray(_grain(n, (232, 234, 238), 3.0, 1), dtype=np.float32)
    black = np.asarray(_grain(n, (22, 22, 24), 2.0, 2), dtype=np.float32)
    # V=0 at the top of the PNG; stripe centered at V=0.5 so a vertical flip still hits it.
    yy = np.linspace(0.0, 1.0, n, endpoint=False) + 0.5 / n
    band = _soft_band(yy, V_STRIPE[0], V_STRIPE[1], fade=8.0 / n)[:, None, None]
    base = white * (1.0 - band) + black * band

    Image.fromarray(base.astype(np.uint8)).save(TEX / "rm65_shell.png")
    base_img = base.copy()
    u0, u1 = int(U_PANEL[0] * n), int(U_PANEL[1] * n)
    v0, v1 = int(V_PANEL_PNG[0] * n), int(V_PANEL_PNG[1] * n)
    base_img[v0:v1, u0:u1] = _make_panel(v1 - v0, u1 - u0)
    Image.fromarray(base_img.astype(np.uint8)).save(TEX / "rm65_base.png")
    _grain(n, (48, 50, 54), 2.5, 3).save(TEX / "rm65_gripper.png")
    _grain(n, (22, 22, 24), 2.0, 4).save(TEX / "rm65_joint.png")
    print("wrote atlases in", TEX.relative_to(ROOT))


def process_link(name: str, cfg: dict) -> None:
    verts, face_n = load_stl(cfg["src"])
    rings = cfg["rings"]
    # Insert edge loops at each ring's black/white iso-planes before UV.
    for rg in rings:
        origin, axis, _r0, _r1, axial_lo, axial_hi = rg
        mid = 0.5 * (axial_lo + axial_hi)
        hw = max(0.5 * (axial_hi - axial_lo), 0.0015)
        verts = split_by_axial_plane(verts, origin, axis, mid - hw)
        verts = split_by_axial_plane(verts, origin, axis, mid + hw)
    face_n = _face_normals(verts)
    vn = np.repeat(face_n[:, None, :], 3, axis=1)
    uv = wrap_white_uv(verts, int(cfg["wrap_axis"]))
    used = np.zeros(len(verts), dtype=bool)
    if rings:
        # Assign by triangle centroid so a large tri cannot drag the distal end into the ring.
        cent = verts.mean(axis=1)[:, None, :]
        cn = face_n[:, None, :]
        cscores = np.stack([ring_vertex_score(cent, cn, rg)[:, 0] for rg in rings], axis=0)
        assigned = np.where(cscores.max(axis=0) > 0, cscores.argmax(axis=0), -1).astype(np.int32)
        used = assigned >= 0
        for i, rg in enumerate(rings):
            tri = assigned == i
            ntri = int(tri.sum())
            print(f"  {name} ring{i} tris={ntri}")
            if ntri == 0:
                continue
            ruv = ring_vertex_uv(verts, vn, rg)
            _fix_u_seam(ruv, mask=tri)
            uv[tri] = ruv[tri]
    else:
        print(f"  {name} rings=0 (white wrap only)")
    if cfg.get("panel"):
        pan = panel_mask(verts, face_n) & ~used
        print(f"  {name} panel={int(pan.sum())}")
        if np.any(pan):
            uv[pan] = panel_uv(verts[pan])
    write_obj(cfg["dst"], verts, face_n, uv)
    print(f"wrote {cfg['dst'].relative_to(ROOT)}  tris={len(verts)}  {cfg['dst'].stat().st_size / 1024:.0f} KiB")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    build_atlases()
    for name, cfg in MESH_CFG.items():
        process_link(name, cfg)
    for src, dst in GRIPPER_JOBS:
        verts, normals = load_stl(src)
        uv = cylindrical_uv(verts, axis=int(np.argmax(verts.reshape(-1, 3).max(0) - verts.reshape(-1, 3).min(0))))
        write_obj(dst, verts, normals, uv)
        print(f"wrote {dst.relative_to(ROOT)}  tris={len(verts)}  {dst.stat().st_size / 1024:.0f} KiB")


if __name__ == "__main__":
    main()
