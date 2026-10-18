"""Tessellate official RMG24 STEP into MuJoCo meshes + MJCF.

Source: res/rm65b/github/Two-Finger-Gripper/Hardware/CAD/RMG24 Claw 3D Model.stp
CAD is millimetres, nearly fully closed. Frame after export (metres):

  origin  mounting-flange centre (CAD x = -55 mm, y = 0, z = 0)
  +Z      along the fingers (away from the flange)
  +Y      opening direction (right finger)

Each finger slides ±32.5 mm (official 65 mm stroke). q = 0 is the CAD pose (closed).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound, TopoDS_Shape
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

ROOT = Path(__file__).resolve().parent
MESH = ROOT / "meshes"
# robotics/res/... from vdo-sim/assets/objects/rmg24 (parents[4] = robotics)
DEFAULT_STEP = (
    ROOT.parents[4]
    / "res/rm65b/github/Two-Finger-Gripper/Hardware/CAD/RMG24 Claw 3D Model.stp"
)

# CAD opening-plane X (mm); fingers sit at about ±10 mm from this plane.
CAD_CX0_MM = -55.0
STROKE_M = 0.065
HALF_STROKE_M = STROKE_M / 2.0
TOTAL_MASS_KG = 0.674  # product spec
LIN_DEFLECTION_MM = 0.25


def _bbox(shape: TopoDS_Shape) -> tuple[float, ...]:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return box.Get()


def _vol_com(shape: TopoDS_Shape) -> tuple[float, tuple[float, float, float]]:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    c = props.CentreOfMass()
    return props.Mass(), (c.X(), c.Y(), c.Z())


def _classify(b: tuple[float, ...], com: tuple[float, float, float]) -> str:
    xmin, _ymin, zmin, xmax, _ymax, zmax = b
    dx = xmax - xmin
    cx, _cy, _cz = com
    straddle = xmin < CAD_CX0_MM - 8.0 and xmax > CAD_CX0_MM + 8.0
    in_band = xmin > CAD_CX0_MM - 28.0 and xmax < CAD_CX0_MM + 28.0
    if dx > 50.0 or straddle:
        return "base"
    if in_band and (zmin >= 64.0 or zmax > 85.0):
        return "left" if cx < CAD_CX0_MM else "right"
    return "base"


def _compound(solids: list[TopoDS_Shape]) -> TopoDS_Compound:
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for solid in solids:
        builder.Add(comp, solid)
    return comp


def _placed(shape: TopoDS_Shape) -> TopoDS_Shape:
    """CAD mm → flange frame mm: translate then Rz(90°) so opening is +Y."""
    trans = gp_Trsf()
    trans.SetTranslation(gp_Vec(-CAD_CX0_MM, 0.0, 0.0))
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), math.pi / 2.0)
    rot.Multiply(trans)
    return BRepBuilderAPI_Transform(shape, rot, True).Shape()


def _write_stl(shape: TopoDS_Shape, path: Path) -> None:
    BRepMesh_IncrementalMesh(shape, LIN_DEFLECTION_MM)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    path.parent.mkdir(parents=True, exist_ok=True)
    if not writer.Write(shape, str(path)):
        raise RuntimeError(f"STL write failed: {path}")


def _inertia(stl_path: Path, density: float) -> tuple[float, np.ndarray, np.ndarray]:
    mesh = trimesh.load_mesh(stl_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    mesh.apply_scale(0.001)  # mm → m
    mesh.density = density
    inertia = np.array(mesh.moment_inertia, dtype=np.float64)
    return float(mesh.mass), np.array(mesh.center_mass, dtype=np.float64), inertia


def _fmt_inertial(mass: float, com: np.ndarray, I: np.ndarray, indent: int = 6) -> str:
    pad = " " * indent
    ixx, iyy, izz = I[0, 0], I[1, 1], I[2, 2]
    ixy, ixz, iyz = I[0, 1], I[0, 2], I[1, 2]
    return (
        f'{pad}<inertial pos="{com[0]:.6g} {com[1]:.6g} {com[2]:.6g}" mass="{mass:.6g}" '
        f'fullinertia="{ixx:.6g} {iyy:.6g} {izz:.6g} {ixy:.6g} {ixz:.6g} {iyz:.6g}"/>'
    )


def _write_mjcf(inertials: dict[str, tuple[float, np.ndarray, np.ndarray]]) -> None:
    ib, il, ir = (inertials[k] for k in ("base", "left", "right"))
    i_base = _fmt_inertial(*ib, indent=6)
    i_left = _fmt_inertial(*il, indent=8)
    i_right = _fmt_inertial(*ir, indent=8)
    xml = f"""<!-- RealMan RMG24 two-finger parallel gripper (MuJoCo).
     Meshes from official STEP (mm) with scale 0.001. Frame: flange origin, +Z fingers, +Y open.
     q=0 closed (CAD pose); each finger travel {HALF_STROKE_M} m (stroke {STROKE_M} m). -->
<mujoco model="rmg24">
  <compiler angle="radian" autolimits="true" balanceinertia="true"/>
  <option gravity="0 0 -9.81"/>
  <default>
    <default class="rmg24">
      <joint limited="true" damping="0.5" armature="0.0002"/>
      <default class="visual">
        <geom contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="collision">
        <geom condim="3" contype="1" conaffinity="1" group="3"
              solref="0.005 1" solimp="0.99 0.999 1e-05" friction="1.2 0.01 0.01"/>
      </default>
    </default>
  </default>

  <asset>
    <material name="rmg24_body" rgba="0.22 0.23 0.25 1" reflectance="0.06" shininess="0.16"
              specular="0.22" roughness="0.62"/>
    <material name="rmg24_finger" rgba="0.18 0.18 0.20 1" reflectance="0.05" shininess="0.12"
              specular="0.16" roughness="0.68"/>
    <material name="rmg24_collision" rgba="1.0 0.28 0.1 0.9"/>
    <mesh name="rmg24_base" file="meshes/base.stl" scale="0.001 0.001 0.001"/>
    <mesh name="rmg24_finger_left" file="meshes/finger_left.stl" scale="0.001 0.001 0.001"/>
    <mesh name="rmg24_finger_right" file="meshes/finger_right.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <body name="rmg24_base" childclass="rmg24">
{i_base}
      <geom name="rmg24_base_visual" type="mesh" mesh="rmg24_base" class="visual" material="rmg24_body"/>
      <geom name="rmg24_base_collision" type="mesh" mesh="rmg24_base" class="collision" material="rmg24_collision"/>
      <site name="rmg24_flange" pos="0 0 0" size="0.001" rgba="0 0 0 0" group="4"/>
      <site name="rmg24_tcp" pos="0 0 0.137" size="0.001" rgba="0 0 0 0" group="4"/>
      <body name="rmg24_finger_left">
        <joint name="rmg24_left" type="slide" axis="0 1 0" range="{-HALF_STROKE_M:.4g} 0"/>
{i_left}
        <geom name="rmg24_left_visual" type="mesh" mesh="rmg24_finger_left" class="visual" material="rmg24_finger"/>
        <geom name="rmg24_left_collision" type="mesh" mesh="rmg24_finger_left" class="collision" material="rmg24_collision"/>
      </body>
      <body name="rmg24_finger_right">
        <joint name="rmg24_right" type="slide" axis="0 1 0" range="0 {HALF_STROKE_M:.4g}"/>
{i_right}
        <geom name="rmg24_right_visual" type="mesh" mesh="rmg24_finger_right" class="visual" material="rmg24_finger"/>
        <geom name="rmg24_right_collision" type="mesh" mesh="rmg24_finger_right" class="collision" material="rmg24_collision"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <fixed name="rmg24_open">
      <joint joint="rmg24_left" coef="-1"/>
      <joint joint="rmg24_right" coef="1"/>
    </fixed>
  </tendon>

  <equality>
    <joint joint1="rmg24_left" joint2="rmg24_right" polycoef="0 -1 0 0 0"/>
  </equality>

  <actuator>
    <position name="rmg24_ctrl" tendon="rmg24_open" kp="400" dampratio="1.0"
              forcerange="-150 150" ctrlrange="0 {STROKE_M}"/>
  </actuator>

  <contact>
    <exclude body1="rmg24_base" body2="rmg24_finger_left"/>
    <exclude body1="rmg24_base" body2="rmg24_finger_right"/>
  </contact>
</mujoco>
"""
    (ROOT / "rmg24.xml").write_text(xml)


def main(step_path: Path = DEFAULT_STEP) -> None:
    if not step_path.is_file():
        raise FileNotFoundError(step_path)

    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {step_path}")
    reader.TransferRoots()
    shape = reader.OneShape()

    solids: list[TopoDS_Shape] = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        solids.append(exp.Current())
        exp.Next()

    groups: dict[str, list[TopoDS_Shape]] = {"base": [], "left": [], "right": []}
    for solid in solids:
        groups[_classify(_bbox(solid), _vol_com(solid)[1])].append(solid)
    print(
        "solids",
        {k: len(v) for k, v in groups.items()},
        "from",
        step_path.name,
    )

    MESH.mkdir(parents=True, exist_ok=True)
    stl_paths = {
        "base": MESH / "base.stl",
        "left": MESH / "finger_left.stl",
        "right": MESH / "finger_right.stl",
    }
    for name, parts in groups.items():
        _write_stl(_placed(_compound(parts)), stl_paths[name])

    volumes = []
    for path in stl_paths.values():
        mesh = trimesh.load_mesh(path, force="mesh")
        if not isinstance(mesh, trimesh.Trimesh):
            mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
        mesh.apply_scale(0.001)
        volumes.append(float(mesh.volume))
    density = TOTAL_MASS_KG / sum(volumes)
    print(f"volume_m3={sum(volumes):.6e} density={density:.1f} kg/m^3")

    inertials = {name: _inertia(path, density) for name, path in stl_paths.items()}
    for name, (mass, com, _) in inertials.items():
        print(f"  {name}: mass={mass:.4f} kg  com={com}")
    _write_mjcf(inertials)
    print("wrote", ROOT / "rmg24.xml")


if __name__ == "__main__":
    main()
