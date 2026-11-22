"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/aid_left.usda, assets/aid_right.usda: a behind-the-ear hearing aid, a free articulation: its shell with ear
  hook, and its battery door on a hinge ``door_joint`` at the bottom end (a nail lip on its back to open it).
- assets/case.usda: the drying case: a box with an open tray in its top and its lid standing open; one rigid body.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

SHELL = (0.78, 0.72, 0.64)        # beige
HOOK = (0.85, 0.83, 0.78)
CELL = (0.70, 0.50, 0.25)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage


def material(stage, path, color, roughness=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def collide(prim, offset=0.0005):
    p = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def body(stage, path, mass, pos=(0, 0, 0), root=False):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    if root:
        UsdPhysics.ArticulationRootAPI.Apply(p)
        p.AddAppliedSchema("PhysxArticulationAPI")
        p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
        p.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        p.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    return xf


def box(stage, path, lo, hi, mat, coll=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
    bind(c, mat)
    if coll:
        collide(c)
    return c


def prism(stage, path, outline, z0, z1, mat, coll=True):
    """A convex prism: a convex polygon (x, y) in the x-y plane, from z0 to z1."""
    pts = [Gf.Vec3f(float(x), float(y), float(z0)) for x, y in outline] + \
          [Gf.Vec3f(float(x), float(y), float(z1)) for x, y in outline]
    n = len(outline)
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    if coll:
        collide(m)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def cylinder(stage, path, axis, centre, radius, height, mat, coll=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
    bind(c, mat)
    if coll:
        collide(c)
    return c


def capsule_between(stage, path, a, b, radius, mat):
    """A capsule from point a to point b (both in the x-y plane, z = 0)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(L)
    c.CreateAxisAttr("X")
    mid = (a + b) / 2
    c.AddTranslateOp().Set(Gf.Vec3d(float(mid[0]), float(mid[1]), 0.0))
    c.AddRotateZOp().Set(math.degrees(math.atan2(d[1], d[0])))
    bind(c, mat)
    collide(c)
    return c


# -- the hearing aid ---------------------------------------------------------------------------------------------------
def build_aid(side: str):
    name = f"aid_{side}"
    stage = stage_for(name)
    r = f"/{name}"
    shell = material(stage, f"{r}/Looks/shell", SHELL, 0.45)
    hook = material(stage, f"{r}/Looks/hook", HOOK, 0.2)
    cell = material(stage, f"{r}/Looks/cell", CELL, 0.3, 0.8)
    mark = material(stage, f"{r}/Looks/mark", (0.25, 0.25, 0.27), 0.4)   # the maker's logo

    L, D, T, dl = g.AID_L, g.AID_D, g.AID_T, g.DOOR_L
    body(stage, f"{r}/body", g.AID_MASS - g.DOOR_MASS, root=True)
    b = f"{r}/body"
    # the shell above the door: its back curves in toward the top (a gentle banana)
    prism(stage, f"{b}/shell", [(dl, -D / 2), (L - 0.008, -D / 2), (L, -D / 2 + 0.004), (L, D / 2 - 0.001),
                                (L - 0.004, D / 2), (dl, D / 2)], -T / 2, T / 2, shell)
    # the ear hook: a curved tube from the top
    for i, (a, c) in enumerate(zip(g.HOOK_ARC, g.HOOK_ARC[1:])):
        capsule_between(stage, f"{b}/hook{i}", a, c, g.HOOK_R, hook)
    # a logo dot on each side, near the top
    for s, z in (("up", T / 2), ("down", -T / 2)):
        cylinder(stage, f"{b}/mark_{s}", "Z", (L - 0.008, 0.0, z), 0.0018, 0.0004, mark, coll=False)

    # the battery door: the bottom 10 mm of the shell, hinged at its front corner, with a nail tab on its back
    hx, hy = g.HINGE
    body(stage, f"{r}/door", g.DOOR_MASS, pos=(hx, hy, 0.0))
    d = f"{r}/door"
    prism(stage, f"{d}/shell", [(0.0005 - hx, -D / 2 - hy), (dl - hx, -D / 2 - hy), (dl - hx, D / 2 - hy),
                                (0.0 - hx, D / 2 - hy - 0.0005), (0.0 - hx, -D / 2 - hy + 0.0015)],
          -T / 2 + 0.0003, T / 2 - 0.0003, shell)
    tt, th, to = g.TAB
    box(stage, f"{d}/tab", (-to - hx, -D / 2 - hy, -th / 2), (0.0005 - hx, -D / 2 + tt - hy, th / 2), shell)
    # the cell (size 13) lies in the door, seen when it is open
    cylinder(stage, f"{d}/cell", "Z", (dl - 0.0040 - hx, 0.0 - hy, 0.0), 0.0039, 0.0054, cell, coll=False)

    j = UsdPhysics.RevoluteJoint.Define(stage, f"{r}/door_joint")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(b)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(d)])
    j.CreateLocalPos0Attr(Gf.Vec3f(hx, hy, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-g.DOOR_LIMIT)          # opening turns it the negative way about z
    j.CreateUpperLimitAttr(0.0)
    dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    dr.CreateTypeAttr("force")
    dr.CreateTargetPositionAttr(0.0)
    stage.GetRootLayer().Save()


# -- the drying case ---------------------------------------------------------------------------------------------------
def build_case():
    stage = stage_for("case")
    r = "/case"
    white = material(stage, f"{r}/Looks/white", (0.93, 0.93, 0.92), 0.5)
    grey = material(stage, f"{r}/Looks/grey", (0.55, 0.57, 0.60), 0.6)
    smoke = material(stage, f"{r}/Looks/smoke", (0.35, 0.40, 0.48), 0.2)
    body(stage, f"{r}/body", g.CASE_MASS)
    b = f"{r}/body"
    X, Y, H = g.CASE_SIZE
    wx, wy, wd = g.WELL
    floor = H - wd
    box(stage, f"{b}/base", (-X / 2, -Y / 2, 0.0), (X / 2, Y / 2, floor), white)
    # the walls round the tray
    for i, (x0, x1) in enumerate(((-X / 2, -wx / 2), (wx / 2, X / 2))):
        box(stage, f"{b}/wall{i}", (x0, -Y / 2, floor), (x1, Y / 2, H), white)
    for s in ("tray",):
        cx = 0.0
        for j, (y0, y1) in enumerate(((-Y / 2, -wy / 2), (wy / 2, Y / 2))):
            box(stage, f"{b}/end_{s}{j}", (cx - wx / 2, y0, floor), (cx + wx / 2, y1, H), white)
        mark = grey
        box(stage, f"{b}/floor_{s}", (cx - wx / 2 + 0.001, -wy / 2 + 0.001, floor), (cx + wx / 2 - 0.001, wy / 2 - 0.001,
                                                                                    floor + 0.0004), grey, coll=False)
        # a coloured rim round the well
        for j, (lo, hi) in enumerate((((cx - wx / 2 - 0.002, -wy / 2 - 0.002), (cx + wx / 2 + 0.002, -wy / 2)),
                                      ((cx - wx / 2 - 0.002, wy / 2), (cx + wx / 2 + 0.002, wy / 2 + 0.002)),
                                      ((cx - wx / 2 - 0.002, -wy / 2), (cx - wx / 2, wy / 2)),
                                      ((cx + wx / 2, -wy / 2), (cx + wx / 2 + 0.002, wy / 2)))):
            box(stage, f"{b}/rim_{s}{j}", (lo[0], lo[1], H), (hi[0], hi[1], H + 0.0004), mark, coll=False)
    # the lid: hinged along the back (+x) top edge, standing open past upright
    a = math.radians(g.LID_OPEN)
    lid = UsdGeom.Xform.Define(stage, f"{b}/lid")
    lid.AddTranslateOp().Set(Gf.Vec3d(X / 2, 0.0, H))
    lid.AddRotateYOp().Set(g.LID_OPEN)
    box(stage, f"{b}/lid/plate", (-X, -Y / 2, 0.0), (0.0, Y / 2, g.LID_T), smoke)
    # a button and a light on the front
    cylinder(stage, f"{b}/button", "X", (-X / 2 - 0.001, -0.025, H / 2), 0.004, 0.002, grey, coll=False)
    cylinder(stage, f"{b}/light", "X", (-X / 2 - 0.0005, 0.025, H / 2), 0.0015, 0.001,
             material(stage, f"{r}/Looks/green", (0.1, 0.8, 0.3), 0.3), coll=False)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for side in ("left", "right"):
        build_aid(side)
    build_case()
    print("wrote", sorted(p.name for p in OUT.glob("*.usda")))
