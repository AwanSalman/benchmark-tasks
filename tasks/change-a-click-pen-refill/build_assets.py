"""Write the scene's USD assets from geometry.py (needs pxr).

- assets/refill.usda: the new refill, one rigid body: point cone, front section, ink tube.
- assets/spring.usda: the pen's spring, one rigid body: an open coil modelled as a ring of slats round its bore.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"


def stage_for(name):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, f"/{name}").GetPrim())
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


def collide(prim, offset=0.0001):
    p = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def body(stage, path, mass):
    xf = UsdGeom.Xform.Define(stage, path)
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    return xf


def mesh(stage, path, pts, faces, mat, offset=0.0001):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    collide(m, offset)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def frustum(stage, path, x0, x1, r0, r1, mat, n=16):
    """A solid of revolution about x from (x0, r0) to (x1, r1), n-sided."""
    a = [2 * math.pi * k / n for k in range(n)]
    pts = [(x0, r0 * math.cos(t), r0 * math.sin(t)) for t in a] + [(x1, r1 * math.cos(t), r1 * math.sin(t)) for t in a]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))] + [[k, (k + 1) % n, n + (k + 1) % n, n + k] for k in range(n)]
    return mesh(stage, path, pts, faces, mat)


def build_refill():
    stage = stage_for("refill")
    brass = material(stage, "/refill/Looks/brass", (0.80, 0.65, 0.30), 0.3, 0.9)
    tube = material(stage, "/refill/Looks/tube", (0.85, 0.86, 0.88), 0.3, 0.6)
    body(stage, "/refill/body", g.REFILL_MASS)
    b = "/refill/body"
    frustum(stage, f"{b}/point", 0.0, g.POINT_L, g.POINT_R0, g.FRONT_R, brass)
    frustum(stage, f"{b}/front", g.POINT_L, g.SHOULDER, g.FRONT_R, g.FRONT_R, brass)
    frustum(stage, f"{b}/shoulder", g.SHOULDER, g.SHOULDER + 0.001, g.FRONT_R, g.TUBE_R, tube)
    frustum(stage, f"{b}/tube", g.SHOULDER + 0.001, g.REFILL_L, g.TUBE_R, g.TUBE_R, tube)
    stage.GetRootLayer().Save()


def build_spring():
    stage = stage_for("spring")
    steel = material(stage, "/spring/Looks/steel", (0.72, 0.73, 0.75), 0.25, 1.0)
    body(stage, "/spring/body", g.SPRING_MASS)
    n, ri, ro, L = g.SPRING_SEGMENTS, g.SPRING_RI, g.SPRING_RO, g.SPRING_L
    lr, ll = g.SPRING_LEAD
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    # each slat in three lengths: a lead-in at each end (the bore opening out) and the middle
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n

        def ring(r_in):
            return [(r_in * math.cos(t0), r_in * math.sin(t0)), (ro * math.cos(t0), ro * math.sin(t0)),
                    (ro * math.cos(t1), ro * math.sin(t1)), (r_in * math.cos(t1), r_in * math.sin(t1))]

        for j, (x0, r0, x1, r1) in enumerate(((-L / 2, ri + lr, -L / 2 + ll, ri), (-L / 2 + ll, ri, L / 2 - ll, ri),
                                              (L / 2 - ll, ri, L / 2, ri + lr))):
            pts = [(x0, y, z) for y, z in ring(r0)] + [(x1, y, z) for y, z in ring(r1)]
            mesh(stage, f"/spring/body/slat{k}_{j}", pts, faces, steel, offset=0.0003)
    stage.GetRootLayer().Save()


def shell(stage, path, x0, x1, ri0, ri1, ro0, ro1, mat, n):
    """A hollow solid of revolution about x, inner radius ri0 -> ri1 and outer ro0 -> ro1 from x0 to x1, as n convex
    wedges."""
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n

        def ring(ri, ro):
            return [(ri * math.cos(t0), ri * math.sin(t0)), (ro * math.cos(t0), ro * math.sin(t0)),
                    (ro * math.cos(t1), ro * math.sin(t1)), (ri * math.cos(t1), ri * math.sin(t1))]

        pts = [(x0, y, z) for y, z in ring(ri0, ro0)] + [(x1, y, z) for y, z in ring(ri1, ro1)]
        mesh(stage, f"{path}_{k}", pts, faces, mat)


def build_barrel():
    stage = stage_for("barrel")
    blue = material(stage, "/barrel/Looks/blue", (0.10, 0.22, 0.55), 0.4, 0.0)
    body(stage, "/barrel/body", g.BARREL_MASS)
    b, n = "/barrel/body", g.BARREL_SEGMENTS
    nose_ro = g.NOSE_TIP_RO + (g.BARREL_RO - g.NOSE_TIP_RO) * 0.6
    x0, x1 = 0.0, g.NOSE_L
    shell(stage, f"{b}/nose", x0, x1, g.NOSE_RI, g.NOSE_RI, g.NOSE_TIP_RO, nose_ro, blue, n)
    x0, x1 = x1, x1 + g.SEAT_STEP
    shell(stage, f"{b}/step", x0, x1, g.NOSE_RI, g.SEAT_RI, nose_ro, nose_ro, blue, n)
    x0, x1 = x1, x1 + g.SEAT_L
    shell(stage, f"{b}/seat", x0, x1, g.SEAT_RI, g.SEAT_RI, nose_ro, g.BARREL_RO, blue, n)
    x0, x1 = x1, x1 + g.FUNNEL_L
    shell(stage, f"{b}/funnel", x0, x1, g.SEAT_RI, g.BARREL_RI, g.BARREL_RO, g.BARREL_RO, blue, n)
    shell(stage, f"{b}/bore", x1, g.BARREL_L - g.MOUTH_CH, g.BARREL_RI, g.BARREL_RI, g.BARREL_RO, g.BARREL_RO, blue, n)
    shell(stage, f"{b}/mouth", g.BARREL_L - g.MOUTH_CH, g.BARREL_L, g.BARREL_RI, g.BARREL_RI + g.MOUTH_CH,
          g.BARREL_RO, g.BARREL_RO, blue, n)
    stage.GetRootLayer().Save()


def build_back():
    stage = stage_for("back")
    blue = material(stage, "/back/Looks/blue", (0.10, 0.22, 0.55), 0.4, 0.0)
    body(stage, "/back/body", g.BACK_MASS)
    b = "/back/body"
    frustum(stage, f"{b}/floor", 0.0, g.BACK_FLOOR, g.BACK_RO, g.BACK_RO, blue)
    shell(stage, f"{b}/bore", g.BACK_FLOOR, g.BACK_L - g.SLEEVE_L, g.BACK_RI, g.BACK_RI, g.BACK_RO, g.BACK_RO, blue, 16)
    shell(stage, f"{b}/sleeve", g.BACK_L - g.SLEEVE_L, g.BACK_L - 0.0015, g.SLEEVE_RI, g.SLEEVE_RI, g.BACK_RO, g.BACK_RO,
          blue, 16)
    shell(stage, f"{b}/chamfer", g.BACK_L - 0.0015, g.BACK_L, g.SLEEVE_RI, g.SLEEVE_RI + g.SLEEVE_CH, g.BACK_RO,
          g.BACK_RO, blue, 16)
    stage.GetRootLayer().Save()


def build_holder():
    """A desk pen stand: a base slab and three sockets, the refill's (7 mm bore), the front barrel's (12 mm) and the
    back half's (16 mm). Its own frame: origin at the slab's underside, x up; sockets at y = +0.02 (refill), -0.02
    (front barrel) and -0.06 (back half)."""
    stage = stage_for("holder")
    grey = material(stage, "/holder/Looks/grey", (0.35, 0.35, 0.38), 0.6, 0.0)
    body(stage, "/holder/body", 0.2)
    b = "/holder/body"
    pts = [(x, y, z) for x in (0.0, 0.004) for y, z in ((-0.075, -0.015), (0.035, -0.015), (0.035, 0.015), (-0.075, 0.015))]
    mesh(stage, f"{b}/base", pts, [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]], grey)
    for name, y, ri, ro in (("refill", 0.02, g.TUBE_R + 0.001, 0.007), ("barrel", -0.02, g.BARREL_RO + 0.001, 0.009),
                            ("back", -0.06, g.BACK_RO + 0.001, 0.011)):
        xf = UsdGeom.Xform.Define(stage, f"{b}/{name}_socket")
        xf.AddTranslateOp().Set(Gf.Vec3d(0, y, 0))
        shell(stage, f"{b}/{name}_socket/w", 0.004, 0.034, ri, ri, ro, ro, grey, 16)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_holder()
    build_back()
    build_refill()
    build_spring()
    build_barrel()
    print("wrote", sorted(p.name for p in OUT.glob("*.usda")))
