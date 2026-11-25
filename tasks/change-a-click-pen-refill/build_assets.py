"""Write the scene's USD assets from geometry.py (needs pxr).

- assets/refill.usda: the new refill: brass point, 3 mm ink tube, the spring stop 22 mm from the point.
- assets/spring.usda: the pen's spring: an open coil modelled as a ring of slats round its bore.
- assets/barrel.usda: the pen's one-piece clear barrel: nose cone, bore, pocket clip.
- assets/clicker.usda: the clicker unit: button cap, guide rings, hollow body, click hook.
- assets/holder.usda: the desk pen stand: a base slab and three tubes.
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


BOX = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]


def box(stage, path, x0, x1, y0, y1, z0, z1, mat):
    pts = [(x, y, z) for x in (x0, x1) for y, z in ((y0, z0), (y1, z0), (y1, z1), (y0, z1))]
    return mesh(stage, path, pts, BOX, mat)


def build_refill():
    stage = stage_for("refill")
    brass = material(stage, "/refill/Looks/brass", (0.80, 0.65, 0.30), 0.3, 0.9)
    ink = material(stage, "/refill/Looks/ink", (0.08, 0.10, 0.16), 0.35, 0.0)
    clear = material(stage, "/refill/Looks/clear", (0.80, 0.78, 0.90), 0.3, 0.0)
    body(stage, "/refill/body", g.REFILL_MASS)
    b = "/refill/body"
    frustum(stage, f"{b}/point", 0.0, g.POINT_TAPER, g.POINT_R0, g.BRASS_R, brass)
    frustum(stage, f"{b}/brass", g.POINT_TAPER, g.POINT_L, g.BRASS_R, g.BRASS_R, brass)
    frustum(stage, f"{b}/tube", g.POINT_L, g.REFILL_L - 0.015, g.TUBE_R, g.TUBE_R, ink)
    frustum(stage, f"{b}/end", g.REFILL_L - 0.015, g.REFILL_L - g.END_TAPER, g.TUBE_R, g.TUBE_R, clear)
    # the end plug, rounded off: it finds the clicker's funnel
    frustum(stage, f"{b}/plug", g.REFILL_L - g.END_TAPER, g.REFILL_L, g.TUBE_R, g.END_R, clear)
    frustum(stage, f"{b}/stop", g.STOP_AT, g.STOP_AT + g.STOP_L, g.STOP_R, g.STOP_R, ink, n=12)
    stage.GetRootLayer().Save()


def build_spring():
    stage = stage_for("spring")
    steel = material(stage, "/spring/Looks/steel", (0.72, 0.73, 0.75), 0.25, 1.0)
    body(stage, "/spring/body", g.SPRING_MASS)
    n, ri, ro, L = g.SPRING_SEGMENTS, g.SPRING_RI, g.SPRING_RO, g.SPRING_L
    lr, ll = g.SPRING_LEAD
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n

        def ring(r_in, r_out):
            return [(r_in * math.cos(t0), r_in * math.sin(t0)), (r_out * math.cos(t0), r_out * math.sin(t0)),
                    (r_out * math.cos(t1), r_out * math.sin(t1)), (r_in * math.cos(t1), r_in * math.sin(t1))]

        # each slat in three lengths: the end turns (the bore opening out and the outside rounding in) and the middle
        rc = ro - g.SPRING_ROUND
        for j, (x0, r0, o0, x1, r1, o1) in enumerate(((-L / 2, ri + lr, rc, -L / 2 + ll, ri, ro),
                                                      (-L / 2 + ll, ri, ro, L / 2 - ll, ri, ro),
                                                      (L / 2 - ll, ri, ro, L / 2, ri + lr, rc))):
            pts = [(x0, y, z) for y, z in ring(r0, o0)] + [(x1, y, z) for y, z in ring(r1, o1)]
            mesh(stage, f"/spring/body/slat{k}_{j}", pts, BOX, steel, offset=0.0002)
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



def barrel_ro(x):
    """The barrel's outside radius at x from the nose tip."""
    if x <= g.NOSE_L:
        return g.NOSE_TIP_RO + (g.FRONT_RO - g.NOSE_TIP_RO) * x / g.NOSE_L
    if x <= g.BACK_START:
        return g.FRONT_RO
    return g.FRONT_RO + (g.BACK_RO - g.FRONT_RO) * (x - g.BACK_START) / (g.BARREL_L - g.BACK_START)


def build_barrel():
    stage = stage_for("barrel")
    clear = material(stage, "/barrel/Looks/clear", (0.35, 0.70, 0.95), 0.2, 0.0)
    clip = material(stage, "/barrel/Looks/clip", (0.20, 0.50, 0.90), 0.3, 0.0)
    body(stage, "/barrel/body", g.BARREL_MASS)
    b, n = "/barrel/body", g.BARREL_SEGMENTS
    shell(stage, f"{b}/nose", 0.0, g.NOSE_L, g.NOSE_HOLE_R, g.FRONT_RI, g.NOSE_TIP_RO, g.FRONT_RO, clear, n)
    # the nose's solid end: a plug filling the cone's bore up to NOSE_SOLID (the 1 mm point hole is left out: nothing
    # goes through it here)
    r_in = g.NOSE_HOLE_R + (g.FRONT_RI - g.NOSE_HOLE_R) * g.NOSE_SOLID / g.NOSE_L
    frustum(stage, f"{b}/nose_plug", 0.0, g.NOSE_SOLID, g.NOSE_HOLE_R + 0.0002, r_in + 0.0002, clear, n)
    shell(stage, f"{b}/front", g.NOSE_L, g.BACK_START, g.FRONT_RI, g.FRONT_RI, g.FRONT_RO, g.FRONT_RO, clear, n)
    x0, x1 = g.BACK_START, g.WIDE_AT
    shell(stage, f"{b}/middle", x0, x1, g.FRONT_RI, g.FRONT_RI, barrel_ro(x0), barrel_ro(x1), clear, n)
    x0, x1 = x1, x1 + g.STEP_L
    shell(stage, f"{b}/step", x0, x1, g.FRONT_RI, g.BACK_RI, barrel_ro(x0), barrel_ro(x1), clear, n)
    x0, x1 = x1, g.BARREL_L - g.MOUTH_CH
    if x1 - x0 > 1e-4:                                   # (no zero-length shell: its flat pieces cook badly)
        shell(stage, f"{b}/back", x0, x1, g.BACK_RI, g.BACK_RI, barrel_ro(x0), barrel_ro(x1), clear, n)
    shell(stage, f"{b}/mouth", x1, g.BARREL_L, g.BACK_RI, g.BACK_RI + g.MOUTH_CH, barrel_ro(x1), g.BACK_RO, clear, n)
    # the pocket clip on the +y side, standing off the barrel except at its root by the back end
    y0 = g.BACK_RO - 0.0002
    box(stage, f"{b}/clip", g.CLIP_FROM, g.BARREL_L - 0.002, y0, y0 + g.CLIP_T, -g.CLIP_W / 2, g.CLIP_W / 2, clip)
    stage.GetRootLayer().Save()


def build_clicker():
    stage = stage_for("clicker")
    blue = material(stage, "/clicker/Looks/blue", (0.12, 0.25, 0.85), 0.4, 0.0)
    body(stage, "/clicker/body", g.CLICKER_MASS)
    b, n = "/clicker/body", 12
    shell(stage, f"{b}/mouth", 0.0, g.MOUTH_L, g.MOUTH_R, g.HOLLOW_R, g.BODY_R, g.BODY_R, blue, n)
    ring_lo = g.CLICKER_L - g.CAP_L - 0.010 - g.RING_L
    shell(stage, f"{b}/body", g.MOUTH_L, ring_lo, g.HOLLOW_R, g.HOLLOW_R, g.BODY_R, g.BODY_R, blue, n)
    shell(stage, f"{b}/ring_lo", ring_lo, ring_lo + g.RING_L, g.HOLLOW_R, g.HOLLOW_R, g.RING_R, g.RING_R, blue, n)
    shell(stage, f"{b}/upper", ring_lo + g.RING_L, g.HOLLOW_L, g.HOLLOW_R, g.HOLLOW_R, g.BODY_R, g.BODY_R, blue, n)
    top = g.CLICKER_L - g.CAP_L
    frustum(stage, f"{b}/floor", g.HOLLOW_L, top - g.RING_L, g.BODY_R, g.BODY_R, blue, n)
    frustum(stage, f"{b}/ring_hi", top - g.RING_L, top, g.RING_R, g.RING_R, blue, n)
    frustum(stage, f"{b}/cap", top, g.CLICKER_L, g.CAP_R, g.CAP_R, blue, n)
    box(stage, f"{b}/hook", g.HOOK_AT, g.HOOK_AT + g.HOOK_L, g.BODY_R - 0.0002, g.HOOK_R, -0.0006, 0.0006, blue)
    stage.GetRootLayer().Save()


def build_holder():
    """The desk pen stand: a base slab and three tubes in a row along y (clicker, barrel, refill). Its own frame:
    origin at the slab's underside centre, x up."""
    stage = stage_for("holder")
    grey = material(stage, "/holder/Looks/grey", (0.35, 0.35, 0.38), 0.6, 0.0)
    body(stage, "/holder/body", 0.2)
    b = "/holder/body"
    box(stage, f"{b}/base", 0.0, g.SLAB_T, -g.BASE_Y / 2, g.BASE_Y / 2, -g.BASE_X / 2, g.BASE_X / 2, grey)
    for name, y in g.SOCKET_Y.items():
        xf = UsdGeom.Xform.Define(stage, f"{b}/{name}_socket")
        xf.AddTranslateOp().Set(Gf.Vec3d(0, y, 0))
        shell(stage, f"{b}/{name}_socket/w", g.SLAB_T, g.SLAB_T + g.SOCKET_H[name], g.SOCKET_RI[name], g.SOCKET_RI[name],
              g.SOCKET_RO[name], g.SOCKET_RO[name], grey, 16)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.usda"):
        old.unlink()
    build_holder()
    build_barrel()
    build_clicker()
    build_refill()
    build_spring()
    print("wrote", sorted(p.name for p in OUT.glob("*.usda")))
