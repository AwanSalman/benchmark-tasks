"""Write the scene's USD assets from geometry.py (run with Isaac Sim's Python: needs pxr and scipy).

    python build_assets.py

- assets/kettle.usda: one free rigid body. Wall (with the spout hole), floor, top with the lid opening, the handle, and the filter's slot behind the spout: two guide lips that hold the filter's side edges against
  the wall's inside, a side wall beyond each edge (both flared open at the top), and a stop ledge.
- assets/lid.usda: the lid, a disc with a locating plug under it and a knob on top.
- assets/base.usda: the kettle's power base, a low puck.
- assets/filter.usda: one free rigid body, the curved filter plate (as flat strips) and its grip tab.

Everything is a convex hull built here from the numbers in geometry.py; no third-party meshes.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
STEEL = (0.78, 0.79, 0.80)
INSIDE = (0.62, 0.63, 0.64)
BLACK = (0.08, 0.08, 0.09)
FILTER = (0.30, 0.55, 0.85)


def stage_for(name):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    st = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, f"/{name}").GetPrim())
    return st


def material(st, path, color, rough=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def phys_material(st, path, s, d):
    mat = UsdShade.Material.Define(st, path)
    m = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    m.CreateStaticFrictionAttr(s)
    m.CreateDynamicFrictionAttr(d)
    m.CreateRestitutionAttr(0.0)
    return mat


def rigid(st, path, mass):
    xf = UsdGeom.Xform.Define(st, path)
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    return xf


def hull(st, path, pts, look, phys, offset=0.0006):
    from scipy.spatial import ConvexHull

    pts = np.asarray(pts, float)
    h = ConvexHull(pts)
    faces = []
    for simplex, eq in zip(h.simplices, h.equations):
        a, b, c = pts[simplex]
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
            simplex = simplex[[0, 2, 1]]
        faces.append(list(simplex))
    m = UsdGeom.Mesh.Define(st, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([3] * len(faces))
    m.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    api = UsdShade.MaterialBindingAPI.Apply(m.GetPrim())
    api.Bind(look)
    api.Bind(phys, materialPurpose="physics")
    p = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    return m


def box(x0, x1, y0, y1, z0, z1):
    return [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


def polar(r, a, z):
    return (r * math.cos(a), r * math.sin(a), z)


def build_kettle():
    st = stage_for("kettle")
    r = "/kettle"
    steel = material(st, f"{r}/Looks/steel", STEEL, 0.3, 0.8)
    inside = material(st, f"{r}/Looks/inside", INSIDE, 0.4, 0.6)
    black = material(st, f"{r}/Looks/black", BLACK, 0.5)
    pp = phys_material(st, f"{r}/Looks/pp", *g.KETTLE_FRICTION)
    b = f"{r}/body"
    rigid(st, b, g.KETTLE_MASS)

    # wall, with the spout hole cut through it at the front
    n = 96
    hole_a = math.asin(g.HOLE_HALF_W / g.R_IN)
    hz0, hz1 = g.HOLE_Z
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        am = (a0 + a1) / 2
        e0, e1 = am - 0.53 * (a1 - a0), am + 0.53 * (a1 - a0)
        bands = [(0.0, g.H)]
        if abs(math.remainder(am, 2 * math.pi)) < hole_a:
            bands = [(0.0, hz0), (hz1, g.H)]
        for j, (z0, z1) in enumerate(bands):
            pts = [polar(rr, a, z) for rr in (g.R_IN, g.R_OUT) for a in (e0, e1) for z in (z0, z1)]
            hull(st, f"{b}/wall{k:02d}_{j}", pts, steel, pp)
    disc = [polar(g.R_OUT, 2 * math.pi * k / 32, 0.0)[:2] for k in range(32)]
    hull(st, f"{b}/floor", [(x, y, z) for x, y in disc for z in (0.0, g.FLOOR)], inside, pp)
    # spout: a short open trough outside the hole
    hy = g.HOLE_HALF_W + 0.003
    hull(st, f"{b}/spout", [(x, y, z) for x, z in ((g.R_OUT - 0.002, hz0 - 0.004), (g.R_OUT - 0.002, hz0),
                                                     (g.R_OUT + 0.028, hz0 + 0.014), (g.R_OUT + 0.028, hz0 + 0.010))
                            for y in (-hy, hy)], steel, pp)

    # the filter's slot
    phi = g.half_angle()
    over, side, wall = g.LIP_OVER / g.R_IN, g.SIDE_PLAY / g.R_IN, 0.002 / g.R_IN
    flare_a = g.FLARE / g.R_IN
    r1 = g.R_IN - g.F_T - g.SLOT_PLAY            # the lips' slot face
    r0 = r1 - g.LIP_T
    gz0, gz1 = g.GUIDE_Z
    zl = gz1 - g.LEAD
    for s, name in ((1, "l"), (-1, "r")):
        a_lo, a_hi = sorted((s * (phi - over), s * (phi + side + wall)))
        lip = []
        for a in (a_lo, a_hi):
            lip += [polar(r0, a, gz0), polar(r1, a, gz0), polar(r0, a, gz1), polar(r1, a, zl),
                    polar(r1 - g.FLARE, a, gz1)]
        hull(st, f"{b}/lip_{name}", lip, inside, pp)
        a_face, a_out = s * (phi + side), s * (phi + side + wall)
        sw = []
        for rr in (r0, g.R_IN + 0.0005):
            sw += [polar(rr, a_face, gz0), polar(rr, a_out, gz0), polar(rr, a_face, zl),
                   polar(rr, a_face + s * flare_a * 0.999, gz1), polar(rr, a_out + s * 0.0001, gz1),
                   polar(rr, a_out, zl)]
        hull(st, f"{b}/side_{name}", sw, inside, pp)
    edge = phi + side + wall
    for j in range(4):
        a0, a1 = -edge + 2 * edge * j / 4, -edge + 2 * edge * (j + 1) / 4
        hull(st, f"{b}/ledge{j}", [polar(rr, a, z) for rr in (r0, g.R_IN + 0.0005) for a in (a0, a1)
                                   for z in (gz0 - 0.004, gz0)], inside, pp)

    # top, with the lid opening: quads between the opening circle and the wall, around the opening's centre
    cx, cy = g.OPEN_C
    m = 64
    for k in range(m):
        a0, a1 = 2 * math.pi * k / m, 2 * math.pi * (k + 1) / m
        pts, ok = [], True
        for a in (a0 - 0.01, a1 + 0.01):
            ux, uy = math.cos(a), math.sin(a)
            bq = cx * ux + cy * uy
            t = -bq + math.sqrt(bq * bq - (cx * cx + cy * cy - g.R_OUT ** 2))
            if t < g.OPEN_R + 0.002:
                ok = False
            for rr in (g.OPEN_R, t):
                for z in (g.H - g.TOP_T, g.H):
                    pts.append((cx + rr * ux, cy + rr * uy, z))
        if ok:
            hull(st, f"{b}/top{k:02d}", pts, steel, pp)

    # the handle
    hx0, hx1 = g.HANDLE_X
    hy = g.HANDLE_HALF_Y
    hz0, hz1 = g.HANDLE_Z
    hull(st, f"{b}/handle_grip", box(hx0, hx1, -hy, hy, hz0, hz1), black, pp)
    hull(st, f"{b}/handle_top", box(hx0, -g.R_OUT + 0.004, -hy, hy, hz1 - 0.018, hz1), black, pp)
    hull(st, f"{b}/handle_bot", box(hx0, -g.R_OUT + 0.004, -hy, hy, hz0, hz0 + 0.018), black, pp)
    st.GetRootLayer().Save()


def build_lid():
    st = stage_for("lid")
    r = "/lid"
    alu = material(st, f"{r}/Looks/alu", (0.82, 0.83, 0.85), 0.35, 0.9)
    black = material(st, f"{r}/Looks/black", BLACK, 0.5)
    pp = phys_material(st, f"{r}/Looks/pp", 0.5, 0.4)
    b = f"{r}/body"
    rigid(st, b, g.LID_MASS)
    # the disc and the plug under it, behind the dome (lid x <= DOME_X); the dome over the filter's grip tab
    def chord(rr, z0, z1, front):
        pts = [polar(rr, 2 * math.pi * k / 64, 0.0)[:2] for k in range(64)]
        pts = [(x, y) for x, y in pts if (x >= g.DOME_X) == front]
        yc = math.sqrt(rr * rr - g.DOME_X ** 2)
        pts += [(g.DOME_X, yc), (g.DOME_X, -yc)]
        return [(x, y, z) for x, y in pts for z in (z0, z1)]

    hull(st, f"{b}/disc", chord(g.LID_R, 0.0, g.LID_T, False), alu, pp)
    hull(st, f"{b}/plug", chord(g.PLUG_R, -g.PLUG_D, 0.0, False), alu, pp)
    hull(st, f"{b}/dome", chord(g.LID_R, g.DOME_UP, g.DOME_UP + g.LID_T, True), alu, pp)
    yc = math.sqrt(g.LID_R ** 2 - g.DOME_X ** 2)
    hull(st, f"{b}/step", box(g.DOME_X - 0.003, g.DOME_X, -yc, yc, 0.0, g.DOME_UP + g.LID_T), alu, pp)
    knob = [polar(g.KNOB_R, 2 * math.pi * k / 16, 0.0)[:2] for k in range(16)]
    hull(st, f"{b}/knob", [(x, y, z) for x, y in knob for z in (g.LID_T, g.LID_T + g.KNOB_H)], black, pp)
    st.GetRootLayer().Save()


def build_base():
    st = stage_for("base")
    r = "/base"
    black = material(st, f"{r}/Looks/black", BLACK, 0.5)
    pp = phys_material(st, f"{r}/Looks/pp", 0.6, 0.5)
    b = f"{r}/body"
    rigid(st, b, g.BASE_MASS)
    rim = [polar(g.BASE_R, 2 * math.pi * k / 48, 0.0)[:2] for k in range(48)]
    hull(st, f"{b}/puck", [(x, y, z) for x, y in rim for z in (0.0, g.BASE_H)], black, pp)
    st.GetRootLayer().Save()


def build_filter():
    st = stage_for("filter")
    r = "/filter"
    look = material(st, f"{r}/Looks/blue", FILTER, 0.5)
    pp = phys_material(st, f"{r}/Looks/pp", *g.F_FRICTION)
    b = f"{r}/body"
    xf = rigid(st, b, g.F_MASS)
    # a 1.5 mm plate falls about 3 mm per physics step from 25 mm up: speculative contacts keep it from sinking into
    # the kettle's thin walls (and being thrown out of them)
    p = xf.GetPrim()
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    p.CreateAttribute("physxRigidBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool).Set(True)
    phi = g.half_angle()
    base = g.R_IN - g.F_T                       # the inner face's radius: filter x = kettle r cos(a) - base

    def pt(rr, a, z):
        return (rr * math.cos(a) - base, rr * math.sin(a), z)

    n = g.F_SEGMENTS
    for j in range(n):
        a0, a1 = -phi + 2 * phi * j / n, -phi + 2 * phi * (j + 1) / n
        pts = [pt(rr, a, z) for rr in (base, g.R_IN) for a in (a0, a1) for z in (0.0, g.F_H)]
        if j in (0, n - 1):                      # bottom outer corner chamfered 1 mm (moulded parts are rounded)
            a_edge, a_in = (a0, a0 + 0.0010 / g.R_IN) if j == 0 else (a1, a1 - 0.0010 / g.R_IN)
            keep = [pt(rr, a, z) for rr in (base, g.R_IN) for a in (a0, a1) for z in (0.0, g.F_H)
                    if not (z == 0.0 and a == a_edge)]
            pts = keep + [pt(rr, a_in, 0.0) for rr in (base, g.R_IN)] + [pt(rr, a_edge, 0.0010) for rr in (base, g.R_IN)]
        hull(st, f"{b}/strip{j}", pts, look, pp, offset=0.0010)
    hull(st, f"{b}/grip_tab", box(-g.FIN_L, 0.0, -g.FIN_T / 2, g.FIN_T / 2, *g.FIN_Z), look, pp, offset=0.0010)
    st.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_kettle()
    build_lid()
    build_base()
    build_filter()
    for p in sorted(OUT.glob("*.usda")):
        print(p.name, p.stat().st_size, "bytes")
