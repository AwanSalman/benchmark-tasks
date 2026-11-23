"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/holder.usda: the coin holder, one free articulation. Root link ``body``: a solid base, the outer shell, and
  four tubes. Each tube is a ring of wall segments; on the side facing away from the holder's axis the wall stops one
  coin below the top plate, where an exit channel (a floor and two side guides) runs out to the rim. The top plate
  is a lip over each tube with a round opening that runs on to the rim as a slot and, ahead of the tube's centre,
  widens to the channel. Per tube ``<coin>``: a follower disc on prismatic joint ``f_<coin>`` (along z), sprung up
  under the stack, which holds the top coin against the lip.
- assets/<coin>.usda: one rigid body per coin type, a 32-sided prism with the coin's diameter (equal area) and
  thickness.
- assets/tray.usda: the shallow round tray the holder stands in.

The holder's and the tray's origins are on their undersides under their axes; a coin's origin is its centre.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

BLACK = (0.045, 0.045, 0.05)
INNER = (0.10, 0.10, 0.11)
FOLLOWER = (0.85, 0.35, 0.08)   # orange: an empty tube shows its follower
TRAY = (0.80, 0.80, 0.78)


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


def physics_material(stage, path, static, dynamic):
    mat = UsdShade.Material.Define(stage, path)
    p = mat.GetPrim()
    m = UsdPhysics.MaterialAPI.Apply(p)
    m.CreateStaticFrictionAttr(static)
    m.CreateDynamicFrictionAttr(dynamic)
    m.CreateRestitutionAttr(0.0)
    return mat


def bind(prim, mat, phys=None):
    api = UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    api.Bind(mat)
    if phys is not None:
        api.Bind(phys, materialPurpose="physics")


def collide(prim, offset=0.0004):
    p = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def body(stage, path, mass, pos=(0, 0, 0), rot_z_deg=0.0, root=False):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_z_deg:
        xf.AddRotateZOp().Set(rot_z_deg)
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    if root:
        UsdPhysics.ArticulationRootAPI.Apply(p)
        p.AddAppliedSchema("PhysxArticulationAPI")
        p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
        p.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        p.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    return xf


def hull(stage, path, points, mat, phys=None, coll=True):
    """A convex hull collider (and its look) through ``points``."""
    from scipy.spatial import ConvexHull

    pts = np.asarray(points, float)
    h = ConvexHull(pts)
    faces = []
    centre = pts.mean(0)
    for simplex, eq in zip(h.simplices, h.equations):
        a, b, c = pts[simplex]
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
            simplex = simplex[[0, 2, 1]]
        faces.append(list(simplex))
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([3] * len(faces))
    m.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    del centre
    bind(m, mat, phys)
    if coll:
        collide(m)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def visual_mesh(stage, path, verts, tris, mat):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in verts])
    m.CreateFaceVertexCountsAttr([3] * len(tris))
    m.CreateFaceVertexIndicesAttr([int(i) for t in tris for i in t])
    m.CreateSubdivisionSchemeAttr("none")
    bind(m, mat)
    return m


def joint(stage, path, body0, body1, axis, pos0, rot0_z_deg, limits):
    j = UsdPhysics.PrismaticJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    hz = math.radians(rot0_z_deg) / 2
    j.CreateLocalRot0Attr(Gf.Quatf(math.cos(hz), 0, 0, math.sin(hz)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")      # gains come from task.py's actuators
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    return j


def max_combine(mat):
    p = mat.GetPrim()
    p.AddAppliedSchema("PhysxMaterialAPI")
    p.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("max")
    return mat


def build_holder():
    from shapely.geometry import Point, Polygon
    from shapely.ops import triangulate

    stage = stage_for("holder")
    r = "/holder"
    black = material(stage, f"{r}/Looks/black", BLACK, 0.55)
    inner = material(stage, f"{r}/Looks/inner", INNER, 0.6)
    fol = material(stage, f"{r}/Looks/follower", FOLLOWER, 0.6)
    rubber = physics_material(stage, f"{r}/Looks/rubber", *g.HOLDER_FRICTION)
    slick = physics_material(stage, f"{r}/Looks/slick", *g.SLICK)

    H, R, PU = g.HOLDER_H, g.HOLDER_R, g.plate_under()
    z_bot = PU - g.TUBE_DEPTH
    xf = body(stage, f"{r}/body", g.HOLDER_MASS, root=True)
    UsdPhysics.MassAPI(xf.GetPrim()).CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.045))   # holder with its coins
    n = 48
    ring = [(R * math.cos(2 * math.pi * k / n), R * math.sin(2 * math.pi * k / n)) for k in range(n)]
    hull(stage, f"{r}/body/base", [(x, y, z) for x, y in ring for z in (0.0, z_bot)], black, rubber)

    # outer shell (it collides: coins tipping out slide down it), open over each channel's band at the rim
    opens = []
    for c in g.TUBES:
        d = g.tube_dims(c)
        ux, uy = g.exit_dir(c)
        opens.append((math.atan2(uy, ux), math.asin(min(1.0, d["channel_w"] / 2 / R)) + 0.05,
                      PU - d["channel_h"] - g.FLOOR_T))
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        top = PU
        for ang, half, low in opens:
            if abs(math.remainder((a0 + a1) / 2 - ang, 2 * math.pi)) < half:
                top = low
        pts = [(rr * math.cos(a), rr * math.sin(a), z) for rr in (R - 0.002, R) for a in (a0, a1) for z in (z_bot, top)]
        hull(stage, f"{r}/body/shell{k:02d}", pts, black, slick)

    region = Point(0, 0).buffer(R, 16)
    for c, (tx, ty) in g.TUBES.items():
        d = g.tube_dims(c)
        ux, uy = g.exit_dir(c)
        vx, vy = -uy, ux
        rc = math.hypot(tx, ty)
        floor_top = PU - d["channel_h"]
        br = d["bore_r"]
        tube = f"{r}/body/{c}"

        # wall: on the exit side (facing u) only up to the channel floor
        S = 24
        for k in range(S):
            a0, a1 = 2 * math.pi * k / S, 2 * math.pi * (k + 1) / S
            am = (a0 + a1) / 2
            a0e, a1e = am - 0.55 * (a1 - a0), am + 0.55 * (a1 - a0)
            facing = math.cos(am) * ux + math.sin(am) * uy
            top = floor_top if facing > -0.05 else PU
            pts = []
            for a in (a0e, a1e):
                cs, sn = math.cos(a), math.sin(a)
                for rr in (br, br + g.WALL):
                    pts += [(tx + rr * cs, ty + rr * sn, z_bot), (tx + rr * cs, ty + rr * sn, top)]
            hull(stage, f"{tube}_wall{k:02d}", pts, inner, slick)

        # channel floor: strips across the channel, from the bore's edge out to the rim
        W = d["channel_w"]
        NS = 10
        for i in range(NS):
            w0 = -W / 2 + W * i / NS
            w1 = w0 + W / NS
            wn = min(abs(w0), abs(w1)) if w0 * w1 > 0 else 0.0
            wf = max(abs(w0), abs(w1))
            s_in = math.sqrt(max(br ** 2 - wn ** 2, 0.0))
            s_out = math.sqrt(R ** 2 - wf ** 2) - rc
            if s_out <= s_in:
                continue
            pts = [(tx + s * ux + w * vx, ty + s * uy + w * vy, z) for s in (s_in, s_out) for w in (w0, w1)
                   for z in (floor_top - g.FLOOR_T, floor_top)]
            hull(stage, f"{tube}_floor{i:02d}", pts, inner, slick)

        # channel side guides, floor to the top face, so a coin goes straight out
        for sign in (-1.0, 1.0):
            w0, w1 = sign * W / 2, sign * (W / 2 + g.GUIDE_T)
            s_in = math.sqrt(max(br ** 2 - (W / 2) ** 2, 0.0))
            s_out = math.sqrt(R ** 2 - (W / 2 + g.GUIDE_T) ** 2) - rc
            pts = [(tx + s_ * ux + w * vx, ty + s_ * uy + w * vy, z) for s_ in (s_in, s_out) for w in (w0, w1)
                   for z in (floor_top, H)]
            hull(stage, f"{tube}_guide{'l' if sign > 0 else 'r'}", pts, inner, slick)

        # top plate cut-outs: the lip opening, the slot on to the rim and the open mouth ahead of the tube's centre
        def strip(s0, s1, half):
            return Polygon([(tx + s_ * ux + w * vx, ty + s_ * uy + w * vy) for s_, w in
                            ((s0, -half), (s1, -half), (s1, half), (s0, half))])

        region = (region.difference(Point(tx, ty).buffer(d["hole_r"], 8))
                  .difference(strip(0.0, 0.08, d["slot_w"] / 2)).difference(strip(d["mouth_s"], 0.08, W / 2)))

        # follower: authored under the full stack; its top stop is 0.3 mm under the plate
        fz = PU - 0.0003 + g.f_low(c)
        fpath = f"{r}/{c}_follower"
        body(stage, fpath, g.FOLLOWER_MASS, pos=(tx, ty, fz))
        fr = br - 0.0003
        disc = [(fr * math.cos(2 * math.pi * k / 24), fr * math.sin(2 * math.pi * k / 24)) for k in range(24)]
        hull(stage, f"{fpath}/disc", [(x, y, z) for x, y in disc for z in (-g.FOLLOWER_T, 0.0)], fol, slick)
        joint(stage, f"{r}/joints/f_{c}", f"{r}/body", fpath, "Z", (tx, ty, fz), 0.0,
              (0.0, -g.f_low(c)))        # from under the full stack (its bottom stop) up to the lip

    # top plate: what is left, triangulated into thin prisms
    k = 0
    for tri in triangulate(region):
        if not region.contains(tri.representative_point()) or tri.area < 1e-8:
            continue
        xy = list(tri.exterior.coords)[:3]
        hull(stage, f"{r}/body/plate{k:03d}", [(x, y, z) for x, y in xy for z in (PU, H)], black, slick)
        k += 1
    stage.GetRootLayer().Save()
    return k


def build_coin(name):
    dia, t, mass, color, _ = g.COINS[name]
    stage = stage_for(name)
    r = f"/{name}"
    look = material(stage, f"{r}/Looks/metal", color, 0.35, 0.8)
    phys = max_combine(physics_material(stage, f"{r}/Looks/phys", *g.COIN_FRICTION))
    body(stage, f"{r}/coin", mass)
    n = g.COIN_SIDES
    rc = dia / 2 * math.sqrt(2 * math.pi / (n * math.sin(2 * math.pi / n)))   # same area as the disc
    pts = [(rc * math.cos(2 * math.pi * k / n), rc * math.sin(2 * math.pi * k / n), z) for k in range(n)
           for z in (-t / 2, t / 2)]
    m = hull(stage, f"{r}/coin/disc", pts, look, phys)
    p = m.GetPrim()
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.0005)
    p.CreateAttribute("physxCollision:torsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.006)
    p.CreateAttribute("physxCollision:minTorsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.002)
    stage.GetRootLayer().Save()


def build_tray():
    stage = stage_for("tray")
    r = "/tray"
    look = material(stage, f"{r}/Looks/tray", TRAY, 0.4)
    phys = physics_material(stage, f"{r}/Looks/phys", *g.TRAY_FRICTION)
    body(stage, f"{r}/body", g.TRAY_MASS)
    n = 48
    R = g.TRAY_R
    hull(stage, f"{r}/body/floor", [(R * math.cos(2 * math.pi * k / n), R * math.sin(2 * math.pi * k / n), z)
                                     for k in range(n) for z in (0.0, g.TRAY_FLOOR)], look, phys)
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = [(rr * math.cos(a), rr * math.sin(a), z) for rr in (R - g.TRAY_WALL, R) for a in (a0, a1)
               for z in (0.0, g.TRAY_H)]
        hull(stage, f"{r}/body/wall{k:02d}", pts, look, phys)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.usda"):
        old.unlink()
    print("plate pieces", build_holder())
    for c in g.COINS:
        build_coin(c)
    build_tray()
    for p in sorted(OUT.rglob("*.usda")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
