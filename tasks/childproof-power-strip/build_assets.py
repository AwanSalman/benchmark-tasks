"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/strip.usda: the power strip, a free-standing articulation (it stands on the table and slides if pushed): the
  tower with four NEMA 5-15 outlets on its front face, each slot a real opening into a cavity, and in every slot a
  sprung contact leaf on a prismatic joint ``leaf<outlet><slot>`` that presses a blade against the slot's far wall.
- assets/cap<i>.usda: a child-safety outlet cap: round face, grip ridge on the front, two flat blades on the back.
- assets/tray.usda: the foam tray the caps start in, blades down in its slits (kinematic, it stays put).
- assets/lamp.usda: a desk lamp fixed to the table, its cord (links on soft hinges whose rest shape is the way the
  cord lies) and its two-blade plug, which starts pushed into the strip's bottom outlet.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

WHITE = (0.90, 0.90, 0.88)
IVORY = (0.95, 0.93, 0.85)
DARK = (0.06, 0.06, 0.07)
BRASS = (0.78, 0.60, 0.26)
NICKEL = (0.75, 0.76, 0.78)
CAP_COLOUR = (0.92, 0.92, 0.94)
FOAM = (0.22, 0.24, 0.30)
CORD = (0.12, 0.12, 0.12)
LAMP = (0.20, 0.32, 0.55)
SHADE = (0.95, 0.92, 0.80)
RED = (0.80, 0.10, 0.08)


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


def physics_material(stage, path, friction, combine="average"):
    mat = UsdShade.Material.Define(stage, path)
    p = mat.GetPrim()
    m = UsdPhysics.MaterialAPI.Apply(p)
    m.CreateStaticFrictionAttr(friction)
    m.CreateDynamicFrictionAttr(friction)
    m.CreateRestitutionAttr(0.0)
    p.AddAppliedSchema("PhysxMaterialAPI")
    p.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(combine)
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def bind_physics(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat, materialPurpose="physics")


def collide(prim, offset=0.0005):
    p = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def body(stage, path, mass, pos=(0, 0, 0), quat=None, root=False):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if quat is not None:                                         # (w, x, y, z)
        xf.AddOrientOp().Set(Gf.Quatf(*map(float, quat)))
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
    """An axis-aligned box from corner lo to corner hi (in its parent's frame)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
    bind(c, mat)
    if coll:
        collide(c)
    return c


def hexahedron(stage, path, corners, mat, coll=True):
    """A convex solid from 8 corners: the 4 of one end, then the 4 of the other, both in the same winding."""
    pts = [Gf.Vec3f(*map(float, c)) for c in corners]
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    if coll:
        collide(m)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def end_rect(x, y0, y1, z0, z1):
    return [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]


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


def plate_with_holes(a_range, b_range, holes):
    """Rectangles covering [a0, a1] x [b0, b1] except the holes (each (a0, a1, b0, b1))."""
    cuts = sorted({a_range[0], a_range[1], *[h[0] for h in holes], *[h[1] for h in holes]})
    cuts = [c for c in cuts if a_range[0] <= c <= a_range[1]]
    out = []
    for a0, a1 in zip(cuts, cuts[1:]):
        if a1 - a0 < 1e-9:
            continue
        inside = sorted((h for h in holes if h[0] < a1 - 1e-9 and h[1] > a0 + 1e-9), key=lambda h: h[2])
        b = b_range[0]
        for h in inside:
            if h[2] > b + 1e-9:
                out.append((a0, a1, b, h[2]))
            b = max(b, h[3])
        if b_range[1] > b + 1e-9:
            out.append((a0, a1, b, b_range[1]))
    return out


def revolute(stage, path, body0, body1, axis, pos0, rot0, pos1=(0, 0, 0), limits=(-60.0, 60.0)):
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")          # gains come from task.py's actuators
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(0.0)
    return j


def prismatic(stage, path, body0, body1, axis, pos0, limits, rot0=(1, 0, 0, 0)):
    j = UsdPhysics.PrismaticJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(0.0)
    return j


def quat_wxyz(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# -- the power strip ---------------------------------------------------------------------------------------------------
def slot_centres():
    """(outlet, slot, y, z) of every slot, strip frame."""
    return [(k, s, sgn * g.SLOT_DY, z) for k, z in enumerate(g.OUTLET_Z) for s, sgn in enumerate((-1, 1))]


def build_strip():
    stage = stage_for("strip")
    r = "/strip"
    white = material(stage, f"{r}/Looks/white", WHITE, 0.6)
    ivory = material(stage, f"{r}/Looks/ivory", IVORY, 0.5)
    dark = material(stage, f"{r}/Looks/dark", DARK, 0.8)
    brass = material(stage, f"{r}/Looks/brass", BRASS, 0.3, 0.9)
    red = material(stage, f"{r}/Looks/red", RED, 0.4)
    contact = physics_material(stage, f"{r}/Looks/contact", g.CONTACT_FRICTION, "min")

    body(stage, f"{r}/body", g.STRIP_MASS, root=True)
    F, D, W, H = g.FRONT, g.STRIP_D, g.STRIP_W, g.STRIP_H
    back = F + g.SLOT_DEPTH
    z0 = g.FOOT_H
    box(stage, f"{r}/body/core", (back, -W / 2, z0), (D / 2, W / 2, H), white)
    # the face plate, with the slot openings
    opens = [(y - g.SLOT_W / 2, y + g.SLOT_W / 2, z - g.SLOT_H / 2, z + g.SLOT_H / 2) for _, _, y, z in slot_centres()]
    for i, (y0, y1, z0, z1) in enumerate(plate_with_holes((-W / 2, W / 2), (z0, H), opens)):
        box(stage, f"{r}/body/face{i:02d}", (F, y0, z0), (F + g.FACE_T, y1, z1), white)
    # behind it, each slot widens toward +y into the cavity that holds its contact leaf
    cav = [(y - g.CAV_HALF, y + g.CAV_HALF, z - g.SLOT_H / 2, z + g.SLOT_H / 2) for _, _, y, z in slot_centres()]
    for i, (y0, y1, z0, z1) in enumerate(plate_with_holes((-W / 2, W / 2), (z0, H), cav)):
        box(stage, f"{r}/body/wall{i:02d}", (F + g.FACE_T, y0, z0), (back, y1, z1), white)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics(prim, contact)
    rubber = physics_material(stage, f"{r}/Looks/rubber", g.STRIP_FRICTION)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        foot = box(stage, f"{r}/body/foot{i}", (sx * (D / 2 - 0.004) - 0.006, sy * (W / 2 - 0.004) - 0.006, 0.0),
                   (sx * (D / 2 - 0.004) + 0.006, sy * (W / 2 - 0.004) + 0.006, z0), dark)
        bind_physics(foot, rubber)
    # looks: each outlet's ivory face (with its slot openings), the slots' dark floors, the round ground holes, a rocker
    # switch on the top and the strip's own cord out of the back
    fw, fh = g.OUTLET_FACE
    for k, z in enumerate(g.OUTLET_Z):
        holes = [(y - g.SLOT_W / 2, y + g.SLOT_W / 2, z - g.SLOT_H / 2, z + g.SLOT_H / 2) for kk, _, y, _ in
                 slot_centres() if kk == k]
        for i, (y0, y1, z0, z1) in enumerate(plate_with_holes((-fw / 2, fw / 2), (z - fh / 2, z + fh / 2), holes)):
            box(stage, f"{r}/body/outlet{k}_{i}", (F - 0.0006, y0, z0), (F - 0.0001, y1, z1), ivory, coll=False)
        for s, sgn in enumerate((-1, 1)):
            box(stage, f"{r}/body/floor{k}{s}", (back - 0.001, sgn * g.SLOT_DY - g.CAV_HALF, z - g.SLOT_H / 2),
                (back, sgn * g.SLOT_DY + g.CAV_HALF, z + g.SLOT_H / 2), dark, coll=False)
        cylinder(stage, f"{r}/body/ground{k}", "X", (F - 0.0008, 0.0, z - 0.012), 0.0022, 0.0006, dark, coll=False)
    box(stage, f"{r}/body/switch", (-0.012, -0.010, H), (0.012, 0.010, H + 0.004), red, coll=False)
    cylinder(stage, f"{r}/body/cord", "X", (D / 2 + 0.02, 0.0, 0.012), 0.0035, 0.04, dark, coll=False)

    # the contact leaves: a pair in each slot, LEAF_GAP apart at rest; a blade wedges them apart. Each leaf's frame
    # has its y pointing away from the slot's centre line (the pair's -y leaf is turned 180 deg about x), so every
    # joint ``leaf<outlet><slot><side>`` opens toward +.
    for k, s, y, z in slot_centres():
        for side, sgn in enumerate((1, -1)):
            leaf = f"{r}/leaf{k}{s}{side}"
            x0, x1 = F + g.LEAF_X0, F + g.LEAF_X1
            at = ((x0 + x1) / 2, y + sgn * (g.LEAF_GAP / 2 + g.LEAF_T / 2), z)
            quat = (1, 0, 0, 0) if sgn > 0 else (0, 1, 0, 0)
            body(stage, leaf, g.LEAF_MASS, pos=at, quat=quat)
            h, t, hx = g.SLOT_H / 2 - 0.0005, g.LEAF_T / 2, (x1 - x0) / 2
            ch, rise = g.LEAF_LEAD                               # its lead-in ramp, toward the face
            hexahedron(stage, f"{leaf}/blade", end_rect(-hx + ch, -t, t, -h, h) + end_rect(hx, -t, t, -h, h), brass)
            hexahedron(stage, f"{leaf}/lead", end_rect(-hx, -t + rise, t, -h, h) + end_rect(-hx + ch, -t, t, -h, h),
                       brass)
            for prim in Usd.PrimRange(stage.GetPrimAtPath(leaf)):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics(prim, contact)
            prismatic(stage, f"{r}/joints/leaf{k}{s}{side}", f"{r}/body", leaf, "Y", at, (0.0, g.LEAF_TRAVEL),
                      rot0=quat)
    stage.GetRootLayer().Save()


# -- caps, tray ------------------------------------------------------------------------------------------------------
def build_cap(i: int):
    name = f"cap{i}"
    stage = stage_for(name)
    r = f"/{name}"
    plastic = material(stage, f"{r}/Looks/plastic", CAP_COLOUR, 0.5)
    body(stage, f"{r}/body", g.CAP_MASS)
    b = f"{r}/body"
    cylinder(stage, f"{b}/face", "X", (-g.CAP_T / 2, 0, 0), g.CAP_R, g.CAP_T, plastic)
    box(stage, f"{b}/ridge", (-g.CAP_T - g.RIDGE_H, -g.RIDGE_L / 2, -g.RIDGE_T / 2), (-g.CAP_T, g.RIDGE_L / 2,
                                                                                         g.RIDGE_T / 2), plastic)
    p = stage.GetPrimAtPath(f"{b}/ridge")
    p.CreateAttribute("physxCollision:torsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.006)
    p.CreateAttribute("physxCollision:minTorsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.004)
    t, w, L, tip = g.BLADE_T / 2, g.BLADE_W / 2, g.BLADE_L, g.BLADE_TIP
    for s, sgn in enumerate((-1, 1)):
        y = sgn * g.SLOT_DY
        hexahedron(stage, f"{b}/blade{s}", end_rect(0.0, y - t, y + t, -w, w) + end_rect(L - tip, y - t, y + t, -w, w),
                   plastic)
        hexahedron(stage, f"{b}/tip{s}", end_rect(L - tip, y - t, y + t, -w, w) +
                   end_rect(L, y - 0.0003, y + 0.0003, -w + 0.0012, w - 0.0012), plastic)
    stage.GetRootLayer().Save()


def build_tray():
    stage = stage_for("tray")
    r = "/tray"
    foam = material(stage, f"{r}/Looks/foam", FOAM, 0.9)
    body(stage, f"{r}/body", None)
    sx, sy, sz = g.TRAY_SIZE
    base = sz - g.SLIT_DEPTH
    box(stage, f"{r}/body/base", (-sx / 2, -sy / 2, 0), (sx / 2, sy / 2, base), foam)
    slits = [(-g.SLIT_L / 2, g.SLIT_L / 2, c + sgn * g.SLOT_DY - g.SLIT_W / 2, c + sgn * g.SLOT_DY + g.SLIT_W / 2)
             for c in g.CAP_DY for sgn in (-1, 1)]
    for i, (x0, x1, y0, y1) in enumerate(plate_with_holes((-sx / 2, sx / 2), (-sy / 2, sy / 2), slits)):
        box(stage, f"{r}/body/top{i:02d}", (x0, y0, base), (x1, y1, sz), foam)
    stage.GetRootLayer().Save()


# -- the lamp, its cord and plug --------------------------------------------------------------------------------------
def plug_origin():
    """The plug's frame (its front face's centre, x into the outlet) in the station frame."""
    y = g.STRIP_Y
    return np.array([g.STRIP_X + g.FRONT, y, g.TOP + g.OUTLET_Z[g.PLUG_OUTLET]])


def cord_path():
    """Points (station frame) along the cord from the lamp's base to the back of the plug."""
    r = g.TOP + g.CORD_R + 0.0005
    p = plug_origin()
    back = p - [g.PLUG_D, 0, 0]
    ctrl = np.array([
        (g.LAMP_X - g.LAMP_BASE_R - 0.002, g.LAMP_Y, r),
        (0.56, 0.135, r), (0.47, 0.10, r), (0.37, 0.065, r), (0.30, 0.02, r), (0.275, -0.03, r),
        (back[0] - 0.050, back[1], r + 0.002), (back[0] - 0.030, back[1], back[2] - 0.020),
        (back[0] - 0.012, back[1], back[2] - 0.001), tuple(back)])
    # Catmull-Rom through the control points, then equal-length links
    pts = [ctrl[0]]
    P = np.vstack([ctrl[0], ctrl, ctrl[-1]])
    for i in range(1, len(P) - 2):
        for t in np.linspace(0, 1, 60)[1:]:
            p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
            pts.append(0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                              (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    pts = np.array(pts)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    n = max(2, round(s[-1] / g.CORD_LINK))
    out = [pts[0]]
    for k in range(1, n + 1):
        target = s[-1] * k / n
        i = min(np.searchsorted(s, target), len(s) - 1)
        f = (target - s[i - 1]) / max(seg[i - 1], 1e-12)
        out.append(pts[i - 1] + f * (pts[i] - pts[i - 1]))
    return np.array(out)


def frame_along(d):
    x = d / np.linalg.norm(d)
    y = np.cross([0.0, 0.0, 1.0], x)
    y = y / np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def build_lamp():
    stage = stage_for("lamp")
    r = "/lamp"
    blue = material(stage, f"{r}/Looks/blue", LAMP, 0.4, 0.3)
    shade = material(stage, f"{r}/Looks/shade", SHADE, 0.7)
    cord = material(stage, f"{r}/Looks/cord", CORD, 0.7)
    plastic = material(stage, f"{r}/Looks/plug", (0.15, 0.15, 0.16), 0.6)
    nickel = material(stage, f"{r}/Looks/nickel", NICKEL, 0.3, 0.9)
    origin = np.array([g.LAMP_X, g.LAMP_Y, g.TOP])

    body(stage, f"{r}/base", 1.0, root=True)
    cylinder(stage, f"{r}/base/foot", "Z", (0, 0, g.LAMP_BASE_T / 2), g.LAMP_BASE_R, g.LAMP_BASE_T, blue)
    cylinder(stage, f"{r}/base/stem", "Z", (0, 0, g.LAMP_BASE_T + 0.11), 0.007, 0.22, blue, coll=False)
    cone = UsdGeom.Cone.Define(stage, f"{r}/base/shade")
    cone.CreateRadiusAttr(0.075)
    cone.CreateHeightAttr(0.10)
    cone.CreateAxisAttr("Z")
    cone.AddTranslateOp().Set(Gf.Vec3d(0, 0, g.LAMP_BASE_T + 0.24))
    bind(cone, shade)

    pts = cord_path()
    prev, prev_R, prev_end = f"{r}/base", np.eye(3), None
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        L = float(np.linalg.norm(b - a))
        R = frame_along(b - a)
        link = f"{r}/c{i:02d}"
        body(stage, link, g.CORD_LINK_MASS, pos=a - origin, quat=quat_wxyz(R))
        cap = UsdGeom.Capsule.Define(stage, f"{link}/tube")
        cap.CreateRadiusAttr(g.CORD_R)
        cap.CreateHeightAttr(max(0.001, L - 2 * g.CORD_R + 0.002))
        cap.CreateAxisAttr("X")
        cap.AddTranslateOp().Set(Gf.Vec3d(L / 2, 0, 0))
        bind(cap, cord)
        collide(cap)
        pos0 = (a - origin) if i == 0 else prev_end
        axis = "Z" if i % 2 == 0 else "Y"
        revolute(stage, f"{r}/joints/j{i}", prev, link, axis, pos0, quat_wxyz(prev_R.T @ R),
                 limits=(-70.0, 70.0) if axis == "Z" else (-60.0, 60.0))
        prev, prev_R, prev_end = link, R, (L, 0.0, 0.0)

    # the plug: its frame at its front face, x into the outlet
    p = plug_origin()
    plug = f"{r}/plug"
    body(stage, plug, g.PLUG_MASS, pos=p - origin)
    box(stage, f"{plug}/shell", (-g.PLUG_D, -g.PLUG_W / 2, -g.PLUG_H / 2), (0, g.PLUG_W / 2, g.PLUG_H / 2), plastic)
    cylinder(stage, f"{plug}/strain", "X", (-g.PLUG_D - 0.006, 0, 0), 0.0055, 0.012, plastic, coll=False)
    t, w = g.BLADE_T / 2, g.BLADE_W / 2
    for s, sgn in enumerate((-1, 1)):
        y = sgn * g.SLOT_DY
        hexahedron(stage, f"{plug}/blade{s}", end_rect(0.0, y - t, y + t, -w, w) +
                   end_rect(g.PLUG_BLADE_L - 0.002, y - t, y + t, -w, w), nickel)
        hexahedron(stage, f"{plug}/tip{s}", end_rect(g.PLUG_BLADE_L - 0.002, y - t, y + t, -w, w) +
                   end_rect(g.PLUG_BLADE_L, y - 0.0004, y + 0.0004, -w + 0.001, w - 0.001), nickel)
    revolute(stage, f"{r}/joints/j{len(pts) - 1}", prev, plug, "Z", prev_end, quat_wxyz(prev_R.T),
             pos1=(-g.PLUG_D, 0, 0), limits=(-70.0, 70.0))
    stage.GetRootLayer().Save()
    return len(pts)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_strip()
    for i in range(g.CAPS):
        build_cap(i)
    build_tray()
    n = build_lamp()
    print("cord joints:", n)
    for p in sorted(OUT.rglob("*.usda")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
