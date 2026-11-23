"""Write assets/*.usda from geometry.py (needs pxr): lobe.usda (one half of the pod's shell, open at the waist) and
kernel.usda."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
FACES = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]


def stage_for(name):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    return stage, root


def material(stage, path, color, roughness=0.8):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def mesh(stage, path, pts, faces, mat, collide=True, visible=True):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim())
    UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)
    p = m.GetPrim()
    if not visible:
        m.MakeInvisible()
    if not collide:
        return
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.0003)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")


def ring(x0, x1, ri0, ri1, ro0, ro1, path, stage, mat, n, **kw):
    """A hollow piece of a solid of revolution about x, as n convex wedges."""
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n

        def quad(ri, ro):
            return [(ri * math.cos(t0), ri * math.sin(t0)), (ro * math.cos(t0), ro * math.sin(t0)),
                    (ro * math.cos(t1), ro * math.sin(t1)), (ri * math.cos(t1), ri * math.sin(t1))]

        pts = [(x0, y, z) for y, z in quad(ri0, ro0)] + [(x1, y, z) for y, z in quad(ri1, ro1)]
        mesh(stage, f"{path}_{k}", pts, FACES, mat, **kw)


def build_lobe():
    stage, root = stage_for("lobe")
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(g.LOBE_MASS)
    tan = material(stage, "/lobe/Looks/shell", (0.55, 0.38, 0.20), 0.95)
    R, T, L, n = g.POD_R, g.SHELL_T, g.LOBE_L, g.WALL_SEGMENTS
    W, WL = g.WAIST_R, g.WAIST_L
    # the wall collides as a straight tube; the pod's pinched waist is drawn over it (visual only)
    ring(0.0, L, R - T, R - T, R, R, "/lobe/wall", stage, tan, n, visible=False)
    ring(0.0, WL, W - T, R - T, W, R, "/lobe/look_waist", stage, tan, n, collide=False)
    ring(WL, L, R - T, R - T, R, R, "/lobe/look_wall", stage, tan, n, collide=False)
    # the round end: shells of a sphere about (L, 0, 0), sliced along x; the last slice solid
    m = g.END_SEGMENTS
    for j in range(m):
        a0, a1 = (math.pi / 2) * j / m, (math.pi / 2) * (j + 1) / m
        x0, x1 = L + R * math.sin(a0), L + R * math.sin(a1)
        ro0, ro1 = R * math.cos(a0), R * math.cos(a1)
        ri = lambda a: max(0.0, (R - T) ** 2 - (R * math.sin(a)) ** 2) ** 0.5   # noqa: E731
        ri0, ri1 = ri(a0), ri(a1)
        if ri0 < 0.0005 and ri1 < 0.0005:
            pts = [(x, r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n))
                   for x, r in ((x0, ro0), (x1, max(ro1, 0.0005))) for k in range(n)]
            faces = [list(range(n))[::-1], list(range(n, 2 * n))] + \
                    [[k, (k + 1) % n, n + (k + 1) % n, n + k] for k in range(n)]
            mesh(stage, f"/lobe/end{j}", pts, faces, tan)
        else:
            ring(x0, x1, ri0, max(ri1, 0.0), ro0, max(ro1, 0.0008), f"/lobe/end{j}", stage, tan, n)
    stage.GetRootLayer().Save()


def build_kernel():
    stage, root = stage_for("kernel")
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(g.KERNEL_MASS)
    skin = material(stage, "/kernel/Looks/skin", (0.62, 0.30, 0.22), 0.6)
    n, L, R = 12, g.KERNEL_L, g.KERNEL_R
    pts = []
    for i in range(7):   # an ellipsoid-ish hull: 7 rings along x
        x = -L / 2 + L * i / 6
        r = R * max(0.25, (1 - (2 * x / L) ** 2)) ** 0.5
        pts += [(x, r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    faces = []
    for i in range(6):
        faces += [[i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k] for k in range(n)]
    faces += [list(range(n))[::-1], list(range(6 * n, 7 * n))]
    mesh(stage, "/kernel/body", pts, faces, skin)
    stage.GetRootLayer().Save()


def build_vessel(name, R, H, wall, base, mass, color):
    """An open round vessel: a floor disc and a straight wall, as convex pieces."""
    stage, root = stage_for(name)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(mass)
    mat = material(stage, f"/{name}/Looks/glaze", color, 0.35)
    n = 24
    pts = [(R * math.cos(2 * math.pi * k / n), R * math.sin(2 * math.pi * k / n), z) for z in (0.0, base)
           for k in range(n)]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))] + [[k, (k + 1) % n, n + (k + 1) % n, n + k] for k in range(n)]
    mesh(stage, f"/{name}/floor", pts, faces, mat)
    for k in range(n):   # the wall: wedges standing on the floor
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        ri, ro = R - wall, R
        quad = [(ri * math.cos(t0), ri * math.sin(t0)), (ro * math.cos(t0), ro * math.sin(t0)),
                (ro * math.cos(t1), ro * math.sin(t1)), (ri * math.cos(t1), ri * math.sin(t1))]
        pts = [(x, y, base) for x, y in quad] + [(x, y, H) for x, y in quad]
        mesh(stage, f"/{name}/wall_{k}", pts, FACES, mat)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_lobe()
    build_kernel()
    build_vessel("bowl", g.BOWL_R, g.BOWL_H, g.BOWL_WALL, g.BOWL_BASE, g.BOWL_MASS, (0.93, 0.92, 0.88))
    build_vessel("dish", g.DISH_R, g.DISH_H, g.DISH_WALL, g.DISH_BASE, g.DISH_MASS, (0.30, 0.45, 0.62))
    print("wrote", sorted(p.name for p in OUT.glob("*.usda")))
