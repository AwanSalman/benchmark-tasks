"""On the station table: two in-shell peanuts lying on the bench on the robot's left, a white snack bowl in front of
the robot and a small blue saucer for the shells nearer to it. Each peanut is two shell halves (lobes) that start
joined at the waist, with a kernel loose inside each half. The shell gives once the pod is bent 3 degrees at the
waist (or pulled or twisted 0.8 mm apart there); after that the halves are separate pieces.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Shell the two peanuts: the nuts in the white bowl, the shells on the blue saucer."

TOP = g.TOP
_S = math.sqrt(0.5)
TO_PLUS_Y, TO_MINUS_Y = (0.0, 0.0, _S, _S), (0.0, 0.0, -_S, _S)
AXIS_Z = TOP + g.POD_R + 0.0003
K_Z = AXIS_Z - (g.POD_R - g.SHELL_T - g.KERNEL_R) + 0.0002
PODS = tuple(g.PODS)                                   # "p1", "p2"
HALVES = ("b", "f")                                    # back (toward +y) and front lobe


def _f(v):
    return tuple(float(x) for x in v)


def lobe(pod, h):
    return f"{pod}_{h}"


def kernel(pod, h):
    return f"{pod}_k{h}"


LOBES = tuple(lobe(p, h) for p in PODS for h in HALVES)
KERNELS = tuple(kernel(p, h) for p in PODS for h in HALVES)


def _define_crack(stage, front_path, pod):
    """The crack: a fixed joint between the pod's two lobes at the waist, defined as the front lobe is spawned. The
    monitor switches it off once the pod is bent past the crack threshold, and the reset switches it back on."""
    from pxr import Gf, Sdf, UsdPhysics

    root = front_path.rsplit("/", 1)[0]
    j = UsdPhysics.FixedJoint.Define(stage, f"{root}/{lobe(pod, 'b')}/crack")
    j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{lobe(pod, 'b')}")])
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{lobe(pod, 'f')}")])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(0, 0, 0, 1))      # the front lobe's x is the back lobe's -x: 180 deg about z
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateCollisionEnabledAttr(False)
    j.CreateJointEnabledAttr(True)


LOBE_KW = dict(usd="assets/lobe.usda", size=(0.02, 0.015, 0.015), mass=g.LOBE_MASS, friction=g.SHELL_FRICTION,
               solver_iters=32, torsional_patch=0.003)
KERNEL_KW = dict(usd="assets/kernel.usda", size=(g.KERNEL_L, 2 * g.KERNEL_R, 2 * g.KERNEL_R), mass=g.KERNEL_MASS,
                 friction=g.KERNEL_FRICTION, friction_min=True, solver_iters=32)

POD_OBJECTS = []
for _p, (_x, _y) in g.PODS.items():
    _w = (_x, _y, AXIS_Z)
    POD_OBJECTS += [
        UsdBody(lobe(_p, "b"), pos=_f(_w), rot_xyzw=TO_PLUS_Y, **LOBE_KW),
        UsdBody(lobe(_p, "f"), pos=_f(_w), rot_xyzw=TO_MINUS_Y,
                after_spawn=(lambda pod: lambda st, path: _define_crack(st, path, pod))(_p), **LOBE_KW),
        UsdBody(kernel(_p, "b"), pos=_f((_x, _y + g.KERNEL_AT, K_Z)), rot_xyzw=TO_PLUS_Y, **KERNEL_KW),
        UsdBody(kernel(_p, "f"), pos=_f((_x, _y - g.KERNEL_AT, K_Z)), rot_xyzw=TO_PLUS_Y, **KERNEL_KW),
    ]
BOWL = UsdBody("bowl", usd="assets/bowl.usda", pos=_f((*g.BOWL_XY, TOP + 0.0003)),
               size=(2 * g.BOWL_R, 2 * g.BOWL_R, g.BOWL_H), mass=g.BOWL_MASS, friction=g.CERAMIC_FRICTION)
DISH = UsdBody("dish", usd="assets/dish.usda", pos=_f((*g.DISH_XY, TOP + 0.0003)),
               size=(2 * g.DISH_R, 2 * g.DISH_R, g.DISH_H), mass=g.DISH_MASS, friction=g.CERAMIC_FRICTION)
OBJECTS = [*POD_OBJECTS, BOWL, DISH]

# Finger pads, from each finger link's origin in the gripper link's frame: the pad's tip and a point 3 cm back along
# it (yam_arena's URDF).
PADS = {"tip_left": [(-0.0445, 0.024, 0.0903), (-0.0445, 0.024, 0.0603)],
        "tip_right": [(0.0445, -0.024, 0.0903), (0.0445, -0.024, 0.0603)]}

_STATE: dict = {}


# -- poses ------------------------------------------------------------------------------------------------------------
def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy()


def _rot(xyzw):
    import numpy as np

    x, y, z, w = xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose(environment, name):
    """An object's origin and rotation, station frame. A lobe's origin is on the pod's axis at the waist, its x along
    the axis toward its closed end."""
    base = environment.unwrapped
    p = _np(base.scene[name].data.root_pose_w)[0]
    return p[:3] - _np(base.scene.env_origins[0]), _rot(p[3:7])


def lobe_centre(environment, name):
    p, R = pose(environment, name)
    import numpy as np

    return p + R @ np.array([g.LOBE_L * 0.7, 0.0, 0.0])


def pad_points(environment) -> list:
    import numpy as np

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    points = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        p = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        R = _rot(p[names.index("gripper"), 3:7])
        for link, offsets in PADS.items():
            at = p[names.index(link), :3] - origin
            points += [at + R @ np.asarray(o) for o in offsets]
    return points


# -- the crack joints --------------------------------------------------------------------------------------------------
def _joint_prim(environment, pod):
    from isaaclab.sim.utils.stage import get_current_stage

    root = getattr(environment.unwrapped.scene, "env_prim_paths", ["/World/envs/env_0"])[0]
    return get_current_stage().GetPrimAtPath(f"{root}/{lobe(pod, 'b')}/crack")


def _set_joint(environment, pod, enabled: bool) -> None:
    from pxr import UsdPhysics

    prim = _joint_prim(environment, pod)
    if prim:
        UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(bool(enabled))


def _put_back(environment) -> None:
    """Put every pod body back on its start pose (the reset's settling would otherwise run with the old joints)."""
    import torch

    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    for spec in POD_OBJECTS:
        a = base.scene[spec.name]
        rp = a.data.root_pose_w
        dev = getattr(rp, "torch", rp).device
        pose7 = torch.tensor([[*spec.pos, *spec.rot_xyzw]], dtype=torch.float32, device=dev)
        pose7[0, :3] += torch.as_tensor(getattr(origin, "torch", origin), device=dev)[:3]
        vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
        if hasattr(a, "write_root_pose_to_sim_index"):
            a.write_root_pose_to_sim_index(root_pose=pose7)
            a.write_root_velocity_to_sim_index(root_velocity=vel)
        else:
            a.write_root_pose_to_sim(pose7)
            a.write_root_velocity_to_sim(vel)


@on_reset
def _start(environment=None) -> None:
    _STATE.clear()
    _STATE.update(cracked={p: False for p in PODS}, history=[], bowl_moved=0.0, bowl_tilt=0.0)
    if environment is not None:
        for p in PODS:
            _set_joint(environment, p, True)
        _put_back(environment)


# -- monitor ----------------------------------------------------------------------------------------------------------
def _in_a_shell(environment, kp) -> bool:
    """A kernel still inside a shell half: within the cavity of any lobe."""
    for name in LOBES:
        p, R = pose(environment, name)
        q = R.T @ (kp - p)
        if -0.002 < q[0] < g.LOBE_L + g.POD_R and math.hypot(q[1], q[2]) < g.POD_R:
            return True
    return False


def _in_vessel(pt, vp, R_in, depth) -> bool:
    return math.hypot(pt[0] - vp[0], pt[1] - vp[1]) < R_in and -0.002 < pt[2] - vp[2] < depth


@on_step
def _monitor(environment) -> None:
    """After every control step: crack any pod bent past the threshold; where each kernel and shell half is; how far
    the hands are from them; whether everything is still; whether the bowl was knocked."""
    import numpy as np

    S = _STATE
    if "cracked" not in S:
        _start(None)
    for p in PODS:
        if S["cracked"][p]:
            continue
        bp, bR = pose(environment, lobe(p, "b"))
        fp, fR = pose(environment, lobe(p, "f"))
        bend = math.degrees(math.acos(max(-1.0, min(1.0, float(-bR[:, 0] @ fR[:, 0])))))
        if bend > g.CRACK_BEND_DEG or float(np.linalg.norm(fp - bp)) > g.CRACK_GAP:
            S["cracked"][p] = True
            _set_joint(environment, p, False)
    bowl_p, bowl_R = pose(environment, "bowl")
    dish_p, _ = pose(environment, "dish")
    S["bowl_moved"] = max(S["bowl_moved"], math.hypot(bowl_p[0] - g.BOWL_XY[0], bowl_p[1] - g.BOWL_XY[1]))
    S["bowl_tilt"] = max(S["bowl_tilt"], math.degrees(math.acos(min(1.0, float(bowl_R[2, 2])))))
    kernels, shells, points = {}, {}, []
    for k in KERNELS:
        kp, _ = pose(environment, k)
        points.append(kp)
        kernels[k] = bool(_in_vessel(kp, bowl_p, g.BOWL_R - g.BOWL_WALL, g.BOWL_H)
                          and not _in_a_shell(environment, kp))
    for name in LOBES:
        c = lobe_centre(environment, name)
        points.append(c)
        pod = name.split("_")[0]
        shells[name] = ("dish" if S["cracked"][pod] and _in_vessel(c, dish_p, g.DISH_R - g.DISH_WALL, g.DISH_H + 0.01)
                        else "bowl" if _in_vessel(c, bowl_p, g.BOWL_R - g.BOWL_WALL, g.BOWL_H) else None)
    pads = pad_points(environment)
    clear = min(float(np.linalg.norm(pt - q)) for pt in pads for q in points) - 0.012   # less a piece's half-size
    hist = S["history"]
    hist.append(np.concatenate(points))
    del hist[:-20]
    drift = max(float(np.max(np.linalg.norm((h - hist[-1]).reshape(-1, 3), axis=1))) for h in hist) \
        if len(hist) == 20 else 1.0
    S.update(kernels=kernels, shells=shells, clear=clear, drift=drift)


# -- scoring ----------------------------------------------------------------------------------------------------------
def _partial() -> float:
    S = _STATE
    if "kernels" not in S:
        return 0.0
    s = 0.15 * sum(S["kernels"].values()) + 0.075 * sum(v == "dish" for v in S["shells"].values())
    if any(v == "bowl" for v in S["shells"].values()) or S["bowl_moved"] > g.BOWL_MOVED or S["bowl_tilt"] > 20.0:
        s = min(s, 0.5)
    return min(s, 0.9)


def success(objects) -> bool:
    """All four kernels in the bowl and out of their shells, all four shell halves on the saucer, no shell piece in
    the bowl, the bowl not knocked, every finger pad 3 cm clear of every nut and shell piece, nothing moving."""
    S = _STATE
    if "kernels" not in S:
        return False
    return bool(all(S["kernels"].values()) and all(v == "dish" for v in S["shells"].values())
                and S["bowl_moved"] <= g.BOWL_MOVED and S["bowl_tilt"] <= 20.0
                and S["clear"] >= g.CLEAR and S["drift"] < g.STILL)


def score(objects) -> float:
    """Success scores 1.0. Otherwise: 0.15 for each kernel in the bowl and out of its shell, plus 0.075 for each
    shell half (of a cracked pod) on the saucer; at most 0.5 if any shell piece is in the bowl or the bowl was moved
    3 cm or tipped 20 deg; at most 0.9."""
    return 1.0 if success(objects) else _partial()
