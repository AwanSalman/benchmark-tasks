"""Change a click pen's refill. The pen is a one-piece-barrel click ballpoint taken apart: its clear barrel stands
nose-down in a desk pen stand, with the clicker unit and a new refill standing beside it; the pen's spring has fallen
out and stands on end on the table. The spring goes back into the barrel's nose, the new refill point-first into the
barrel through the spring, and the clicker into the back end, its hook toward the clip, where it latches.

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
# success() must hold for this long before the trial ends with success. yam_arena's embodiment enforces it: it counts
# the consecutive control steps on which success() is true and ends the trial only once they reach SUCCESS_HOLD_S
# (yam_arena/inspect_embodiment.py, the task-file scorer). score() itself reports the state at each moment.
SUCCESS_HOLD_S = 1.0
INSTRUCTION = ("Put the new refill in the pen: the spring into the barrel's nose, the refill in point-first through "
               "it, then push the clicker in until it latches, its hook toward the clip.")

TOP = g.TOP
_S = math.sqrt(0.5)
UP = (0.0, -_S, 0.0, _S)      # the part's own x axis pointing up (its y axis stays the world's y)


def _f(v):
    return tuple(float(x) for x in v)


def _socket(name):
    return (g.HOLDER_AT[0], g.HOLDER_AT[1] + g.SOCKET_Y[name])


FLOOR = TOP + g.SLAB_T + 0.0005
HOLDER = UsdBody("holder", usd="assets/holder.usda", pos=_f((*g.HOLDER_AT, TOP)), rot_xyzw=UP,
                 size=(g.SLAB_T + 0.03, g.BASE_Y, g.BASE_X), mass=None, friction=(0.5, 0.4))
# the barrel's frame: origin at the nose tip, x toward its open back end; the clip on its +y side (the robot's left)
# The barrel is held fixed by the stand (mass=None: kinematic), as a snug hole or putty holds it in a real trial.
BARREL = UsdBody("barrel", usd="assets/barrel.usda", pos=_f((*_socket("barrel"), FLOOR)), rot_xyzw=UP,
                 size=(g.BARREL_L, 0.01, 0.01), mass=None, friction=(0.08, 0.05), friction_min=True, solver_iters=32,
                 torsional_patch=0.004, no_sleep=True)
# the refill's frame: origin at its point, x toward its back end (up while it stands point-down)
REFILL = UsdBody("refill", usd="assets/refill.usda", pos=_f((*_socket("refill"), FLOOR)), rot_xyzw=UP,
                 size=(g.REFILL_L, 0.003, 0.003), mass=g.REFILL_MASS, friction=(0.6, 0.5), solver_iters=32,
                 torsional_patch=0.004, no_sleep=True, max_speed=0.4)
# the clicker's frame: origin at its lower end, x toward the button; its hook on its +y side
# Its turning is damped (angular_damping) in place of the real hook's latch, which holds it once it is in.
CLICKER = UsdBody("clicker", usd="assets/clicker.usda", pos=_f((*_socket("clicker"), FLOOR)), rot_xyzw=UP,
                  size=(g.CLICKER_L, 0.0066, 0.0066), mass=g.CLICKER_MASS, friction=(0.4, 0.3), solver_iters=32,
                  torsional_patch=0.003, no_sleep=True, angular_damping=20.0)
SPRING = UsdBody("spring", usd="assets/spring.usda", pos=_f((*g.SPRING_XY, TOP + g.SPRING_L / 2 + 0.0005)),
                 rot_xyzw=UP, size=(g.SPRING_L, 0.0042, 0.0042), mass=g.SPRING_MASS, friction=(0.6, 0.5), solver_iters=32,
                 torsional_patch=0.003, no_sleep=True, max_speed=0.15)
OBJECTS = [HOLDER, BARREL, REFILL, CLICKER, SPRING]
PARTS = ("barrel", "refill", "clicker", "spring")

_STATE: dict = {}


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy()


def _rot(xyzw):
    import numpy as np

    x, y, z, w = xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose(environment, name):
    base = environment.unwrapped
    p = _np(base.scene[name].data.root_pose_w)[0]
    return p[:3] - _np(base.scene.env_origins[0]), _rot(p[3:7])


def speed(environment, name) -> float:
    import numpy as np

    base = environment.unwrapped
    return float(np.linalg.norm(_np(base.scene[name].data.root_lin_vel_w)[0]))


def tool_points(environment):
    """Both arms' tool points (between the finger pads, 13 mm short of their tips), station frame."""
    import numpy as np

    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        p = _np(robot.data.body_link_pose_w)[0, list(robot.body_names).index("gripper")]
        out.append(p[:3] - _np(base.scene.env_origins[0]) + _rot(p[3:7]) @ np.asarray(TCP_OFFSET))
    return out


def _deg(u, v):
    import numpy as np

    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))))))


@on_reset
def _start(environment=None) -> None:
    _STATE.clear()


@on_step
def _monitor(environment) -> None:
    import numpy as np

    bp, bR = pose(environment, "barrel")
    rp, rR = pose(environment, "refill")
    cp, cR = pose(environment, "clicker")
    sp, _ = pose(environment, "spring")
    sb = bR.T @ (sp - bp)                      # the spring's centre in the barrel's frame (x from the nose tip)
    sr = rR.T @ (sp - rp)                      # ... and in the refill's frame (x from the point)
    pb = bR.T @ (rp - bp)                      # the refill's point in the barrel's frame
    lo = bR.T @ (cp - bp)                      # the clicker's lower end in the barrel's frame
    top = bR.T @ (cp + cR[:, 0] * g.CLICKER_L - bp)   # its button's top
    axis = bR[:, 0]
    hook = cR[:, 1] - axis * (cR[:, 1] @ axis)          # the clicker's hook side and the barrel's clip side, square to
    clip = bR[:, 1] - axis * (bR[:, 1] @ axis)          # the barrel's axis
    ends = [bp, bp + bR[:, 0] * g.BARREL_L, rp, rp + rR[:, 0] * g.REFILL_L, cp, cp + cR[:, 0] * g.CLICKER_L, sp]
    clear = min(float(np.linalg.norm(t - q)) for t in tool_points(environment) for q in ends)
    _STATE.update(spring_x=float(sb[0]), spring_off=float(np.hypot(sb[1], sb[2])),
                  spring_along=float(sr[0]), spring_on_off=float(np.hypot(sr[1], sr[2])),
                  point_x=float(pb[0]), point_off=float(np.hypot(pb[1], pb[2])),
                  refill_axis=_deg(rR[:, 0], axis),
                  clicker_out=float(top[0] - g.BARREL_L), clicker_off=float(np.hypot(lo[1], lo[2])),
                  clicker_axis=_deg(cR[:, 0], axis), hook_deg=_deg(hook, clip),
                  clear=clear, still=max(speed(environment, k) for k in PARTS) <= g.STILL)


def spring_on_refill() -> bool:
    """The spring round the refill's front: its centre within 1.2 mm of the refill's axis, between 12 mm ahead of the
    point and 8 mm behind it (against the stop its centre is 7 mm behind the point)."""
    s = _STATE
    return bool(s and s["spring_on_off"] < g.SPRING_ON_OFF and -0.012 < s["spring_along"] < 0.008)


def spring_placed() -> bool:
    """The spring back in place: in the barrel's nose (centre within 3.25 mm of its axis, within 5 cm of the nose tip)
    or round the refill's front."""
    s = _STATE
    return bool(s and ((s["spring_off"] < g.SPRING_OFF and 0.0 < s["spring_x"] < g.SPRING_NOSE)
                       or spring_on_refill()))


def refill_in() -> bool:
    """The refill in the barrel point-first through the spring: its point within 1.5 mm of the barrel's axis, 8 to 30 mm
    from the nose tip, in line within 6 degrees, and the spring round its front."""
    s = _STATE
    return bool(s and s["point_off"] < g.POINT_OFF and g.POINT_RANGE[0] < s["point_x"] < g.POINT_RANGE[1]
                and s["refill_axis"] < g.AXIS_DEG and spring_on_refill())


def clicker_in() -> bool:
    """The clicker latched in the back end: its button 6 to 10 mm out of the barrel, its lower end within 1.5 mm of the
    barrel's axis, in line within 6 degrees and its hook within 20 degrees of the clip's side."""
    s = _STATE
    return bool(s and g.CLICKER_RANGE[0] < s["clicker_out"] < g.CLICKER_RANGE[1] and s["clicker_off"] < g.CLICKER_OFF
                and s["clicker_axis"] < g.AXIS_DEG and s["hook_deg"] < g.HOOK_DEG)


def done() -> bool:
    s = _STATE
    return bool(s and s["clear"] >= g.CLEAR and s["still"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def score(objects) -> float:
    """0 unless the spring is back in place (in the barrel's nose or round the refill's front). Then 0.25, plus 0.25
    with the refill in point-first through the spring, plus 0.25 with the clicker latched, plus 0.25 (1.0, success)
    with both hands 4 cm clear of every part and nothing moving. The trial ends once 1.0 has held for SUCCESS_HOLD_S."""
    if not spring_placed():
        return 0.0
    if not refill_in():
        return 0.25
    if not clicker_in():
        return 0.5
    return 1.0 if done() else 0.75
