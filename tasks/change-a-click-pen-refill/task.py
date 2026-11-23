"""Change a click pen's refill. A desk pen stand holds the pen's parts: the new refill standing point-up, the front
barrel standing nose-up and the back half standing open end up; the refill's spring stands on end on the table. The
spring goes onto the refill's front, the front barrel over both, and the front barrel into the back half.

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
INSTRUCTION = ("Put the new refill in the pen: the spring onto its tip, the front barrel over them, then push the "
               "front barrel into the back half.")

TOP = g.TOP
_S = math.sqrt(0.5)
UP, DOWN = (0.0, -_S, 0.0, _S), (0.0, _S, 0.0, _S)      # the part's own x axis pointing up / down


def _f(v):
    return tuple(float(x) for x in v)


def _socket(name):
    return (g.HOLDER_AT[0], g.HOLDER_AT[1] + g.SOCKET_Y[name])


FLOOR = TOP + g.SLAB_T + 0.0005
HOLDER = UsdBody("holder", usd="assets/holder.usda", pos=_f((*g.HOLDER_AT, TOP)), rot_xyzw=UP,
                 size=(0.034, 0.11, 0.03), mass=None, friction=(0.5, 0.4))
# the refill's frame: origin at its point, x toward its back end (pointing down while it stands point-up)
REFILL = UsdBody("refill", usd="assets/refill.usda", pos=_f((*_socket("refill"), FLOOR + g.REFILL_L)), rot_xyzw=DOWN,
                 size=(g.REFILL_L, 0.005, 0.005), mass=g.REFILL_MASS, friction=(0.25, 0.2), friction_min=True,
                 solver_iters=32, torsional_patch=0.004)
SPRING = UsdBody("spring", usd="assets/spring.usda", pos=_f((*g.SPRING_XY, TOP + g.SPRING_L / 2 + 0.0005)),
                 rot_xyzw=UP, size=(g.SPRING_L, 0.005, 0.005), mass=g.SPRING_MASS, friction=(0.6, 0.5),
                 solver_iters=32, torsional_patch=0.002)
# the front barrel's frame: origin at its nose tip, x toward its open end (pointing down while it stands nose-up)
BARREL = UsdBody("barrel", usd="assets/barrel.usda", pos=_f((*_socket("barrel"), FLOOR + g.BARREL_L)), rot_xyzw=DOWN,
                 size=(g.BARREL_L, 0.01, 0.01), mass=g.BARREL_MASS, friction=(0.6, 0.5), solver_iters=32,
                 torsional_patch=0.004)
# the back half's frame: origin at its closed (button) end, x toward its open end (pointing up)
BACK = UsdBody("back", usd="assets/back.usda", pos=_f((*_socket("back"), FLOOR)), rot_xyzw=UP,
               size=(g.BACK_L, 0.014, 0.014), mass=g.BACK_MASS, friction=(0.6, 0.5), solver_iters=32,
               torsional_patch=0.004)
OBJECTS = [HOLDER, REFILL, SPRING, BARREL, BACK]
PARTS = ("refill", "spring", "barrel", "back")

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


@on_reset
def _start(environment=None) -> None:
    _STATE.clear()


@on_step
def _monitor(environment) -> None:
    import numpy as np

    rp, rR = pose(environment, "refill")
    sp, _ = pose(environment, "spring")
    bp, bR = pose(environment, "barrel")
    kp, kR = pose(environment, "back")
    s = rR.T @ (sp - rp)                       # the spring's centre in the refill's frame (x from the point back)
    n = rR.T @ (bp - rp)                       # the front barrel's nose tip in the refill's frame
    m = kR.T @ (bp + bR[:, 0] * g.BARREL_L - kp)   # the front barrel's open end in the back half's frame
    e = kR.T @ (rp + rR[:, 0] * g.REFILL_L - kp)   # the refill's back end in the back half's frame
    # every point of the pen's parts the hands must be clear of: each part's two ends
    ends = [rp, rp + rR[:, 0] * g.REFILL_L, sp, bp, bp + bR[:, 0] * g.BARREL_L, kp, kp + kR[:, 0] * g.BACK_L]
    clear = min(float(np.linalg.norm(t - q)) for t in tool_points(environment) for q in ends)
    _STATE.update(spring_along=float(s[0]), spring_off=float(np.hypot(s[1], s[2])),
                  nose_along=float(n[0]), nose_off=float(np.hypot(n[1], n[2])),
                  barrel_axis=math.degrees(math.acos(max(-1.0, min(1.0, float(bR[:, 0] @ rR[:, 0]))))),
                  mouth_along=float(m[0]), mouth_off=float(np.hypot(m[1], m[2])),
                  back_axis=math.degrees(math.acos(max(-1.0, min(1.0, abs(float(kR[:, 0] @ bR[:, 0])))))),
                  end_along=float(e[0]), end_off=float(np.hypot(e[1], e[2])), clear=clear,
                  still=max(speed(environment, k) for k in PARTS) <= g.STILL)


def spring_on() -> bool:
    """The spring is on the refill's front section: its centre within 1.2 mm of the refill's axis, between the point's
    taper and the shoulder."""
    s = _STATE
    return bool(s and s["spring_off"] < g.SPRING_OFF and g.POINT_L < s["spring_along"] < g.SHOULDER)


def barrel_over() -> bool:
    """The front barrel is over the refill: the refill's point within 1.5 mm of the barrel's axis, from 1 mm above to
    8 mm below its nose tip, and the two in line within 6 degrees."""
    s = _STATE
    return bool(s and s["nose_off"] < g.NOSE_OFF and -g.NOSE_BELOW < s["nose_along"] < g.NOSE_ABOVE
                and s["barrel_axis"] < g.AXIS_DEG)


def joined() -> bool:
    """The front barrel is pushed into the back half: its open end within 1.5 mm of the sleeve's floor and of the back
    half's axis, the two in line within 6 degrees, and the refill's back end inside the back half."""
    s = _STATE
    return bool(s and abs(s["mouth_along"] - (g.BACK_L - g.SLEEVE_L)) < g.JOIN_GAP and s["mouth_off"] < g.JOIN_OFF
                and s["back_axis"] < g.AXIS_DEG and 0.0 < s["end_along"] < g.BACK_L and s["end_off"] < g.BACK_RI)


def done() -> bool:
    s = _STATE
    return bool(s and s["clear"] >= g.CLEAR and s["still"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def score(objects) -> float:
    """0 unless the spring is on the refill's front section. Then 0.25, plus 0.25 with the front barrel over the refill
    (its point in the nose), plus 0.25 with the front barrel pushed into the back half, plus 0.25 (1.0, success) with
    both hands 4 cm clear of every part and nothing moving."""
    if not spring_on():
        return 0.0
    if not barrel_over():
        return 0.25
    if not joined():
        return 0.5
    return 1.0 if done() else 0.75
