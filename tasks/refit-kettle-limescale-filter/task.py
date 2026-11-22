"""On the station table: a KitchenAid 1.25 L electric kettle, off its power base, its spout away from the robot and its
lid on, with its limescale filter out; the rinsed filter lies flat on the bench beside the kettle, outer face down, its
grip tab up.

The lid comes off by its knob. The filter is a curved mesh plate: it goes in through the lid opening and slides down
the slot behind the spout, its curve facing outward to match the kettle's wall, until it stands on the slot's stop.
Then the lid goes back on.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 ...
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
INSTRUCTION = "Put the limescale filter back in the kettle."
TOP = g.TOP
S2 = math.sqrt(0.5)


def _f(v):
    return tuple(float(x) for x in v)


KETTLE = UsdBody("kettle", usd="assets/kettle.usda", pos=_f((*g.KETTLE_XY, TOP + 0.0003)),
                 size=_f((2 * g.R_OUT, 2 * g.R_OUT, g.H)), mass=g.KETTLE_MASS, usd_materials=True, solver_iters=32)
# lying outer face down, grip tab up: filter x -> world -z, filter z -> world +x
FILTER = UsdBody("filter", usd="assets/filter.usda", pos=_f((*g.FILTER_XY, TOP + g.F_T + 0.0005)),
                 rot_xyzw=(0.0, S2, 0.0, S2), size=_f((g.F_H, g.F_W, g.F_T)), mass=g.F_MASS, usd_materials=True,
                 solver_iters=32, torsional_patch=0.006)
LID = UsdBody("lid", usd="assets/lid.usda",
              pos=_f((g.KETTLE_XY[0] + g.OPEN_C[0], g.KETTLE_XY[1] + g.OPEN_C[1], TOP + 0.0003 + g.H + 0.0002)),
              size=_f((2 * g.LID_R, 2 * g.LID_R, g.LID_T + g.KNOB_H)), mass=g.LID_MASS, usd_materials=True,
              solver_iters=32)
BASE = UsdBody("power_base", usd="assets/base.usda", pos=_f((*g.BASE_XY, TOP + 0.0003)),
               size=_f((2 * g.BASE_R, 2 * g.BASE_R, g.BASE_H)), mass=g.BASE_MASS, usd_materials=True)
OBJECTS = [KETTLE, FILTER, LID, BASE]

_STATE: dict = {}


def _reset_values():
    _STATE.update(seated=False, in_slot=False, lid_on=False, clear=False, still=False, err=None, tilt=None,
                  hand_dist=None, lid_dist=None, lid_err=None, lid_tilt=None, kettle_tilt=0.0, kettle_max_tilt=0.0, kettle_moved=0.0)


_reset_values()


@on_reset
def _reset(environment=None) -> None:
    _reset_values()


def filter_in_kettle(kp, kR, fp, fR):
    """The filter's origin relative to its seated place (kettle frame) and its angle (deg) from its seated
    orientation (upright, curve outward)."""
    import numpy as np

    rel = kR.T @ (np.asarray(fp) - np.asarray(kp))
    Rrel = kR.T @ np.asarray(fR)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rrel) - 1) / 2))))
    return rel - np.asarray(g.seat_origin_k()), ang


def classify(err, ang):
    """(in its slot, seated on its stop) from the filter's offset from its seated place and its angle."""
    in_xy = abs(err[1]) <= g.SIDE_PLAY + 0.0005 and -g.SLOT_PLAY - 0.0005 <= err[0] <= 0.0005
    in_slot = bool(in_xy and ang <= g.LEAN_DEG and err[2] <= g.IN_SLOT_Z)
    seated = bool(in_slot and ang <= g.UPRIGHT_DEG and abs(err[2]) <= g.SEATED_Z)
    return in_slot, seated


def hand_points(environment):
    """Both arms' tool points (between the finger pads, 13 mm short of their tips), station frame."""
    import numpy as np

    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    origin = getattr(origin, "torch", origin).detach().cpu().numpy()
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = list(robot.body_names).index("gripper")
        pose = robot.data.body_link_pose_w
        pose = getattr(pose, "torch", pose)[0, i].detach().cpu().numpy()
        x, y, z, w = pose[3:7]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        out.append(pose[:3] - origin + R @ np.asarray(TCP_OFFSET))
    return out


def lid_on(kp, kR, lp, lR):
    """(on, centre offset from the opening's centre in the kettle frame, tilt in degrees from the kettle's axis)."""
    import numpy as np

    rel = kR.T @ (np.asarray(lp) - np.asarray(kp)) - np.asarray(g.lid_on_k())
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((kR.T @ np.asarray(lR))[2, 2])))))
    on = bool(math.hypot(rel[0], rel[1]) <= g.LID_ON_XY and abs(rel[2]) <= g.LID_ON_Z and tilt <= g.LID_ON_DEG)
    return on, rel, tilt


@on_step
def _monitor(environment) -> None:
    import numpy as np

    from yam_arena.scene import object_states

    st = object_states(environment, ["kettle", "filter", "lid"])
    k, f, lid = st["kettle"], st["filter"], st["lid"]
    kR, fR, lR = k.rotation(), f.rotation(), lid.rotation()
    err, ang = filter_in_kettle(k.pos, kR, f.pos, fR)
    in_slot, seated = classify(err, ang)
    # distance from each hand to the filter: to its middle and to its grip tab
    pts = [np.asarray(f.pos) + fR @ np.array(p) for p in ((0.0, 0.0, g.F_H / 2), (-g.FIN_L / 2, 0.0, g.FIN_Z[1] - 0.01))]
    # ... and to the lid: its knob and the near point of its rim
    hands = hand_points(environment)
    lp = np.asarray(lid.pos)
    knob = lp + lR @ np.array([0.0, 0.0, g.LID_T + g.KNOB_H / 2])
    dist = min(float(np.linalg.norm(h - p)) for h in hands for p in pts)
    rim = []
    for h in hands:
        d = lR.T @ (h - lp)
        r = math.hypot(d[0], d[1])
        near = d if r <= g.LID_R else np.array([d[0] * g.LID_R / r, d[1] * g.LID_R / r, d[2]])
        near[2] = min(max(near[2], -g.PLUG_D), g.LID_T)
        rim.append(float(np.linalg.norm(d - near)))
    lid_dist = min(min(float(np.linalg.norm(h - knob)) for h in hands), min(rim))
    on, lerr, ltilt = lid_on(k.pos, kR, lid.pos, lR)
    kt = math.degrees(math.acos(min(1.0, float(kR[2, 2]))))
    _STATE.update(in_slot=in_slot, seated=seated, err=err, tilt=ang, hand_dist=dist, lid_dist=lid_dist,
                  lid_on=on, lid_err=lerr, lid_tilt=ltilt, clear=min(dist, lid_dist) >= g.CLEAR,
                  still=max(float(np.linalg.norm(f.lin_vel)), float(np.linalg.norm(lid.lin_vel))) <= g.STILL,
                  kettle_tilt=kt,
                  kettle_max_tilt=max(_STATE["kettle_max_tilt"], kt),
                  kettle_moved=float(np.linalg.norm(k.pos[:2] - np.asarray(g.KETTLE_XY))))


def _judged() -> bool:
    return bool(_STATE["clear"] and _STATE["still"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def score(objects) -> float:
    """Judged only with both hands 4 cm clear of the filter and the lid and both still (else 0), from the state now:
    1.0 the filter seated on its stop in its slot, curve outward, and the lid back on; 0.5 the filter seated but the lid
    not on; 0.25 the filter in its slot (curve outward) but not down on its stop, lid on or not; 0 otherwise. A kettle
    ever tipped past 20 degrees scores 0."""
    if not _judged() or _STATE["kettle_max_tilt"] > g.TIP_DEG:
        return 0.0
    if _STATE["seated"]:
        return 1.0 if _STATE["lid_on"] else 0.5
    return 0.25 if _STATE["in_slot"] else 0.0
