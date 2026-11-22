"""On the station table, in front of the arms: two behind-the-ear hearing aids lying on their sides, their battery
doors shut, and an open drying case with a tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step, set_joint_targets, write_joint_positions  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Open the battery door on each hearing aid, then lay both aids in the drying case for the night, "
               "doors open.")

# -- objects --------------------------------------------------------------------------------------------------------
_S45 = math.sqrt(0.5)
AIDS = {side: UsdBody(f"aid_{side}", usd=f"assets/aid_{side}.usda",
                      pos=(g.AID_START[side][0], g.AID_START[side][1], g.AID_Z),
                      rot_xyzw=(0.0, 0.0, _S45, _S45),           # bottom end toward -y, the hook toward the robot
                      size=(0.012, 0.05, 0.009), articulation=True,
                      joint_pos=(("door_joint", 0.0),), drives=(("door_joint", g.DOOR_K, g.DOOR_DAMPING),),
                      friction=(0.8, 0.7), solver_iters=32)
        for side in g.AIDS}
CASE = UsdBody("drying_case", usd="assets/case.usda", pos=(g.CASE_C[0], g.CASE_C[1], g.TOP + 0.0005),
               size=g.CASE_SIZE, mass=g.CASE_MASS, friction=(0.9, 0.8), solver_iters=16)
OBJECTS = [*AIDS.values(), CASE]

# Finger pads, from each finger link's origin (at the finger's base) in the gripper link's frame: the pad's tip and a
# point 3 cm back along it (yam_arena's URDF).
PADS = {"tip_left": [(-0.0445, 0.024, 0.0903), (-0.0445, 0.024, 0.0603)],
        "tip_right": [(0.0445, -0.024, 0.0903), (0.0445, -0.024, 0.0603)]}
# the aid's extent in its own frame with its door open (for the hands' clearance)
AID_BOX = ((-0.014, -0.008, -g.AID_T / 2), (0.054, 0.016, g.AID_T / 2))

_STATE: dict = {}


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy()


def _rotation(xyzw):
    import numpy as np

    x, y, z, w = xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def body_pose(environment, asset_name: str, body: str):
    """A body's origin and rotation, station frame."""
    base = environment.unwrapped
    asset = base.scene[asset_name]
    origin = _np(base.scene.env_origins[0])
    i = list(asset.body_names).index(body)
    pose = _np(asset.data.body_link_pose_w)[0, i]
    return pose[:3] - origin, _rotation(pose[3:7])


def aid_pose(environment, side: str):
    return body_pose(environment, f"aid_{side}", "body")


def door_pose(environment, side: str):
    return body_pose(environment, f"aid_{side}", "door")


def case_pose(environment):
    return body_pose(environment, "drying_case", "body")


def door_angle(environment, side: str) -> float:
    """How far an aid's battery door is open, deg (0 shut)."""
    asset = environment.unwrapped.scene[f"aid_{side}"]
    i = asset.find_joints(["door_joint"])[0][0]
    return -math.degrees(float(_np(asset.data.joint_pos)[0, i]))


def well_place(environment, side: str):
    """Where the demo lays this aid's bottom end in the tray, and the case's rotation (station frame)."""
    import numpy as np

    p, R = case_pose(environment)
    return p + R @ np.array([g.PLACE_DX[side] + g.IN_WELL[0], g.IN_WELL[1], g.WELL_FLOOR - g.TOP + g.AID_T / 2]), R


def pad_points(environment) -> list:
    import numpy as np

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    points = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pose = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        R = _rotation(pose[names.index("gripper"), 3:7])
        for link, offsets in PADS.items():
            at = pose[names.index(link), :3] - origin
            points += [at + R @ np.asarray(o) for o in offsets]
    return points


def _box_distance(q, lo, hi) -> float:
    import numpy as np

    d = np.maximum(np.maximum(np.asarray(lo) - q, q - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(d))


def _reset_values() -> None:
    _STATE.update(doors={s: 0.0 for s in g.AIDS}, in_case={s: False for s in g.AIDS}, clearance=1.0, history=[],
                  drift=1.0)


@on_reset
def _reset_state(environment=None) -> None:
    _reset_values()
    if environment is not None:
        for s in g.AIDS:
            set_joint_targets(environment, f"aid_{s}", {"door_joint": 0.0})


@before_reset
def _restore(environment) -> None:
    for s in g.AIDS:
        write_joint_positions(environment, f"aid_{s}", {"door_joint": 0.0})


_reset_values()


@on_step
def _monitor(environment) -> None:
    """After every control step: the battery doors' detents, which aid lies in the case's tray, whether the hands are
    clear of the aids, and whether everything is still."""
    import numpy as np

    cp, cR = case_pose(environment)
    doors, in_case, centres = {}, {}, []
    clearance = 1.0
    pads = pad_points(environment)
    for s in g.AIDS:
        # the detent: within DETENT of shut the door's spring pulls it shut; beyond, it stays where it is put
        a = door_angle(environment, s)
        set_joint_targets(environment, f"aid_{s}", {"door_joint": -math.radians(0.0 if a < g.DETENT else a)})
        doors[s] = a
        p, R = aid_pose(environment, s)
        c = p + R @ np.asarray(g.AID_CENTRE)
        centres.append(c)
        flat = math.degrees(math.acos(min(1.0, abs(float(R[2, 2])))))
        q = cR.T @ (c - cp)                        # its centre in the case's frame
        rest = g.WELL_FLOOR - g.TOP + g.AID_T / 2  # its centre's height lying on the tray's floor
        in_case[s] = bool(abs(q[0]) <= g.WELL[0] / 2 - g.IN_MARGIN and abs(q[1]) <= g.WELL[1] / 2 - g.IN_MARGIN
                          and -0.003 <= q[2] - rest <= g.IN_Z and flat <= g.FLAT)
        for pt in pads:
            clearance = min(clearance, _box_distance(R.T @ (pt - p), *AID_BOX))
    hist = _STATE["history"]
    hist.append(np.concatenate(centres))
    del hist[:-20]
    drift = max(float(np.max(np.linalg.norm((h - hist[-1]).reshape(-1, 3), axis=1))) for h in hist) \
        if len(hist) == 20 else 1.0
    _STATE.update(doors=doors, in_case=in_case, clearance=clearance, drift=drift)


def _open(s) -> bool:
    return _STATE["doors"][s] >= g.DOOR_OPEN


def success(objects) -> bool:
    """Both aids lying in the case's tray with their battery doors open 60 deg or more, every finger pad 2 cm clear of
    both aids and nothing moving (under 2 mm over the last second)."""
    S = _STATE
    return bool(all(S["in_case"][s] and _open(s) for s in g.AIDS) and S["clearance"] >= g.LET_GO
                and S["drift"] < g.STILL)


def score(objects) -> float:
    """Success scores 1.0. Otherwise, for each aid: 0.25 if its battery door is open 60 deg or more, and 0.25 more if
    it also lies in the case's tray; at most 0.75 (for example both done but an aid still held). An aid laid in the
    tray with its door shut scores nothing."""
    if success(objects):
        return 1.0
    return min(0.75, sum(0.25 * _open(s) + 0.25 * (_open(s) and _STATE["in_case"][s]) for s in g.AIDS))
