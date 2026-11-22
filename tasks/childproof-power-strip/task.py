"""On the station table, in front of the arms: an upright four-outlet power strip standing free on its rubber feet, a
desk lamp plugged into its bottom outlet, and a foam tray holding three child-safety outlet caps.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Childproof the power strip: put a safety cap into every outlet that is not in use, then let go.")

TOP = g.TOP
LEAVES = tuple(f"leaf{k}{s}{side}" for k in range(len(g.OUTLET_Z)) for s in range(2) for side in range(2))
FREE_OUTLETS = tuple(k for k in range(len(g.OUTLET_Z)) if k != g.PLUG_OUTLET)

# -- objects --------------------------------------------------------------------------------------------------------
_S45 = math.sqrt(0.5)
STRIP = UsdBody("strip", usd="assets/strip.usda", pos=(g.STRIP_X, g.STRIP_Y, TOP + 0.0005),
                size=(g.STRIP_D, g.STRIP_W, g.STRIP_H), articulation=True,
                joint_pos=tuple((j, 0.0) for j in LEAVES), drives=(("leaf.*", g.LEAF_K, 2.0),),
                friction=(g.STRIP_FRICTION, g.STRIP_FRICTION - 0.05))
CAPS = [UsdBody(f"cap{i}", usd=f"assets/cap{i}.usda",
                pos=(g.TRAY_X, g.TRAY_Y + dy, TOP + g.TRAY_SIZE[2] + 0.0005),
                rot_xyzw=(0.0, _S45, 0.0, _S45),                     # blades down into the tray's slits, ridge up
                size=(0.04, 0.04, 0.03), mass=g.CAP_MASS, friction=(g.CAP_FRICTION, g.CAP_FRICTION - 0.1),
                torsional_patch=0.006, solver_iters=32)
        for i, dy in enumerate(g.CAP_DY)]
TRAY = UsdBody("cap_tray", usd="assets/tray.usda", pos=(g.TRAY_X, g.TRAY_Y, TOP), size=g.TRAY_SIZE, mass=None,
               friction=(0.9, 0.8))
LAMP = UsdBody("desk_lamp", usd="assets/lamp.usda", pos=(g.LAMP_X, g.LAMP_Y, TOP), size=(0.11, 0.11, 0.34),
               articulation=True, fix_root=True, drives=(("j.*", g.CORD_K, g.CORD_DAMPING),), friction=(0.8, 0.7))
OBJECTS = [STRIP, *CAPS, TRAY, LAMP]

# Finger pads, from each finger link's origin (at the finger's base) in the gripper link's frame: the pad's tip and a
# point 3 cm back along it (yam_arena's URDF).
PADS = {"tip_left": [(-0.0445, 0.024, 0.0903), (-0.0445, 0.024, 0.0603)],
        "tip_right": [(0.0445, -0.024, 0.0903), (0.0445, -0.024, 0.0603)]}

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


def strip_pose(environment):
    return body_pose(environment, "strip", "body")


def cap_pose(environment, i: int):
    return body_pose(environment, f"cap{i}", "body")


def plug_pose(environment):
    return body_pose(environment, "desk_lamp", "plug")


def outlet_point(environment, k: int):
    """Outlet k's face centre and the strip's rotation (x into the outlet), station frame."""
    import numpy as np

    p, R = strip_pose(environment)
    return p + R @ np.array([g.FRONT, 0.0, g.OUTLET_Z[k]]), R


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


def _in_strip(p, sp, sR):
    """A station point in the strip's frame."""
    return sR.T @ (p - sp)


def _box_distance(q, lo, hi) -> float:
    import numpy as np

    d = np.maximum(np.maximum(np.asarray(lo) - q, q - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(d))


def _reset_values() -> None:
    _STATE.update(strip0=None, caps=[None] * g.CAPS, seated=0, plug_out=0.0, plug_in=True, pulled=False,
                  moved=0.0, tilt=0.0, strip_ok=True, clearance=1.0, history=[], drift=1.0)


@on_reset
def _reset_state(environment=None) -> None:
    from scenekit import set_joint_targets

    _reset_values()
    if environment is not None:
        # each contact leaf's spring pulls it 3 mm past its stop, so it presses on a blade from the start
        set_joint_targets(environment, "strip", {j: -g.LEAF_PRELOAD for j in LEAVES})


@before_reset
def _restore(environment) -> None:
    from scenekit import write_joint_positions

    write_joint_positions(environment, "strip", {j: 0.0 for j in LEAVES})


_reset_values()


@on_step
def _monitor(environment) -> None:
    """After every control step: which caps sit home in which outlet, whether the lamp's plug is still in, how far
    the strip has moved, whether the hands are clear, and whether everything is still."""
    import numpy as np

    sp, sR = strip_pose(environment)
    if _STATE["strip0"] is None:
        _STATE["strip0"] = sp.copy()
    moved = float(np.linalg.norm((sp - _STATE["strip0"])[:2]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(sR[2, 2])))))

    # caps: home in a free outlet (back face on the strip's face, centred, square)
    where, seated, points = [], 0, []
    for i in range(g.CAPS):
        cp, cR = cap_pose(environment, i)
        q = _in_strip(cp, sp, sR)
        ax = sR.T @ cR[:, 0]
        square = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[0])))))
        home = None
        for k in FREE_OUTLETS:
            if (q[0] >= g.FRONT - g.SEATED and abs(q[1]) <= g.CENTRED and abs(q[2] - g.OUTLET_Z[k]) <= g.CENTRED
                    and square <= g.SQUARE):
                home = k
        where.append(home)
        seated += home is not None
        points.append(cp)

    # the lamp's plug
    pp, pR = plug_pose(environment)
    q = _in_strip(pp, sp, sR)
    out = float(g.FRONT - q[0])
    lateral = max(abs(q[1]), abs(q[2] - g.OUTLET_Z[g.PLUG_OUTLET]))
    plug_in = out <= g.PLUG_IN and lateral <= 0.003
    pulled = out > g.PLUG_OUT or lateral > 0.004

    # the hands: every pad clear of the strip (with its plug) and of every cap
    clearance = 1.0
    for pt in pad_points(environment):
        qs = _in_strip(pt, sp, sR)
        d = _box_distance(qs, (g.FRONT - g.PLUG_D, -g.STRIP_W / 2, 0.0), (g.STRIP_D / 2, g.STRIP_W / 2, g.STRIP_H))
        for cp in points:
            d = min(d, float(np.linalg.norm(pt - cp)) - g.CAP_R)
        clearance = min(clearance, d)

    hist = _STATE["history"]
    hist.append(np.concatenate([sp, *points]))
    del hist[:-20]
    drift = max(float(np.max(np.linalg.norm((h - hist[-1]).reshape(-1, 3), axis=1))) for h in hist) \
        if len(hist) == 20 else 1.0
    _STATE.update(caps=where, seated=seated, plug_out=out, plug_in=plug_in, pulled=pulled, moved=moved, tilt=tilt,
                  strip_ok=moved <= g.STRIP_MOVED and tilt <= g.STRIP_TILT, clearance=clearance, drift=drift)


def success(objects) -> bool:
    """All three caps home in the three free outlets, the lamp still plugged in, the strip where it stood (within 10
    mm, upright within 5 deg), every finger pad 2 cm clear and nothing moving (under 2 mm over the last second)."""
    S = _STATE
    return bool(S["seated"] == g.CAPS and S["plug_in"] and S["strip_ok"] and S["clearance"] >= g.LET_GO
                and S["drift"] < g.STILL)


def score(objects) -> float:
    """Success scores 1.0. Otherwise 0.25 per cap home in a free outlet, at most 0.5 if the strip was shoved more than
    10 mm or tipped more than 5 deg, or the lamp's plug was pulled more than 5 mm out."""
    if success(objects):
        return 1.0
    s = 0.25 * _STATE["seated"]
    if _STATE["pulled"] or not _STATE["strip_ok"]:
        s = min(s, 0.5)
    return s
