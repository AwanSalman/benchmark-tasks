"""On the station table, in front of the arms: a round black car coin holder (76.2 mm across, 101.6 mm tall, 175 g)
standing in a shallow round tray. Its four spring-loaded tubes hold four quarters, two dimes, two nickels and two
pennies, each stack pushed up against its tube's lip; a coin comes out by sliding it sideways under the lip, along
its tube's exit channel and over the rim, where it drops into the tray.

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

INSTRUCTION = "Take 40 cents out of the coin holder into the tray."

TOP = g.TOP
PU = g.plate_under()
FOLLOWERS = tuple(f"f_{c}" for c in g.COINS)
HOLDER_Z = TOP + g.TRAY_FLOOR + 0.0003


def _f(v):
    return tuple(float(x) for x in v)


HOLDER = UsdBody("holder", usd="assets/holder.usda", pos=_f((*g.HOLDER_XY, HOLDER_Z)),
                 size=_f((2 * g.HOLDER_R, 2 * g.HOLDER_R, g.HOLDER_H)), mass=g.HOLDER_MASS, articulation=True,
                 joint_pos=tuple((j, 0.0) for j in FOLLOWERS),
                 drives=(("f_.*", g.FOLLOWER_K, g.FOLLOWER_DAMPING),), usd_materials=True, solver_iters=32)


def _coin_spec(cid):
    c = g.kind(cid)
    i = int(cid[len(c):])
    d, t, mass = g.COINS[c][:3]
    tx, ty = g.TUBES[c]
    z = HOLDER_Z + PU - t / 2 - g.STACK_GAP - i * (t + g.STACK_GAP)
    return UsdBody(cid, usd=f"assets/{c}.usda", pos=_f((g.HOLDER_XY[0] + tx, g.HOLDER_XY[1] + ty, z)),
                   size=_f((d, d, t)), mass=mass, usd_materials=True, solver_iters=32)


COINS = [_coin_spec(k) for k in g.ALL_COINS]
TRAY = UsdBody("tray", usd="assets/tray.usda", pos=_f((*g.HOLDER_XY, TOP + 0.0003)),
               size=_f((2 * g.TRAY_R, 2 * g.TRAY_R, g.TRAY_H)), mass=g.TRAY_MASS, usd_materials=True, solver_iters=16)
OBJECTS = [HOLDER, *COINS, TRAY]

# -- monitor ----------------------------------------------------------------------------------------------------------
_STATE: dict = {}


def _reset_values() -> None:
    _STATE.update(where={k: "tube" for k in g.ALL_COINS}, tilt=0.0, max_tilt=0.0, moved=0.0, clear=True,
                  still=True, cents=0, kinds=set(), spilled=0, followers={})


@on_reset
def _arm_springs(environment=None) -> None:
    from scenekit import set_joint_targets

    _reset_values()
    if environment is not None:
        set_joint_targets(environment, "holder", {f: g.FOLLOWER_FULL / g.FOLLOWER_K for f in FOLLOWERS})


@before_reset
def _restore(environment) -> None:
    from scenekit import set_joint_targets, write_joint_positions

    # the springs stay on through the reset: switched off, the stacks would sag and then be fired up into the lips
    write_joint_positions(environment, "holder", {f: 0.0 for f in FOLLOWERS})
    set_joint_targets(environment, "holder", {f: g.FOLLOWER_FULL / g.FOLLOWER_K for f in FOLLOWERS})


_reset_values()


def where(holder_pos, holder_R, tray_pos, coin_pos) -> str:
    """'tube' (in a tube, under its lip, its centre within 3 mm of the tube's axis), 'holder' (elsewhere in or on the
    holder: part way out along an exit channel, or on its top),
    'tray', 'bench' (on the table outside the tray) or 'air'."""
    import numpy as np

    p = holder_R.T @ (np.asarray(coin_pos) - np.asarray(holder_pos))
    if p[2] < PU + 0.001 and p[2] > 0.0:
        for c, (tx, ty) in g.TUBES.items():
            if math.hypot(p[0] - tx, p[1] - ty) < g.IN_TUBE:
                return "tube"
    if math.hypot(p[0], p[1]) < g.HOLDER_R + 0.002 and 0.0 < p[2] < g.HOLDER_H + 0.012:
        return "holder"
    rel = np.asarray(coin_pos) - np.asarray(tray_pos)
    if math.hypot(rel[0], rel[1]) < g.TRAY_R - g.TRAY_WALL and rel[2] < g.TRAY_H + 0.012:
        return "tray"
    if coin_pos[2] - TOP < 0.012:
        return "bench"
    return "air"


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


@on_step
def _monitor(environment) -> None:
    import numpy as np

    from yam_arena.scene import object_states

    names = [o.name for o in OBJECTS if o.name not in ("holder", "tray")]
    st = object_states(environment, ["holder", "tray", *names])
    h, tray = st["holder"], st["tray"]
    hR = h.rotation()
    tilt = math.degrees(math.acos(min(1.0, float(hR[2, 2]))))
    hands = hand_points(environment)
    where_now, dist, speed = {}, 1.0, 0.0
    for k in names:
        w = where(h.pos, hR, tray.pos, st[k].pos)
        where_now[k] = w
        if w != "tube":
            dist = min(dist, *(float(np.linalg.norm(hp - st[k].pos)) for hp in hands))
            speed = max(speed, float(np.linalg.norm(st[k].lin_vel)))
    _STATE.update(where=where_now, tilt=tilt, max_tilt=max(_STATE["max_tilt"], tilt),
                  moved=float(np.linalg.norm(h.pos[:2] - np.asarray(g.HOLDER_XY))), hand_dist=dist,
                  clear=dist >= g.CLEAR, still=speed <= g.STILL,
                  cents=sum(g.value(k) for k, w in where_now.items() if w == "tray"),
                  kinds={g.kind(k) for k, w in where_now.items() if w == "tray"},
                  spilled=sum(w in ("bench", "air") for w in where_now.values()),
                  followers={f: float(h.joints.get(f, 0.0)) for f in FOLLOWERS})


def success(objects) -> bool:
    return score(objects) >= 1.0


def score(objects) -> float:
    """Judged from the state now, only with both hands 4 cm clear of every coin out of its tube and those coins still
    (else 0): 1.0 exactly 40 cents in the tray (with these stacks only a quarter, a dime and a nickel make it) and no
    coin on the bench; under 40 cents, 0.25 for each of the quarter, dime and nickel in the tray; over 40 cents, 0.25;
    any coin on the bench (or off the tray and the holder), at most 0.5. A holder ever tipped past 20 degrees: 0."""
    if not (_STATE["clear"] and _STATE["still"]) or _STATE["max_tilt"] > g.TIP_DEG:
        return 0.0
    cents = _STATE["cents"]
    if cents == g.TARGET_CENTS:
        s = 1.0
    elif cents > g.TARGET_CENTS:
        s = 0.25
    else:
        s = 0.25 * sum(c in _STATE["kinds"] for c in g.NEEDED)
    if _STATE["spilled"]:
        s = min(s, 0.5)
    return s
