"""Scripted full-score demo. For each aid: the left hand picks it up across its shell and holds its bottom end out
toward the right hand; the right hand takes the battery door's nail lip between its fingertips and swings the door
open round its hinge; the left hand lays the aid in the drying case's tray and lets go.

    python demo.py task.py demo_out [--max-steps 1500] [--no-record] [--no-doors] [--hires]

``--no-doors`` lays both aids in the tray without opening their doors (0).
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, turn, wait  # noqa: E402

TOP = g.TOP
DOWN_Y = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)              # pointing down, fingers close along y
DOWN_X = turn([0, 0, 1], math.pi / 2) @ DOWN_Y                               # pointing down, fingers close along x
INSERT = np.column_stack([(0, 0, 1), (0, -1, 0), (1, 0, 0)]).astype(float)   # pointing +x, fingers above/below
# the right hand on the door's lip: pointing +y and 30 deg down, fingers either side of the lip (closing along x)
PINCH = turn([0, 0, 1], math.pi / 2) @ turn([0, 1, 0], math.radians(30)) @ INSERT @ turn([0, 0, 1], math.pi / 2)
LIP_IN_DOOR = np.array([-g.TAB[2] + 0.0025 - g.HINGE[0], -g.AID_D / 2 + g.TAB[0] / 2 - g.HINGE[1], 0.0])  # 2 mm in
PICK_AT = 0.021            # the left hand's pinch, this far up the shell from the bottom end (on its parallel sides)
HOLD = 0.10                # its grip: just under the aid's 12 mm (fully shut, the fingers give along their axis)
OPEN_TO = 95.0             # deg
GRAB_DEPTH = 0.012         # the lip this far along the right pads from the tool point (their tips 13 mm out)
LAY_GRIP = 0.25            # open this far to let an aid go in the tray (wider, the fingers meet the other aid)


def tcp_pose(ctx, side):
    from yam_arena.constants import TCP_OFFSET

    p, R = ctx.task.body_pose(ctx.emb._env, f"{side}_robot", "gripper")
    return p + R @ np.asarray(TCP_OFFSET), R


def sag(ctx, side):
    """How far the hand's measured tool point falls short of its command (add it to the command)."""
    arm = ctx.left if side == "left" else ctx.right
    return arm.pos - tcp_pose(ctx, side)[0]


def yaw_of(R) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def pick(ctx, s):
    """Left: pinch the aid across its back and front, up its shell, and lift it."""
    L = ctx.left
    p, R = ctx.task.aid_pose(ctx.emb._env, s)
    rot = turn([0, 0, 1], yaw_of(R) - math.pi / 2) @ DOWN_X
    at = p + R @ np.array([PICK_AT, 0.0, 0.0])
    at[2] = TOP + 0.0025 + 0.013                       # pad tips 2.5 mm off the table
    yield from L.move(at + [0, 0, 0.08], rot, 2.0, grip=0.5)
    yield from L.move(at, rot, 1.2)
    yield from L.set_grip(HOLD, 0.8)
    yield from wait(0.3)
    yield from L.move(at + [0, 0, 0.05], rot, 1.0)


def present(ctx, s):
    """Left: bring the aid's bottom end to the place where the door is opened."""
    L = ctx.left
    p, _ = ctx.task.aid_pose(ctx.emb._env, s)
    yield from L.move(L.pos + (np.asarray(g.WORK) - p), None, 1.5)
    yield from wait(0.4)


def open_door(ctx, s):
    """Right: fingertips either side of the door's lip, then turn the hand with the door about its hinge pin."""
    R = ctx.right
    env = ctx.emb._env
    dp, dR = ctx.task.door_pose(env, s)
    _, bR = ctx.task.aid_pose(env, s)
    hinge = dp.copy()
    held = dp + dR @ LIP_IN_DOOR
    pinch = turn([0, 0, 1], yaw_of(bR) - math.pi / 2) @ PINCH          # as the aid sits in the left hand
    z = pinch[:, 2]
    grab = held - z * GRAB_DEPTH                                        # the lip this far along the pads
    yield from R.move(grab - z * 0.07, pinch, 1.8, grip=0.4)
    yield from R.move(grab - z * 0.03, pinch, 0.7)
    for back, secs in ((0.03, 0.4), (0.01, 0.6), (0.0, 0.5), (0.0, 0.3)):
        yield from wait(0.1)
        yield from R.move(grab - z * back + sag(ctx, "right"), pinch, secs)
    yield from R.set_grip(0.0, 0.6)
    yield from wait(0.3)
    off = sag(ctx, "right")                    # held fixed: re-measuring every step makes the hand oscillate
    for k in range(1, 101):
        Rz = turn([0, 0, 1], -math.radians(OPEN_TO) * k / 100)
        R.aim(hinge + Rz @ (grab - hinge) + off, Rz @ pinch)
        yield
    yield from wait(0.3)
    yield from R.set_grip(0.4, 0.5)
    yield from R.move(R.pos - R.rot[:, 2] * 0.05, None, 0.7)
    yield from R.move(R.pos + [0, -0.06, 0.06], None, 0.8)


def lay(ctx, s):
    """Left: turn the aid square to the tray, lower it in, let go and rise clear."""
    L = ctx.left
    env = ctx.emb._env
    p, R = ctx.task.aid_pose(env, s)
    wp, cR = ctx.task.well_place(env, s)
    tcp, _ = tcp_pose(ctx, "left")
    turn_by = (yaw_of(cR) + math.pi / 2) - yaw_of(R)                    # square the aid up with the tray
    Rz = turn([0, 0, 1], turn_by)
    goal = wp - Rz @ (p - tcp) + (L.pos - tcp)
    yield from L.move(goal + [0, 0, 0.06], Rz @ L.rot, 2.0)
    yield from L.move(goal + [0, 0, 0.012], None, 1.0)
    yield from L.move(goal + [0, 0, 0.002], None, 0.6)
    yield from L.set_grip(LAY_GRIP, 0.5)
    yield from wait(0.3)
    yield from L.move(L.pos + [0, 0, 0.07], None, 1.0)


def script(ctx, doors: bool = True):
    for s in ("left", "right"):
        yield from pick(ctx, s)
        if doors:
            yield from present(ctx, s)
            yield from open_door(ctx, s)
        yield from lay(ctx, s)
    yield from ctx.left.go_home(2.0)
    yield from ctx.right.go_home(1.5)
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    env = ctx.emb._env
    parts = []
    for s in g.AIDS:
        p, _ = ctx.task.aid_pose(env, s)
        parts.append(f"{s}: door {S['doors'][s]:5.1f} in case {S['in_case'][s]} at ({p[0]*1000:5.1f},{p[1]*1000:5.1f},"
                     f"{(p[2]-TOP)*1000:5.1f})")
    return f"{' | '.join(parts)} | clear {S['clearance']*1000:5.1f} drift {S['drift']*1000:5.1f}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-doors", action="store_true", help="lay the aids in the tray with their doors shut")
    ap.add_argument("--log-every", type=int, default=20)
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704")
    a = ap.parse_args()
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, doors=not a.no_doors), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=a.log_every, **size)
