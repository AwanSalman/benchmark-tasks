"""Scripted full-score demo. The left hand comes in from the robot's left and holds the power strip near its top,
fingers on its front and back. The right hand takes each cap out of the tray by its grip ridge, turns it so its
blades point at the strip, lines it up with a free outlet 4 cm out and pushes it home, then lets go and backs off.
Finally both hands let go and move clear.

    python demo.py task.py demo_out [--max-steps 1500] [--no-record] [--no-brace] [--hires]

``--no-brace`` makes the same moves with the right hand alone: each push shoves the strip back across the table
instead of seating the cap.
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
from demokit import run, wait  # noqa: E402

TOP = g.TOP
# hand rotations: columns are the gripper's x (the fingers close along it), y, z (it points along z)
INSERT = np.column_stack([(0, 0, 1), (0, -1, 0), (1, 0, 0)]).astype(float)    # pointing +x, fingers above/below
BRACE = np.column_stack([(-1, 0, 0), (0, 0, -1), (0, -1, 0)]).astype(float)   # pointing -y, fingers front/back
GRIP_IN_CAP = np.array([-(g.CAP_T + 0.002 + 0.013), 0.0, 0.0])   # the tool point in a cap's frame: pad tips 2 mm
                                                                   # off its face, on the ridge
RELEASE = 0.30                    # gripper opening (of 1) to let go of a fitted cap: about 28 mm
HOME = 0.0003                     # the cap counts as home this close to the face (the scorer allows 1.5 mm)
PUSH = 0.0010                     # aim the cap's back face this far past the strip's face (it stops on the face)
BRACE_IN = 0.030                 # the left hand's pad tips this far across the strip's top
ORDER = (1, 2, 3)                 # outlets to fill, the lowest (the hardest reach) first


def actual_tcp(ctx, side):
    """The measured tool point and hand rotation (station frame)."""
    from yam_arena.constants import TCP_OFFSET

    p, R = ctx.task.body_pose(ctx.emb._env, f"{side}_robot", "gripper")
    return p + R @ np.asarray(TCP_OFFSET), R


def cap(ctx, i):
    return ctx.task.cap_pose(ctx.emb._env, i)


def outlet(ctx, k):
    return ctx.task.outlet_point(ctx.emb._env, k)


def brace(ctx):
    """Left: in from the robot's left above the outlets, fingers either side of the strip, then close on its front and
    back faces: held there, the strip can neither slide nor swivel under the pushes."""
    arm = ctx.left
    sp, sR = ctx.task.strip_pose(ctx.emb._env)
    at = sp + sR @ np.array([0.0, g.STRIP_W / 2 - BRACE_IN + 0.013, g.BRACE_Z])   # pad tips BRACE_IN past its +y side
    yield from arm.move(at + [0.0, 0.10, 0.03], BRACE, 2.5, grip=1.0)
    yield from arm.move(at + [0.0, 0.10, 0.0], BRACE, 0.8)
    yield from arm.move(at, BRACE, 1.5)
    yield from arm.set_grip(0.0, 1.2)
    yield from wait(0.4)


def let_go_strip(ctx):
    arm = ctx.left
    yield from arm.set_grip(1.0, 1.0)
    yield from arm.move(arm.pos + [0, 0.10, 0.0], None, 1.2)
    yield from arm.move(arm.pos + [0, 0.0, 0.04], None, 0.6)


def take_cap(ctx, i):
    """Right: from above, close on the cap's grip ridge, lift it out of the tray and turn its blades toward the strip."""
    arm = ctx.right
    p, R = cap(ctx, i)
    at, rot = p + R @ GRIP_IN_CAP, R @ INSERT
    yield from arm.move(at + [0, 0, 0.07], rot, 2.0, grip=1.0)
    yield from arm.move(at + [0, 0, 0.002], rot, 1.0)
    yield from arm.set_grip(0.0, 0.7)
    yield from wait(0.3)
    yield from arm.move(at + [0, 0, 0.06], rot, 1.0)


def line_up(ctx, i, k, standoff):
    """Right: the hand pose that puts cap i square in front of outlet k, standoff m out, from where the cap sits in
    the hand now (measured), corrected for how far the hand falls short of its command."""
    arm = ctx.right
    tcp_p, _ = actual_tcp(ctx, "right")
    p, R = cap(ctx, i)
    o, sR = outlet(ctx, k)
    rot = (sR @ R.T) @ arm.rot                                   # turn the commanded hand by the cap's error
    rel = arm.rot.T @ (p - tcp_p)                                # the cap's origin from the tool point, in the hand
    goal = o - sR[:, 0] * standoff
    return goal - rot @ rel + (arm.pos - tcp_p), rot


def bring_cap(ctx, i, k):
    arm = ctx.right
    o, sR = outlet(ctx, k)
    _, R = cap(ctx, i)
    pos, rot = line_up(ctx, i, k, 0.08)
    yield from arm.move(pos, rot, 2.2)
    for standoff, seconds in ((0.08, 0.6), (0.03, 1.2), (0.012, 1.0), (0.012, 0.5)):
        yield from wait(0.2)
        pos, rot = line_up(ctx, i, k, standoff)
        yield from arm.move(pos, rot, seconds)
    yield from wait(0.3)


def push_in(ctx, i, k, seconds=6.0, rate=0.0006):
    """Right: push cap i into outlet k along the strip's x, steering every step on the cap's measured offset from
    the outlet's axis; the command never runs more than 10 mm ahead of the hand."""
    arm = ctx.right
    seen = []
    for _ in range(round(seconds * 20)):
        tcp_p, _ = actual_tcp(ctx, "right")
        p, R = cap(ctx, i)
        o, sR = outlet(ctx, k)
        depth = float((p - o) @ sR[:, 0])                         # the cap's back face, past the strip's face
        seen.append(depth)
        if depth >= -HOME:                                       # home (its face on the strip's)
            break
        if depth > -0.003 and len(seen) > 10 and depth - seen[-11] < 0.0001:
            break                                                # no longer moving in: stop pushing
        aim = o + sR[:, 0] * min(PUSH, depth + 0.004)
        if depth < -0.0125:                                      # blades not in yet: stay on the axis, approach slowly
            aim = o + sR[:, 0] * min(depth + rate * 2, -0.0115)
        err = aim - p
        cmd = arm.pos + err * 0.6
        lead = cmd - tcp_p
        n = np.linalg.norm(lead)
        cmd = tcp_p + (lead * 0.006 / n if n > 0.006 else lead)
        rot = (sR @ R.T) @ arm.rot
        arm.aim(cmd, rot)
        yield


def place_cap(ctx, i, k):
    arm = ctx.right
    yield from bring_cap(ctx, i, k)
    yield from push_in(ctx, i, k)
    yield from wait(0.3)
    yield from arm.set_grip(RELEASE, 0.6)     # open just clear of the ridge: wide open, the fingers reach the next cap
    o, sR = outlet(ctx, k)
    yield from arm.move(arm.pos - sR[:, 0] * 0.07, None, 1.0)
    yield from arm.set_grip(1.0, 0.4)


def script(ctx, use_brace: bool = True):
    if use_brace:
        yield from brace(ctx)
    for i, k in zip((2, 1, 0), ORDER):         # the cap nearest the strip first
        yield from take_cap(ctx, i)
        yield from place_cap(ctx, i, k)
    if use_brace:
        yield from let_go_strip(ctx)
    yield from ctx.right.go_home(2.0)
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    env = ctx.emb._env
    sp, sR = ctx.task.strip_pose(env)
    deps = []
    for i in range(g.CAPS):
        p, _ = cap(ctx, i)
        q = sR.T @ (p - sp)
        deps.append(f"({(q[0] - g.FRONT) * 1000:5.1f},{q[1] * 1000:4.1f},{q[2] * 1000:5.1f})")
    return (f"caps {S['caps']} seated {S['seated']} [{' '.join(deps)}] strip moved {S['moved'] * 1000:4.1f} mm "
            f"tilt {S['tilt']:.1f}  plug out {S['plug_out'] * 1000:.1f} mm  clear {S['clearance'] * 1000:5.1f} mm  "
            f"drift {S['drift'] * 1000:5.1f} mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-brace", action="store_true", help="fit the caps with the right hand alone")
    ap.add_argument("--log-every", type=int, default=10)
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704")
    a = ap.parse_args()
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, use_brace=not a.no_brace), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=a.log_every, **size)
