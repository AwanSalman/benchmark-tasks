"""Scripted full-score demo of changing the click pen's refill, and two shortcut variants. The right hand does all three
steps; the left stays at its ready pose.

1. Spring: the hand picks the spring up by its top from where it stands on end, holds it straight over the barrel's
   open back end, lowers its lower end 16 mm down the bore (re-centring it on the bore's axis on the way) and lets go;
   it drops down the bore onto the seat in the nose.
2. Refill: the hand lifts the new refill out of the stand, point down, holds it straight over the barrel's back end,
   lowers its point 27 mm down the bore (re-centring on the way) and lets go; it drops through the spring and rests on
   it by its stop.
3. Clicker: the hand lifts the clicker out of the stand, turns it (hand and all, about the vertical) until its hook
   faces the barrel's clip, lowers it into the back end over the refill's end and lets go; it settles 8 mm out. Then
   the finger tips take its button cap and turn it until the hook faces the clip again, and the hand goes back to its
   ready pose.

The demo reads the parts' and hands' poses from the simulator to steer (as a person watches the pen); the arms are
moved only by joint commands through the standard 14-value interface.

    python demo.py task.py OUT [--hires] [--no-record] [--no-spring | --stop-at-refill]

--no-spring: steps 2-3 without the spring (the pen is put together without it: 0).
--stop-at-refill: steps 1-2 only (spring in, refill in through it, no clicker: 0.5).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, turn, wait  # noqa: E402

TOP = g.TOP
DOWN_Y = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
DOWN_X = turn([0, 0, 1], math.pi / 2) @ DOWN_Y                               # pointing down, fingers close along x
SPRING_GRIP = 0.03                     # just under the spring's width (the fingers open 94 mm x grip)
CAP_GRIP = 0.045                       # just under the button cap's 5 mm
INSERT = 0.016                         # the spring's end this far down the bore before it is let go
MARGIN = 0.003                                                               # clearance over the barrel's back end
RELEASE = 0.15                         # the grip opened just enough to let a small part go
OVER = TOP + 0.150                     # travel height near the stand: the finger tips clear the barrel's back end


def rehome(arm):
    """Back at the ready pose: restart the arm's IK from the ready joint angles, so later moves don't depend on how
    the arm got there."""
    from yam_arena.constants import READY

    arm.q = np.array(READY, float)


def pose(ctx, n):
    return ctx.task.pose(ctx.emb._env, n)


def barrel_top(ctx):
    """The barrel's back end (the centre of its opening) and its axis, pointing up out of it."""
    bp, bR = pose(ctx, "barrel")
    return bp + bR[:, 0] * g.BARREL_L, bR[:, 0]


def status(ctx, what):
    s, t = ctx.task._STATE, ctx.task
    print(f"   {what}: spring x {s['spring_x']*1000:6.1f}/{s['spring_off']*1000:4.1f} on-refill "
          f"{s['spring_along']*1000:6.1f}/{s['spring_on_off']*1000:4.1f} placed {t.spring_placed()} | point "
          f"{s['point_x']*1000:6.1f}/{s['point_off']*1000:4.1f} axis {s['refill_axis']:4.1f} in {t.refill_in()} | "
          f"clicker out {s['clicker_out']*1000:6.1f} off {s['clicker_off']*1000:4.1f} axis {s['clicker_axis']:4.1f} "
          f"hook {s['hook_deg']:5.1f} in {t.clicker_in()} | ik L {ctx.left.worst_mm:.1f} R {ctx.right.worst_mm:.1f}",
          flush=True)


def tilt(ctx, name):
    _, R = pose(ctx, name)
    return math.degrees(math.acos(min(1.0, abs(R[2, 0]))))


def align(ctx, arm, point_of, target_of, z_of, steps=6, tol=0.0005, part=None):
    """Move the held part's ``point_of(ctx)`` over ``target_of(ctx)`` in x and y, the hand's height ``z_of(ctx)``."""
    for k in range(steps):
        err = target_of(ctx) - point_of(ctx)
        goal = np.array([arm.pos[0] + err[0], arm.pos[1] + err[1], z_of(ctx)])
        yield from arm.move(goal, None, 2.0 if k == 0 else 0.8)
        yield from wait(0.3)
        yield from wait(0.25)
        side = 0 if arm.side == "left" else 1
        miss = np.linalg.norm(ctx.task.tool_points(ctx.emb._env)[side] - arm.pos) * 1000
        print(f"      align {k}: {np.hypot(*err[:2])*1000:.2f} mm, hand off target {miss:.1f} mm"
              + (f", {part} tilt {tilt(ctx, part):.1f} deg" if part else ""), flush=True)
        if np.hypot(*err[:2]) < tol:
            break


def straighten(ctx, arm, part, tries=2):
    """Tip the hand so the held part hangs straight down (it leans a little in the grip)."""
    for _ in range(tries):
        yield from wait(0.3)
        _, pR = pose(ctx, part)
        a = pR[:, 0] * np.sign(pR[2, 0])
        ax = np.cross(a, [0.0, 0.0, 1.0])
        if np.linalg.norm(ax) < 1e-4:
            return
        yield from arm.move(arm.pos, turn(ax / np.linalg.norm(ax), math.asin(min(1.0, np.linalg.norm(ax)))) @ arm.rot,
                            0.6)


def lower_in(ctx, arm, end_of, part, depths, over=None):
    """Lower the held part's lower end ``end_of(ctx)`` down the barrel's bore in stages, centred at each on its axis (or
    over the point ``over(ctx)``, in x and y)."""
    for depth, secs in depths:
        for k in range(2):
            top, up = barrel_top(ctx)
            target = top - up * depth
            if over is not None:
                target[:2] = over(ctx)[:2]
            yield from arm.move(arm.pos + (target - end_of(ctx)), None, secs if k == 0 else 0.4)
            yield from wait(0.2)
        print(f"      in {depth*1000:+.0f} mm: off axis {np.linalg.norm((end_of(ctx) - target)[:2])*1000:.2f} mm, "
              f"{part} tilt {tilt(ctx, part):.1f} deg", flush=True)


def hook_error(ctx) -> float:
    """The clicker's hook side from the barrel's clip side, turned about the barrel's axis (rad, signed)."""
    _, bR = pose(ctx, "barrel")
    _, cR = pose(ctx, "clicker")
    axis = bR[:, 0]
    hook = cR[:, 1] - axis * (cR[:, 1] @ axis)
    clip = bR[:, 1] - axis * (bR[:, 1] @ axis)
    return math.atan2(float(np.cross(clip, hook) @ axis), float(clip @ hook))


def turn_clicker(ctx, arm):
    """Take the clicker's button cap between the finger tips and turn it, hand and all about the vertical, until its hook
    faces the clip (it turns freely in the bore)."""
    for k in range(3):
        cp, cR = pose(ctx, "clicker")
        out = (cp + cR[:, 0] * g.CLICKER_L - barrel_top(ctx)[0]) @ barrel_top(ctx)[1]
        if not 0.004 < out < 0.012:                                 # not seated in the barrel: nothing to turn
            print(f"      clicker not seated ({out*1000:.1f} mm out)", flush=True)
            return
        err = hook_error(ctx)
        print(f"      hook {math.degrees(err):+.1f} deg", flush=True)
        if abs(err) < math.radians(6):
            return
        cp, cR = pose(ctx, "clicker")
        button = cp + cR[:, 0] * g.CLICKER_L
        yield from arm.set_grip(0.4, 0.4)
        yield from arm.move(np.array([button[0], button[1], button[2] + PAD_TIP + 0.02]), None, 0.8)
        yield from arm.move(np.array([button[0], button[1], button[2] + PAD_TIP - 0.003]), None, 0.8)
        yield from arm.set_grip(CAP_GRIP, 0.5)                     # a light hold on the 5 mm cap
        yield from wait(0.2)
        yield from arm.move(arm.pos, turn([0.0, 0.0, 1.0], -err) @ arm.rot, 0.3 + 1.2 * abs(err))
        yield from wait(0.3)
        yield from arm.set_grip(0.4, 1.0)
        yield from arm.move(arm.pos + [0, 0, 0.02], None, 0.5)


def spring_in(ctx):
    L = ctx.right                                                   # (the right hand: it is the steadier over the stand)
    sp, _ = pose(ctx, "spring")
    grip_h = g.SPRING_L + 0.004                                     # the hand's tool point 4 mm above the spring's top:
                                                                    # the pads hold its top 9 mm
    at = np.array([sp[0], sp[1], TOP + grip_h])
    yield from L.move(np.array([at[0], at[1], OVER]), DOWN_X, 2.0, grip=0.4)
    yield from L.move(at, DOWN_X, 1.0)
    yield from L.set_grip(SPRING_GRIP, 0.8)
    yield from wait(0.3)
    top, _ = barrel_top(ctx)
    hand_z = top[2] + MARGIN + grip_h                               # the spring's lower end 5 mm over the opening
    yield from L.move(np.array([L.pos[0], L.pos[1], hand_z + 0.03]), DOWN_X, 1.2)
    for k in range(2):                       # tip the hand so the spring hangs straight down (it leans in the grip)
        yield from wait(0.3)
        _, sR = pose(ctx, "spring")
        a = sR[:, 0] * np.sign(sR[2, 0])
        ax = np.cross(a, [0.0, 0.0, 1.0])
        if np.linalg.norm(ax) < 1e-4:
            break
        ang = math.asin(min(1.0, np.linalg.norm(ax)))
        yield from L.move(L.pos, turn(ax / np.linalg.norm(ax), ang) @ L.rot, 0.6)

    def bottom(c):
        sp, sR = pose(c, "spring")
        return sp - sR[:, 0] * (g.SPRING_L / 2) * np.sign(sR[2, 0])

    yield from align(ctx, L, bottom, lambda c: barrel_top(c)[0], lambda c: hand_z, part="spring")
    # its lower end down the bore in stages, centred over the bore's axis at each (the finger tips stay above the
    # opening), so the bore keeps it upright when the fingers open
    for depth, secs in ((-0.002, 0.8), (0.004, 0.6), (0.008, 0.6), (INSERT, 0.8)):
        for k in range(2):
            top, up = barrel_top(ctx)
            target = top - up * depth                              # on the bore's axis, this far down from the opening
            err = target - bottom(ctx)
            yield from L.move(L.pos + err, None, secs if k == 0 else 0.4)
            yield from wait(0.2)
        print(f"      in {depth*1000:+.0f} mm: off axis {np.linalg.norm((bottom(ctx) - target)[:2])*1000:.2f} mm, "
              f"spring tilt {tilt(ctx, 'spring'):.1f} deg", flush=True)
    yield from L.set_grip(0.3, 0.5)
    yield from wait(1.0)
    yield from L.move(L.pos + [0, 0, 0.03], None, 0.6)
    yield from L.go_home(1.5)
    rehome(L)
    status(ctx, "1 spring dropped in")


def refill_in(ctx):
    R = ctx.right
    rp, _ = pose(ctx, "refill")
    hold = 0.045                                                    # the hand's tool point 45 mm up, near its balance
    at = rp + [0, 0, hold]
    yield from R.move(np.array([at[0], at[1], OVER]), DOWN_X, 2.0, grip=0.4)
    yield from R.move(at, DOWN_X, 1.0)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    top, _ = barrel_top(ctx)
    hand_z = top[2] + MARGIN + hold                                 # the point 5 mm over the opening
    yield from R.move(np.array([R.pos[0], R.pos[1], hand_z + 0.02]), DOWN_X, 1.5)
    point = lambda c: pose(c, "refill")[0]                          # noqa: E731
    yield from straighten(ctx, R, "refill")
    yield from align(ctx, R, point, lambda c: barrel_top(c)[0], lambda c: hand_z, part="refill")
    # in, in stages with the point on the barrel's axis, until the finger tips are 5 mm over the opening
    yield from lower_in(ctx, R, point, "refill", ((-0.002, 0.8), (0.005, 0.6), (0.012, 0.6), (hold - PAD_TIP - 0.005, 1.0)))
    yield from wait(0.3)
    yield from R.set_grip(RELEASE, 0.4)
    yield from wait(1.5)
    yield from R.move(R.pos + [0, 0, 0.04], None, 0.6)
    yield from R.go_home(1.5)
    rehome(R)
    status(ctx, "2 refill in")


def clicker_in(ctx):
    L = ctx.right                                                   # (the right hand: the left can't reach this high
                                                                    # over the stand accurately)
    cp, _ = pose(ctx, "clicker")
    hold = 0.036                                                    # the hand's tool point 36 mm up the clicker
    at = cp + [0, 0, hold]
    yield from L.move(np.array([at[0], at[1], OVER]), DOWN_X, 2.0, grip=0.4)
    yield from L.move(at, DOWN_X, 1.0)
    yield from L.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from L.move(L.pos + [0, 0, 0.05], DOWN_X, 1.0)
    yield from wait(0.3)
    drop = L.pos[2] - pose(ctx, "clicker")[0][2]                    # how far its lower end hangs below the hand
    top, _ = barrel_top(ctx)
    hand_z = top[2] + MARGIN + drop                                 # its lower end 3 mm over the opening
    yield from L.move(np.array([L.pos[0], L.pos[1], top[2] + 0.015 + drop]), DOWN_X, 1.0)   # its end 15 mm over
    yield from L.move(np.array([top[0], top[1], top[2] + 0.015 + drop]), DOWN_X, 1.5)
    yield from straighten(ctx, L, "clicker")
    lower = lambda c: pose(c, "clicker")[0]                         # noqa: E731

    def refill_end(c):
        rp, rR = pose(c, "refill")
        return rp + rR[:, 0] * g.REFILL_L

    # turn the hand about the vertical until the hook faces the clip, then centre it
    for _ in range(3):
        err = hook_error(ctx)
        print(f"      hook before going in {math.degrees(err):+.1f} deg", flush=True)
        if abs(err) < math.radians(3):
            break
        yield from L.move(L.pos, turn([0.0, 0.0, 1.0], -err) @ L.rot, 0.4 + 1.0 * abs(err))
        yield from wait(0.3)
    yield from align(ctx, L, lower, lambda c: barrel_top(c)[0], lambda c: hand_z, part="clicker")
    # in, in stages on the barrel's axis, until the finger tips are 5 mm over the opening; let go and it slides down
    # over the refill's end
    yield from lower_in(ctx, L, lower, "clicker", ((-0.002, 0.8), (0.004, 0.6), (drop - PAD_TIP - 0.005, 1.0)))
    yield from L.set_grip(RELEASE, 0.4)
    yield from wait(1.5)
    status(ctx, "3a clicker let go")
    yield from L.move(L.pos + [0, 0, 0.01], None, 0.4)
    yield from turn_clicker(ctx, L)
    yield from L.move(L.pos + [0, 0, 0.03], None, 0.6)
    yield from L.go_home(1.5)
    rehome(L)
    status(ctx, "3 clicker in, hands away")


def script(ctx, mode):
    yield from wait(0.5)
    if mode != "no-spring":
        yield from spring_in(ctx)
    yield from refill_in(ctx)
    if mode != "stop-at-refill":
        yield from clicker_in(ctx)
    for arm in (ctx.left, ctx.right):
        yield from arm.go_home(1.0)
    yield from wait(1.5)
    status(ctx, "end")


def log(ctx, step, info):
    s = ctx.task._STATE
    if not s:
        return ""
    return (f"barrel tilt {tilt(ctx, 'barrel'):4.1f} | L {ctx.left.pos[2]-TOP:.3f} grip {ctx.left.grip:.2f} | "
            f"spring x {s['spring_x']*1000:6.1f} | point {s['point_x']*1000:6.1f}/{s['point_off']*1000:4.1f} | "
            f"clicker out {s['clicker_out']*1000:6.1f} hook {s['hook_deg']:5.1f} | clear {s['clear']*1000:5.1f} still "
            f"{s['still']} | ik L {ctx.left.worst_mm:.1f} R {ctx.right.worst_mm:.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--no-spring", action="store_true", help="put the pen together without the spring (0)")
    mode.add_argument("--stop-at-refill", action="store_true", help="stop after the refill is in (0.5)")
    a = ap.parse_args()
    m = "no-spring" if a.no_spring else "stop-at-refill" if a.stop_at_refill else "full"
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, m), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=50, hold_s=1.0, **size)
