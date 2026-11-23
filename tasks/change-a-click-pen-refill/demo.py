"""Scripted full-score demo of changing the click pen's refill, and two shortcut variants.

1. Spring: the left hand picks the spring up from where it stands on end, holds it 3 mm over the refill's point and
   lets it go; it drops over the point onto the refill's front section.
2. Front barrel: the right hand lifts the front barrel, nose up, out of the stand, holds its open end over the
   refill's point and lowers it over refill and spring, letting go 45 mm down so it settles on the spring.
3. Carry: the right hand takes the front barrel again and the left hand pinches the refill where it shows below the
   barrel; together they lift refill, spring and barrel out of the stand and hold the refill's end over the back half.
4. Join: they lower the refill into the back half; the left hand lets go, and the right hand pushes the front barrel
   into the back half's sleeve, lets go and both hands go back to their ready poses.

The demo reads the parts' and hands' poses from the simulator to steer (as a person watches the pen); the arms are
moved only by joint commands through the standard 14-value interface. No part is turned in the hand.

    python demo.py task.py OUT [--hires] [--no-record] [--no-spring | --stop-at-barrel]

--no-spring: steps 2-4 without the spring (the pen is put together without it: 0).
--stop-at-barrel: steps 1-2 only (spring on, front barrel over it, never joined to the back half: 0.5).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import CONTROL_HZ, PAD_TIP, run, turn, wait  # noqa: E402

TOP = g.TOP
DOWN_Y = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
DOWN_X = turn([0, 0, 1], math.pi / 2) @ DOWN_Y                               # pointing down, fingers close along x
L_SIDE = turn([1, 0, 0], -math.pi / 2) @ DOWN_X @ turn([0, 0, 1], math.pi)   # left pointing -y, fingers along x


def rehome(arm):
    """Back at the ready pose: restart the arm's IK from the ready joint angles, so later moves don't depend on how
    the arm got there."""
    from yam_arena.constants import READY

    arm.q = np.array(READY, float)


def pose(ctx, n):
    return ctx.task.pose(ctx.emb._env, n)


def point_and_up(ctx):
    rp, rR = pose(ctx, "refill")
    return rp, -rR[:, 0]                       # the point, and the refill's axis pointing up out of it


def mouth(ctx):
    bp, bR = pose(ctx, "barrel")
    return bp + bR[:, 0] * g.BARREL_L


def tcp(ctx, side):
    return ctx.task.tool_points(ctx.emb._env)[0 if side == "left" else 1]


def both(ctx, d, seconds):
    """Move both hands by the same offset together."""
    from yam_arena import ik

    L, R = ctx.left, ctx.right
    pL, pR, d = L.pos.copy(), R.pos.copy(), np.asarray(d, float)
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        L.aim(pL + s * d, L.rot)
        R.aim(pR + s * d, R.rot)
        yield


def status(ctx, what):
    s, t = ctx.task._STATE, ctx.task
    print(f"   {what}: spring {s['spring_along']*1000:5.1f}/{s['spring_off']*1000:4.1f} on {t.spring_on()} | nose "
          f"{s['nose_along']*1000:6.1f}/{s['nose_off']*1000:4.1f} over {t.barrel_over()} | mouth "
          f"{s['mouth_along']*1000:6.1f}/{s['mouth_off']*1000:4.1f} axis {s['back_axis']:4.1f} joined {t.joined()} | "
          f"ik L {ctx.left.worst_mm:.1f} R {ctx.right.worst_mm:.1f}", flush=True)


def spring_on(ctx):
    L = ctx.left
    sp, _ = pose(ctx, "spring")
    at = np.array([sp[0], sp[1], TOP + 0.003 + PAD_TIP])          # pad tips 3 mm off the table
    yield from L.move(at + [0, 0, 0.08], DOWN_X, 2.0, grip=0.4)
    yield from L.move(at, DOWN_X, 1.0)
    yield from L.set_grip(0.02, 0.8)
    yield from wait(0.3)
    yield from L.move(at + [0, 0, 0.14], DOWN_X, 1.2)
    for k in range(5):                    # the spring's bottom centre 3 mm above the point, on the refill's axis
        tip, up = point_and_up(ctx)
        sp, sR = pose(ctx, "spring")
        bottom = sp - sR[:, 0] * (g.SPRING_L / 2) * np.sign(sR[2, 0])
        err = (tip + up * 0.003) - bottom
        yield from L.move(L.pos + err, None, 1.5 if k == 0 else 0.5)
        yield from wait(0.25)
        if np.linalg.norm(err) < 0.0003:
            break
    yield from L.set_grip(0.3, 0.4)
    yield from wait(1.0)
    yield from L.move(L.pos + [0, 0, 0.04], None, 0.8)
    yield from L.go_home(1.5)
    rehome(L)
    status(ctx, "1 spring dropped")


def barrel_on(ctx):
    R = ctx.right
    bp, _ = pose(ctx, "barrel")                                   # bp: the nose tip, on top
    at = np.array([bp[0], bp[1], bp[2] - 0.025 + PAD_TIP])        # pads 10 to 25 mm below the nose tip
    yield from R.move(at + [0, 0, 0.06], DOWN_X, 1.5, grip=0.4)
    yield from R.move(at, DOWN_X, 1.0)
    yield from R.set_grip(0.04, 0.8)
    yield from wait(0.3)
    yield from R.move(at + [0, 0, 0.135], DOWN_X, 1.5)           # the open end clear above the refill's point
    for gap, secs in ((0.012, 1.5), (0.006, 0.8), (0.002, 0.6), (-0.010, 1.0), (-0.030, 1.2), (-0.045, 1.2)):
        for k in range(2 if gap > 0 else 1):
            tip, up = point_and_up(ctx)
            target = tip + up * gap                               # the barrel's open end this far above the point
            yield from R.move(R.pos + (target - mouth(ctx)), None, secs if k == 0 else 0.4)
            yield from wait(0.2)
    yield from R.set_grip(0.3, 0.4)
    yield from wait(1.0)
    yield from R.move(R.pos + [0, 0, 0.05], None, 0.8)
    yield from wait(1.0)
    status(ctx, "2 barrel let go")


def carry(ctx):
    L, R = ctx.left, ctx.right
    bp, _ = pose(ctx, "barrel")
    at = np.array([bp[0], bp[1], bp[2] - 0.025 + PAD_TIP])
    yield from R.move(at + [0, 0, 0.06], DOWN_X, 1.5, grip=0.4)
    yield from R.move(at, DOWN_X, 1.0)
    yield from R.set_grip(0.04, 0.6)
    rp, rR = pose(ctx, "refill")
    z = mouth(ctx)[2] - 0.012                                  # the refill between the stand and the barrel's end
    axis_pt = rp + rR[:, 0] * ((rp[2] - z) / max(1e-6, -rR[2, 0]))
    grab = np.array([axis_pt[0], axis_pt[1], z])
    yield from L.move(grab + [0, 0.07, 0.03], L_SIDE, 2.0, grip=0.5)
    yield from L.move(grab + [0, 0.05, 0.0], L_SIDE, 1.0)
    for k in range(3):                                         # by where the pads really are
        yield from wait(0.2)
        err = grab - tcp(ctx, "left")
        yield from L.move(L.pos + [err[0], 0, err[2]] + ([0, -0.05, 0] if k == 0 else [0, 0, 0]), None,
                          1.0 if k == 0 else 0.3)
    yield from L.set_grip(0.0, 0.6)
    yield from wait(0.3)
    yield from both(ctx, [0, 0, 0.085], 2.0)
    kp, kR = pose(ctx, "back")
    top = kp + kR[:, 0] * g.BACK_L
    for k in range(3):                                         # the refill's end over the back half's open end
        rp, rR = pose(ctx, "refill")
        d = top - (rp + rR[:, 0] * g.REFILL_L)
        yield from both(ctx, [d[0], d[1], 0.0], 2.0 if k == 0 else 0.6)
        yield from wait(0.3)
    yield from both(ctx, [0, 0, (top[2] + 0.014) - mouth(ctx)[2]], 2.0)   # the refill 4-5 cm into the back half
    status(ctx, "3 carried and lowered")


def join(ctx):
    L, R = ctx.left, ctx.right
    yield from L.set_grip(0.5, 0.5)
    yield from L.move(L.pos + [0, 0.06, 0.02], None, 1.0)
    yield from L.go_home(1.5)
    for k, extra in enumerate((0.004, 0.0, -0.002)):
        kp, kR = pose(ctx, "back")
        seat = kp + kR[:, 0] * (g.BACK_L - g.SLEEVE_L)
        yield from R.move(R.pos + (seat + [0, 0, extra]) - mouth(ctx), None, 1.2 if k < 2 else 0.6)
        yield from wait(0.3)
    yield from R.set_grip(0.4, 0.5)
    yield from R.move(R.pos + [0, 0, 0.05], None, 0.8)
    yield from R.go_home(1.5)
    status(ctx, "4 joined, hands away")


def script(ctx, mode):
    yield from wait(0.5)
    if mode != "no-spring":
        yield from spring_on(ctx)
    yield from barrel_on(ctx)
    if mode != "stop-at-barrel":
        yield from carry(ctx)
        yield from join(ctx)
    for arm in (ctx.left, ctx.right):
        yield from arm.go_home(1.0)
    yield from wait(1.5)
    status(ctx, "end")


def log(ctx, step, info):
    s = ctx.task._STATE
    if not s:
        return ""
    return (f"spring {s['spring_along']*1000:5.1f}/{s['spring_off']*1000:4.1f} | nose {s['nose_along']*1000:6.1f} | "
            f"mouth {s['mouth_along']*1000:6.1f}/{s['mouth_off']*1000:4.1f} | clear {s['clear']*1000:5.1f} still "
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
    mode.add_argument("--stop-at-barrel", action="store_true", help="stop after the front barrel is on (0.5)")
    a = ap.parse_args()
    m = "no-spring" if a.no_spring else "stop-at-barrel" if a.stop_at_barrel else "full"
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, m), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=100, hold_s=1.0, **size)
