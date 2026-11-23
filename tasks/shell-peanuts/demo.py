"""Scripted full-score demo. For each peanut: the left hand picks it up by its back half and holds it out over the
bowl; the right hand pinches the front half and bends it sideways until the shell cracks at the waist, then draws
it back; the left hand tilts and shakes its half so the kernel slides out into the bowl and lays the empty half on
the saucer; the right hand turns its half open end down over the bowl, shakes the kernel out and lays the half on
the saucer.

    python demo.py task.py OUT [--hires] [--no-record] [--whole | --shells-in-bowl]

``--whole`` puts both peanuts in the bowl uncracked (0). ``--shells-in-bowl`` shells them over the bowl but drops
the empty halves in the bowl too (capped at 0.5). Writes one MP4 per camera, setup.png and result.json to OUT.
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
PITCH = math.radians(20)
# the right hand on the front half: pointing +y and 20 deg down, its fingers closing across the half (along x)
SIDE = turn([0, 0, 1], math.pi / 2) @ turn([0, 1, 0], PITCH) @ INSERT @ turn([0, 0, 1], math.pi / 2)
PICK_AT = 0.011            # the left hand's pinch, from the waist along the back half
HOLD = 0.12                # the left hand's grip on the 15 mm shell
RHOLD = 0.10               # the right hand's
GRAB_AT = 0.010            # the right hand's pinch centre, from the waist along the front half
GRAB_DEPTH = 0.010         # that point this far along the pads from the tool point
BEND = 0.006               # how far the right hand pushes the front half's end sideways to crack the pod
LEFT_TILT = 18.0           # deg the left hand tilts its half, open end down (about as far as its wrist reaches here)
WORK = np.array([g.BOWL_XY[0], g.BOWL_XY[1] + 0.012, TOP + 0.085])   # where the waist is held, over the bowl
OVER_BOWL = np.array([g.BOWL_XY[0], g.BOWL_XY[1] - 0.005, TOP + 0.095])
# where each empty half's middle is laid, from the saucer's centre: apart, so a half laid later doesn't knock an
# earlier one off (the left hand's halves lie along y, on the saucer's left side; the right hand's on its right)
SPOTS = {"p1_b": (0.020, 0.013), "p2_b": (-0.020, 0.013), "p1_f": (0.014, -0.016), "p2_f": (-0.014, -0.016)}


def tcp_pose(ctx, side):
    from yam_arena.constants import TCP_OFFSET

    base = ctx.emb._env.unwrapped
    robot = base.scene[f"{side}_robot"]
    names = list(robot.body_names)
    p = ctx.task._np(robot.data.body_link_pose_w)[0, names.index("gripper")]
    pos = p[:3] - ctx.task._np(base.scene.env_origins[0])
    R = ctx.task._rot(p[3:7])
    return pos + R @ np.asarray(TCP_OFFSET), R


def sag(ctx, side):
    """How far the hand's measured tool point falls short of its command (add it to the command)."""
    arm = ctx.left if side == "left" else ctx.right
    return arm.pos - tcp_pose(ctx, side)[0]


def pose(ctx, name):
    return ctx.task.pose(ctx.emb._env, name)


def nut_in(ctx, pod, h):
    """Whether the half's kernel is still inside it (the demo watches, as a person does)."""
    lp, lR = pose(ctx, f"{pod}_{h}")
    kp, _ = pose(ctx, f"{pod}_k{h}")
    q = lR.T @ (kp - lp)
    return bool(-0.002 < q[0] < g.LOBE_L + g.POD_R and math.hypot(q[1], q[2]) < g.POD_R)


def pick(ctx, pod):
    """Left: pinch the pod across its back half and lift it."""
    L = ctx.left
    p, R = pose(ctx, f"{pod}_b")
    at = p + R @ np.array([PICK_AT, 0.0, 0.0])
    at[2] = TOP + 0.0025 + 0.013                      # pad tips 2.5 mm off the bench
    yield from L.move(at + [0, 0, 0.08], DOWN_X, 2.0, grip=0.5)
    yield from L.move(at, DOWN_X, 1.2)
    yield from L.set_grip(HOLD, 0.8)
    yield from wait(0.3)
    yield from L.move(at + [0, 0, 0.04], DOWN_X, 1.0)


def present(ctx, pod):
    """Left: hold the pod's waist out over the bowl."""
    L = ctx.left
    p, _ = pose(ctx, f"{pod}_b")
    yield from L.move(L.pos + (WORK - p), None, 1.5)
    yield from wait(0.4)


def crack(ctx, pod):
    """Right: pinch the front half and push its far end sideways until the shell gives at the waist; draw it back."""
    R = ctx.right
    fp, fR = pose(ctx, f"{pod}_f")
    held = fp + fR @ np.array([GRAB_AT, 0.0, 0.0])
    z = SIDE[:, 2]
    grab = held - z * GRAB_DEPTH
    yield from R.move(grab - z * 0.07, SIDE, 1.8, grip=0.4)
    yield from R.move(grab - z * 0.025, SIDE, 0.7)
    for back, secs in ((0.01, 0.5), (0.0, 0.5), (0.0, 0.3)):
        yield from wait(0.1)
        yield from R.move(grab - z * back + sag(ctx, "right"), SIDE, secs)
    yield from R.set_grip(RHOLD, 0.6)
    yield from wait(0.3)
    start = R.pos.copy()
    for k in range(1, 31):
        R.aim(start + np.array([BEND * k / 30, 0, 0]), SIDE)
        yield
    yield from wait(0.3)
    yield from R.move(R.pos + [0, -0.04, 0.0], None, 0.8)


def lay_on_saucer(ctx, side, name, into_bowl=False):
    """Carry the held half so its middle is over its spot on the saucer (or the bowl's centre), lower it to just
    above the floor and let go."""
    A = ctx.left if side == "left" else ctx.right
    yield from A.move(A.pos + [0, 0, 0.04], None, 0.6)
    lp, lR = pose(ctx, name)
    centre = lp + lR @ np.array([g.LOBE_L / 2, 0, 0])
    tcp, _ = tcp_pose(ctx, side)
    off = A.pos - tcp
    target = np.array(g.BOWL_XY) if into_bowl else np.array(g.DISH_XY) + SPOTS[name]
    goal_xy = target + (tcp - centre)[:2]
    yield from A.move(np.array([goal_xy[0] + off[0], goal_xy[1] + off[1], A.pos[2]]), None, 1.4)
    lp, lR = pose(ctx, name)
    low = min(float((lp + lR @ np.array([x, 0, 0]))[2]) for x in (0.0, g.LOBE_L + g.POD_R)) - g.POD_R
    floor = TOP + (g.BOWL_BASE if into_bowl else g.DISH_BASE)
    yield from A.move(A.pos - [0, 0, low - (floor + 0.006)], None, 0.8)
    yield from A.set_grip(0.35, 0.4)
    yield from wait(0.4)
    yield from A.move(A.pos + [0, 0, 0.06], None, 0.8)


def empty_left(ctx, pod, into_bowl=False):
    """Left: tilt the back half open end down over the bowl and shake the kernel out; lay the half on the saucer."""
    L = ctx.left
    rot = turn([1, 0, 0], math.radians(LEFT_TILT)) @ DOWN_X
    yield from L.move(L.pos + [0, 0.03, -0.01], rot, 1.0)
    yield from wait(0.3)
    for k in range(15):                               # shake till the nut is out: three shakes, then more if needed
        if k >= 3 and not nut_in(ctx, pod, "b"):
            break
        a = 0.006 if k < 3 else 0.009
        yield from L.move(L.pos + [0, a, a], None, 0.12)
        yield from L.move(L.pos - [0, a, a], None, 0.12)
    yield from wait(0.6)
    yield from L.move(L.pos + [0, 0, 0.02], DOWN_X, 0.8)
    yield from lay_on_saucer(ctx, "left", f"{pod}_b", into_bowl)
    yield from L.go_home(1.5)


def empty_right(ctx, pod, into_bowl=False):
    """Right: turn the front half open end down over the bowl, shake the kernel out; lay the half on the saucer."""
    R = ctx.right
    down = turn([1, 0, 0], -math.radians(90) + PITCH) @ SIDE
    yield from R.move(OVER_BOWL, down, 1.5)
    yield from wait(0.3)
    for k in range(12):                               # shake till the nut is out: twice, then more if needed
        if k >= 2 and not nut_in(ctx, pod, "f"):
            break
        yield from R.move(R.pos + [0, 0, 0.01], None, 0.15)
        yield from R.move(R.pos - [0, 0, 0.01], None, 0.15)
    yield from wait(0.5)
    yield from lay_on_saucer(ctx, "right", f"{pod}_f", into_bowl)
    yield from R.go_home(1.5)


def whole(ctx, pod):
    """Left: put the uncracked pod in the bowl."""
    L = ctx.left
    yield from pick(ctx, pod)
    yield from present(ctx, pod)
    yield from L.move(L.pos - [0, 0, 0.03], None, 0.8)
    yield from L.set_grip(0.35, 0.4)
    yield from wait(0.3)
    yield from L.go_home(1.5)


def script(ctx, mode):
    for pod in g.PODS:
        if mode == "whole":
            yield from whole(ctx, pod)
            continue
        yield from pick(ctx, pod)
        yield from present(ctx, pod)
        yield from crack(ctx, pod)
        yield from empty_left(ctx, pod, mode == "shells-in-bowl")
        yield from empty_right(ctx, pod, mode == "shells-in-bowl")
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    if "kernels" not in S:
        return ""
    k = "".join("B" if v else "." for v in S["kernels"].values())
    sh = "".join({"dish": "D", "bowl": "B", None: "."}[v] for v in S["shells"].values())
    c = "".join("X" if v else "." for v in S["cracked"].values())
    return (f"cracked {c} kernels {k} shells {sh} clear {S['clear']*1000:5.1f} drift {S['drift']*1000:5.1f} "
            f"bowl moved {S['bowl_moved']*1000:4.1f} | ik L {ctx.left.worst_mm:.1f} R {ctx.right.worst_mm:.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704")
    ap.add_argument("--log-every", type=int, default=50)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--whole", action="store_true", help="put both peanuts in the bowl uncracked (0)")
    mode.add_argument("--shells-in-bowl", action="store_true", help="drop the empty halves in the bowl (<= 0.5)")
    a = ap.parse_args()
    m = "whole" if a.whole else "shells-in-bowl" if a.shells_in_bowl else "full"
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, m), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=a.log_every, **size)
