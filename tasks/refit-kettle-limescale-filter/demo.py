"""Scripted full-score demo: the right arm takes the kettle's lid off and holds it aside while the left arm puts the
limescale filter back in; then the right arm puts the lid back on.

    python demo.py task.py OUT [--max-steps 1400] [--no-record] [--hires] [--drop | --no-correct | --hold | --no-lid]

0. (Right arm, while the left does step 1.) Grip the lid's knob, the hand tilted 35 degrees toward the kettle, lift
   the lid straight off and hold it up beside the kettle, over its power base.
1. (Left arm.) Pinch the filter's grip tab where it lies (hand tilted 45 degrees back toward the robot, fingers closing across
   the tab), lift it, and stand the hand up as it rises (tilted back, the arm reaches only about 4 cm over the bench).
2. Carry it over the kettle, turning the hand about the tab's normal until it is tilted 45 degrees forward: the
   filter then hangs upright, curve outward, the hand's housing (wider than the lid opening) stays above the rim and
   the forearm passes over the open lid. Correct the hand by the filter's measured pose (the wrist is soft) until the
   filter is within 0.5 mm and 1.5 degrees of square over its slot, 8 mm above the rim.
3. Lower it into the slot until its bottom edge is 3 mm down the guides, correcting x/y on the way, and let go: it
   slides down the slot to its stop. Back the hand out. (If it has not reached the stop, press its grip tab down with
   the closed fingertips.) Back away and go home.
4. (Right arm.) Carry the lid back over the opening, correcting by where it really hangs in the hand, lower it until
   its plug drops into the opening, let go and go home.

Other routes, for the README (see its table for what each scores):
  --drop        let go of it upright 25 mm over the lid opening, 4 mm off its slot
  --no-correct  the same moves without correcting by the filter's measured pose (open loop on its starting pose)
  --hold        put it in, but keep holding its grip tab at the end
  --no-lid      put the filter in, but set the lid down on the bench instead of back on the kettle
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, turn, wait  # noqa: E402

TOP = g.TOP
PAD_TIP = 0.013
TILT = math.radians(45.0)
PRESS_TILT = 25.0                           # deg forward of straight down, for a press (the forearm clears the lid)
PINCH = np.array([-0.012, 0.0, 0.050])      # pinch point on the grip tab (filter frame): 12 mm in from the plate
HOVER = 0.008                               # filter bottom this far over the lips' top before lowering it in
SLIDE_TO = 0.044                            # hold it until its bottom edge is 6 mm down the guides, then let go
TAB_TOP = np.array([-g.FIN_L / 2, 0.0, g.FIN_Z[1]])   # middle of the grip tab's top edge (filter frame)
LID_TILT = math.radians(35.0)               # right hand tilted toward the kettle to reach the knob (262 mm up)
KNOB_PINCH = g.LID_T + g.KNOB_H - 0.009     # pinch the knob 9 mm below its top (lid frame z)
LID_HOLD = np.array([0.330, -0.210, TOP + 0.30])   # where the right hand holds the lid: up over the power base
LID_BENCH = np.array([0.420, -0.080])       # --no-lid: where the lid is set down on the bench


def rots():
    from yam_arena.demo import grip_rotation

    down = grip_rotation(0.0)                            # fingers close along y, across the tab
    grasp = turn([0, 1, 0], TILT) @ down                 # approach down and back toward the robot
    carry = turn([0, 1, 0], -TILT) @ down                # approach down and forward: the filter hangs upright
    return grasp, carry


def fpose(ctx):
    s = ctx.states()["filter"]
    return np.asarray(s.pos, float), s.rotation()


def seat(ctx):
    """Where the filter's origin goes: on its stop, in the middle of the slot's radial play; and the kettle's
    rotation (the filter's seated orientation)."""
    s = ctx.states()["kettle"]
    kR = s.rotation()
    x, y, z = g.seat_origin_k()
    return np.asarray(s.pos, float) + kR @ np.array([x - g.SLOT_PLAY / 2, y, z]), kR


def actual_tcp(ctx, side):
    """The arm's real tool point (the commanded one leads it: the wrist is soft)."""
    from yam_arena.constants import TCP_OFFSET

    base = ctx.emb._env.unwrapped
    robot = base.scene[f"{side}_robot"]
    i = list(robot.body_names).index("gripper")
    pose = robot.data.body_link_pose_w
    pose = getattr(pose, "torch", pose)[0, i].detach().cpu().numpy()
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
    x, y, z, w = pose[3:7]
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return pose[:3] - origin + R @ np.asarray(TCP_OFFSET), R


def orth(R):
    u, _, vt = np.linalg.svd(R)
    return u @ vt


def angle(Ra, Rb):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1) / 2))))


def lid_state(ctx):
    s = ctx.states()["lid"]
    return np.asarray(s.pos, float), s.rotation()


def lid_grip(base, knob):
    """Straight down turned 35 degrees about the horizontal across the line from the arm's base to the knob."""
    from yam_arena.demo import grip_rotation

    d = np.asarray(knob[:2], float) - np.asarray(base[:2], float)
    yaw = math.atan2(d[1], d[0])
    return turn([0, 0, 1], yaw) @ turn([0, 1, 0], -LID_TILT) @ turn([0, 0, 1], -yaw) @ grip_rotation(yaw)


def right_lid_off(ctx):
    """Grip the knob, lift the lid straight off and hold it up beside the kettle."""
    arm = ctx.right
    lp, lR = lid_state(ctx)
    knob = lp + lR @ np.array([0.0, 0.0, KNOB_PINCH])
    rot = lid_grip(arm.base, knob)
    a = rot[:, 2]
    yield from arm.move(knob - a * 0.07 + [0, 0, 0.03], rot, 1.6, grip=1.0)
    yield from arm.move(knob - a * 0.02, rot, 0.8)
    yield from arm.move(knob, rot, 0.6)
    yield from wait(0.2)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.2)
    yield from arm.move(arm.pos + [0, 0, 0.06], rot, 1.0)
    yield from arm.move(LID_HOLD, rot, 1.5)
    ctx.flags["lid_rot"] = rot
    lp, lR = lid_state(ctx)
    tilt = math.degrees(math.acos(min(1.0, float(lR[2, 2]))))
    print(f"DEMO lid off: lid at {np.round((lp - [0, 0, TOP]) * 1000)} mm, tilt {tilt:.1f}", flush=True)


def right_lid_on(ctx, bench=False):
    """Carry the lid back over the opening (or, ``bench``, onto the bench), lower it, let go and go home."""
    arm = ctx.right
    rot = ctx.flags["lid_rot"]
    a = rot[:, 2]
    if bench:
        goal = np.r_[LID_BENCH, TOP + 0.0005]
    else:
        s = ctx.states()["kettle"]
        goal = np.asarray(s.pos, float) + s.rotation() @ np.asarray(g.lid_on_k())
    yield from arm.move(np.r_[goal[:2] - (lid_state(ctx)[0] - arm.pos)[:2], arm.pos[2]], rot, 1.5)
    for _ in range(4):
        lp, _ = lid_state(ctx)
        off = lp - arm.pos
        yield from arm.move(goal + [0, 0, 0.020] - off, rot, 0.8)
        yield from wait(0.3)
        lp, lR = lid_state(ctx)
        err = lp - (goal + [0, 0, 0.020])
        tilt = math.degrees(math.acos(min(1.0, float(lR[2, 2]))))
        print(f"DEMO lid over goal: err mm {np.round(err * 1000, 1)}, tilt {tilt:.1f}", flush=True)
        if np.linalg.norm(err[:2]) < 0.0015:
            break
    off = lid_state(ctx)[0] - arm.pos
    yield from arm.move(goal + [0, 0, 0.006] - off, rot, 0.8)
    # down slowly until the lid stops (its disc on the top), then let go
    z_cmd = arm.pos[2]
    for k in range(60):
        lp, lR = lid_state(ctx)
        if lp[2] - goal[2] < 0.0015:
            break
        z_cmd -= 0.0005
        arm.aim(np.r_[arm.pos[:2], z_cmd], rot)
        yield
    yield from wait(0.2)
    lp, lR = lid_state(ctx)
    tilt = math.degrees(math.acos(min(1.0, float(lR[2, 2]))))
    print(f"DEMO lid down after {k} steps: {np.round((lp - goal) * 1000, 1)} mm off, tilt {tilt:.1f}", flush=True)
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(arm.pos - a * 0.05 + [0, 0, 0.02], rot, 0.8)
    S = ctx.task._STATE
    print(f"DEMO lid let go: on {S['lid_on']}, err mm {np.round(np.asarray(S['lid_err']) * 1000, 1)}, "
          f"tilt {S['lid_tilt']:.1f}", flush=True)
    yield from arm.go_home(2.0)


def left_pick(ctx):
    grasp, carry = rots()
    arm = ctx.left
    # 1. pinch the grip tab
    p, R = fpose(ctx)
    a = grasp[:, 2]
    tcp = p + R @ PINCH - a * 0.006                          # pad tips 7 mm past the pinch point, 6 mm over the plate
    down = turn([0, 1, 0], -TILT) @ grasp
    yield from arm.move(np.r_[tcp[:2], TOP + 0.10], down, 1.6, grip=0.45)
    yield from arm.move(tcp - a * 0.015 + [0, 0, 0.015], grasp, 1.2)
    yield from arm.move(tcp, grasp, 0.6)
    yield from wait(0.2)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.2)
    yield from arm.move(arm.pos + [0, 0, 0.025], grasp, 0.6)
    yield from arm.move(arm.pos + [0, 0, 0.08], down, 1.2)
    print(f"DEMO lifted: filter z {(fpose(ctx)[0][2] - TOP) * 1000:.0f} mm", flush=True)


def left_insert(ctx, mode=None):
    grasp, carry = rots()
    arm = ctx.left
    correct = mode != "no-correct"
    # 2. carry it over the kettle, upright
    s, kR = seat(ctx)
    yield from arm.move(np.r_[arm.pos[:2], TOP + 0.33], carry, 1.5)       # up first, clear of the kettle
    yield from arm.move(np.r_[s[0] - 0.04, s[1] + 0.02, TOP + 0.36], carry, 1.5)
    yield from wait(0.4)
    rot = carry
    hover = HOVER if mode != "drop" else 0.025
    top_z = TOP + g.GUIDE_Z[1]
    if correct:
        for _ in range(8):
            s, kR = seat(ctx)
            p, R = fpose(ctx)
            rot = orth((kR @ R.T) @ rot)                         # turn the hand by the filter's error
            off = p - arm.pos
            goal = np.r_[s[:2], top_z + hover] + (np.array([0.0, 0.004, 0.0]) if mode == "drop" else 0.0)
            yield from arm.move(goal - off, rot, 1.0)
            yield from wait(0.3)
            p, R = fpose(ctx)
            err, ang = p - goal, angle(kR, R)
            print(f"DEMO hover err mm {np.round(err * 1000, 2)}, {ang:.1f} deg off square", flush=True)
            if np.linalg.norm(err[:2]) < 0.0005 and ang < 1.5:
                break
    else:
        # open loop: where the filter would hang if the grip were exactly as planned and the arm went where told
        off = turn([0, 1, 0], -math.pi / 2) @ (-(PINCH - np.array([0.0, 0.0, 0.0])))
        off = carry @ grasp.T @ (fpose(ctx)[1] @ (-PINCH))
        yield from arm.move(np.r_[s[:2], top_z + hover] - off, carry, 1.5)
        yield from wait(0.3)
    if mode == "drop":
        # let go and keep the hand where it is (25 mm over the rim, already more than 4 cm from the falling filter)
        yield from arm.set_grip(0.5, 0.4)
        yield from wait(4.0)
        return
    # 3. lower it into the slot, still holding it, then let go
    off = fpose(ctx)[0] - arm.pos
    z_cmd = arm.pos[2]
    k, dz = 0, 1.0
    for k in range(120):
        s, kR = seat(ctx)
        p, R = fpose(ctx)
        dz = p[2] - s[2]
        if dz < SLIDE_TO:
            break
        exy = (p[:2] - s[:2]) if correct else np.zeros(2)
        z_cmd = max(z_cmd - 0.0010, s[2] + SLIDE_TO - off[2] - 0.004)
        arm.aim(np.r_[arm.pos[:2] - 0.7 * exy, z_cmd], rot)
        if k % 10 == 0:
            print(f"DEMO lower k {k}: dz {dz * 1000:.2f} mm, xy err {np.round((p[:2] - s[:2]) * 1000, 2)}", flush=True)
        yield
    yield from wait(0.3)
    print(f"DEMO lowered: dz {(fpose(ctx)[0][2] - seat(ctx)[0][2]) * 1000:.2f} mm", flush=True)
    if mode == "hold":
        yield from wait(4.0)
        return
    a = rot[:, 2]
    yield from arm.set_grip(0.45, 0.4)
    yield from arm.move(arm.pos - a * 0.03, rot, 0.6)
    yield from wait(0.6)
    S = ctx.task._STATE
    print(f"DEMO released: dz {(fpose(ctx)[0][2] - seat(ctx)[0][2]) * 1000:.2f} mm, in_slot {S['in_slot']}, "
          f"seated {S['seated']}", flush=True)
    if correct and S["in_slot"] and not S["seated"]:
        # it stopped short: press the grip tab down with the closed fingertips, the hand stood up steeper so the
        # forearm clears the open lid
        yield from arm.move(arm.pos + [0.0, 0.0, 0.03], rot, 0.5)
        rot = orth(turn([0, 1, 0], math.radians(45.0 - PRESS_TILT)) @ rot)
        a = rot[:, 2]
        yield from arm.set_grip(0.0, 0.4)
        p, R = fpose(ctx)
        top = p + R @ TAB_TOP
        yield from arm.move(top - a * (PAD_TIP + 0.015), rot, 0.8)
        yield from arm.move(top - a * (PAD_TIP + 0.001), rot, 0.5)
        corr = np.zeros(3)
        for k in range(160):
            s, kR = seat(ctx)
            p, R = fpose(ctx)
            dz = p[2] - s[2]
            if S["seated"] or dz < -0.005 or dz > 0.065 or np.linalg.norm(p[:2] - s[:2]) > 0.006:
                break
            top = p + R @ TAB_TOP
            real, _ = actual_tcp(ctx, "left")
            corr[:2] = np.clip(corr[:2] + 0.5 * (top - (real + a * PAD_TIP))[:2], -0.012, 0.012)
            over = min(0.020, 0.002 + 0.0005 * k)
            arm.aim(top - a * PAD_TIP + corr + np.r_[0.0, 0.0, -over], rot)
            yield
        print(f"DEMO pressed after {k} steps: dz {dz * 1000:.2f} mm, seated {S['seated']}", flush=True)
        yield from wait(0.2)
    # back away up and out, then home
    yield from arm.move(arm.pos - a * 0.04, rot, 0.8)
    yield from arm.move(np.r_[arm.pos[0] - 0.05, arm.pos[1] + 0.08, TOP + 0.345], rot, 1.2)
    yield from arm.move(np.array([0.15, 0.28, TOP + 0.30]), carry, 1.5)   # out to the arm's own side, clear of the kettle
    yield from arm.go_home(2.0)
    yield from wait(1.5)



def script(ctx, mode=None):
    yield from wait(0.3)
    yield from together(right_lid_off(ctx), left_pick(ctx))
    yield from left_insert(ctx, mode)
    if mode in ("drop", "hold"):
        return
    yield from right_lid_on(ctx, bench=mode == "no-lid")
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    e = None if S["err"] is None else np.round(np.asarray(S["err"]) * 1000, 1)
    t = None if S["tilt"] is None else round(S["tilt"], 1)
    d = None if S["hand_dist"] is None else round(S["hand_dist"] * 1000)
    return (f"in_slot {S['in_slot']} seated {S['seated']} clear {S['clear']} still {S['still']} | err mm {e} "
            f"ang {t} hand mm {d} | lid on {S['lid_on']} lid hand mm "
            f"{None if S['lid_dist'] is None else round(S['lid_dist'] * 1000)} | kettle tilt {S['kettle_tilt']:.1f} (max {S['kettle_max_tilt']:.1f}) "
            f"moved {S['kettle_moved'] * 1000:.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hires", action="store_true")
    m = ap.add_mutually_exclusive_group()
    m.add_argument("--drop", dest="mode", action="store_const", const="drop")
    m.add_argument("--no-correct", dest="mode", action="store_const", const="no-correct")
    m.add_argument("--hold", dest="mode", action="store_const", const="hold")
    m.add_argument("--no-lid", dest="mode", action="store_const", const="no-lid")
    a = ap.parse_args()
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, a.mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=20, hold_s=1.0, **size)
