"""Scripted demo: slide coins one at a time out of the holder into the tray.

    python demo.py task.py OUT [--check] [--no-record] [--hires] [--max-steps N]

For each coin the hand closes its fingers and turns them so the closed tips' 15 mm side runs along the tube's exit
(the 8 mm side across it, so they fit even the dime's 14.7 mm slot). It sets the tips down on the top coin at the back
of the lip's opening, commanded 1.5 mm under the lip (the soft wrist leaves them pressing about 0.5 N, under the
stack spring's 0.6 to 0.8 N, so the stack is not pushed down), and draws them along the exit, kept a few millimetres
ahead of the coin's centre as read from the simulator: friction carries the coin under the lip, along the channel and
over the rim, and it drops into the tray. The arm on the tube's side does it.

Default: 40 cents, the only way these stacks make it: the left arm a dime and then a nickel, the right arm a quarter.
--check: one coin from each tube, for testing the exits.
--route overpay: two dimes, a nickel and a quarter (50 cents, scores 0.25).
--route short: a dime and a quarter (35 cents, two of the three coins needed, scores 0.5).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, wait  # noqa: E402

TOP = g.TOP
PRESS = 0.0015           # tips commanded this far under the lip to start with
PRESS_MAX = 0.004
LEAD = 0.005             # tips commanded this far ahead of where they sit on the coin
END_S = 0.024            # push until the coin's centre is this far out from its tube's centre
LOG: list = []
TRACE = False
ONLY = None
ROUTES = {"overpay": ["dime", "dime", "nickel", "quarter"], "short": ["dime", "quarter"]}
ROUTE = None


def holder_frame(ctx):
    h = ctx.states()["holder"]
    return np.asarray(h.pos, float), h.rotation()


def coin(ctx, k):
    return np.asarray(ctx.states()[k].pos, float)


FLIP = {"quarter": False, "dime": False, "nickel": False, "penny": False}   # the other of the two yaws


def push_rot(u, flip=False):
    """Fingertips down, fingers closed and closing across the exit, so the tips' 15 mm side runs along it: of the two
    such yaws (half a turn apart), the one nearer 90 degrees, or the other with ``flip``."""
    from yam_arena.demo import grip_rotation

    v = np.array([-u[1], u[0], 0.0])
    R90 = grip_rotation(math.pi / 2)
    col = int(np.argmax(np.abs(R90[0, :])))             # the column that closes along x at yaw 90 deg
    yaws = np.linspace(-math.pi, math.pi, 1441)[:-1]
    fit = np.array([abs(float(grip_rotation(y)[:, col] @ v)) for y in yaws])
    good = [y for y, f in zip(yaws, fit) if f >= fit.max() - 1e-4]       # two clusters, half a turn apart
    near = min(good, key=lambda y: abs(math.remainder(y - math.pi / 2, 2 * math.pi)))
    far = max(good, key=lambda y: abs(math.remainder(y - near, 2 * math.pi)))
    return grip_rotation(far if flip else near)


def top_coin(ctx, c):
    """Tube c's top coin still in the tube (or None)."""
    left = [k for k in g.coin_ids(c) if ctx.task._STATE["where"][k] == "tube"]
    return max(left, key=lambda k: coin(ctx, k)[2]) if left else None


def arm_for(c):
    return "right" if g.TUBES[c][1] < 0 else "left"


def push(ctx, c):
    """Slide tube c's top coin out over the rim."""
    arm = getattr(ctx, arm_for(c))
    for _ in range(100):                                  # the last coin out has landed
        if all(w in ("tube", "tray", "bench") for w in ctx.task._STATE["where"].values()):
            break
        yield
    k = top_coin(ctx, c)
    hp, hR = holder_frame(ctx)
    d = g.tube_dims(c)
    tube = hp + hR @ np.array([*g.TUBES[c], g.plate_under()])
    u = hR @ np.array([*g.exit_dir(c), 0.0])
    # the tips (15 mm along, 8 mm across) placed so their back corners are 0.5 mm inside the round opening
    s0 = -math.sqrt((d["hole_r"] - 0.0005) ** 2 - 0.004 ** 2) + 0.0075
    z = tube[2] + PAD_TIP - PRESS
    start = tube + u * s0
    rot = push_rot(u, FLIP[c])
    side = 0 if arm is ctx.left else 1

    def tips():
        return ctx.task.hand_points(ctx.emb._env)[side] - np.array([0, 0, PAD_TIP])

    yield from arm.move(np.r_[start[:2], z + 0.045], rot, 2.0, grip=0.0)
    hover = np.r_[start[:2], z + PRESS + 0.003]
    yield from arm.move(hover, rot, 0.8)
    for _ in range(4):                                   # correct by where the tips really are (the wrist sags)
        yield from wait(0.25)
        err = (start - tips())[:2]
        if np.linalg.norm(err) < 0.0004:
            break
        hover = hover + np.r_[err, 0.0]
        yield from arm.move(hover, rot, 0.3)
    off = hover[:2] - start[:2]                          # command offset that puts the tips where they are wanted
    press = PRESS
    yield from arm.move(np.r_[start[:2] + off, z], rot, 0.6)
    yield from wait(0.3)
    t0 = tips()
    rel0 = float((t0 - tube) @ u) - float((coin(ctx, k) - tube) @ u)   # tips' place on the coin, along the exit
    print(f"DEMO {k}: tips at s {float((t0 - tube) @ u) * 1000:+.1f} (want {s0 * 1000:+.1f}) w "
          f"{float((t0 - tube) @ np.array([-u[1], u[0], 0])) * 1000:+.1f} z {(t0[2] - tube[2]) * 1000:+.2f} mm from the "
          f"lip; coin top {(coin(ctx, k)[2] + g.COINS[c][1] / 2 - tube[2]) * 1000:+.2f}", flush=True)
    s, n, stuck, regrips = s0, 0, 0, 0
    while n < 400:
        p = coin(ctx, k)
        cs = float((p - tube) @ u)
        if cs >= END_S:
            break
        sunk = tube[2] - (p[2] + g.COINS[c][1] / 2)     # the coin's top under the lip: pressed too hard
        ahead = float((tips() - tube) @ u) - cs - rel0   # the tips slipping forward over the coin
        if sunk > 0.0003:
            press = max(0.0, press - 0.00005)
        elif ahead > 0.001 and cs < g.tube_dims(c)["bore_r"]:
            press = min(PRESS_MAX, press + 0.00005)
        stuck = stuck + 1 if (ahead > 0.002 and cs < g.tube_dims(c)["bore_r"]) else 0
        if stuck >= 10 and regrips < 4:                  # slipped off its place: lift, set back down, press harder
            regrips += 1
            stuck = 0
            lift = arm.pos + [0, 0, 0.001]
            yield from arm.move(lift, rot, 0.3)
            back = tube + u * (cs + rel0)
            yield from arm.move(np.r_[back[:2] + off, lift[2]], rot, 0.4)
            press = min(PRESS_MAX, press + 0.0003)
            yield from arm.move(np.r_[back[:2] + off, tube[2] + PAD_TIP - press], rot, 0.4)
            s = cs + rel0
            print(f"DEMO {k}: re-grip {regrips} at coin s {cs * 1000:+.1f}, press {press * 1000:.2f} mm", flush=True)
            continue
        if cs > g.tube_dims(c)["bore_r"]:                # past its bore and tipping over the rim: push it off
            s += 0.0005
        elif ahead < 0.002:                              # advance only while the tips keep their place on the coin
            s = min(s + 0.0005, cs + rel0 + LEAD)
        arm.aim(np.r_[(tube + u * s)[:2] + off, tube[2] + PAD_TIP - press], rot)
        n += 1
        yield
        if TRACE and n % 3 == 0:
            Rc = ctx.states()[k].rotation()
            t = tips()
            print(f"TRACE {k} n {n}: coin s {cs * 1000:+.1f} sunk {sunk * 1000:+.2f} tilt "
                  f"{math.degrees(math.acos(min(1.0, abs(float(Rc[2, 2]))))):.0f} | tips s "
                  f"{float((t - tube) @ u) * 1000:+.1f} ahead {ahead * 1000:+.1f} z {(t[2] - tube[2]) * 1000:+.2f} "
                  f"press {press * 1000:.2f}", flush=True)
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.04], rot, 0.6)
    yield from arm.go_home(1.5)
    st = ctx.task._STATE
    line = (f"DEMO {k} ({c}): pushed {n} steps, coin out {float((coin(ctx, k) - tube) @ u) * 1000:.1f} mm, now in "
            f"{st['where'][k]}; tray {st['cents']} cents; holder tilt {st['tilt']:.2f} moved {st['moved'] * 1000:.2f} mm")
    LOG.append(line)
    print(line, flush=True)


def script(ctx, check=False):
    yield from wait(1.0)
    plan = ["quarter", "dime", "nickel", "penny"] if check else ["dime", "nickel", "quarter"]
    if ROUTE:
        plan = ROUTES[ROUTE]
    if ONLY:
        plan = ONLY
    for c in plan:
        yield from push(ctx, c)
    for side in ("left", "right"):
        yield from getattr(ctx, side).go_home(2.0)
    yield from wait(1.5)
    print("DEMO SUMMARY", flush=True)
    for line in LOG:
        print(line, flush=True)
    st = ctx.task._STATE
    print(f"DEMO end: tray {st['cents']} cents, spilled {st['spilled']}, where "
          f"{ {k: w for k, w in st['where'].items() if w != 'tube'} }, holder max tilt {st['max_tilt']:.2f}", flush=True)


def log(ctx, step, info) -> str:
    st = ctx.task._STATE
    out = {k: w for k, w in st["where"].items() if w != "tube"}
    return (f"out {out} tray {st['cents']}c spilled {st['spilled']} clear {st['clear']} still {st['still']} "
            f"holder tilt {st['tilt']:.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704")
    ap.add_argument("--check", action="store_true", help="one coin from each tube")
    ap.add_argument("--only", default="", help="comma-separated tubes to push, traced")
    ap.add_argument("--route", choices=sorted(ROUTES), help="a wrong strategy, for the failing demos")
    ap.add_argument("--flip", default="", help="comma-separated tubes to push with the other yaw")
    a = ap.parse_args()
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    if a.only:
        globals()["ONLY"], globals()["TRACE"] = a.only.split(","), True
    globals()["ROUTE"] = a.route
    for c in filter(None, a.flip.split(",")):
        FLIP[c] = True
    run(a.task, a.out, lambda ctx: script(ctx, check=a.check), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=40, hold_s=1.0, **size)
