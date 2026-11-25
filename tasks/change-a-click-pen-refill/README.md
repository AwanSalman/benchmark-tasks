# Change a click pen's refill: spring into the nose, new refill in through it, clicker back in with its hook to the clip

A one-piece click ballpoint of the Clip Click type (clear barrel, a clicker that latches into the barrel's back end
under the pocket clip) lies taken apart in a desk pen stand, ready for its new refill. The barrel stands nose-down in
the middle hole, the clicker button-up in one side hole and the new refill point-down in the other. The pen's loose
spring stands on end on the table beside the stand. The instruction is:
**"Put the new refill in the pen: the spring into the barrel's nose, the refill in point-first through it, then push
the clicker in until it latches, its hook toward the clip."**

Changing a pen's refill is a familiar desk job, and on this kind of pen the order matters:
- **The spring** goes in first: a 3 cm coil dropped down the barrel's 6.5 mm bore so that it lands in the nose. Let go
  crooked or off-centre, it jams in the opening or falls beside the barrel.
- **The refill** goes in point-first and has to thread its 2 mm point through the spring, 10 cm down the bore, so that
  the spring ends up round its front against its stop. Off-centre, it slides down beside the spring instead.
- **The clicker** goes in over the refill's end, with its hook toward the clip: that is where the barrel's notches are,
  and it latches only there.

A pen put together without its spring, or with the refill beside it, looks finished but doesn't click.

Board neighbours: the four-colour pen slider (#2987) pushes a slider into a catch; cap-and-sort pens (#360) and a
lancet into a lancing device (#2259) put one part into or onto another. None assembles three loose parts into one
barrel in a set order, with the last one turned to latch.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the pen's parts and the stand, the monitor, `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the scoring thresholds |
| `build_assets.py` | Writes `assets/*.usda` (needs `pxr`) |
| `assets/` | The barrel, clicker, refill, spring and stand |
| `scenekit.py` | USD bodies for yam_arena task files (from `child-resistant-cap`; adds `angular_damping`) |
| `demokit.py` | Helpers for the scripted demo (from `child-resistant-cap`) |
| `demo.py` | The scripted full-score demo and two shortcut variants |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, Python 3.12) on an NVIDIA L40S.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the new refill in the pen: the spring into the barrel's nose, the refill in point-first through it, then push the clicker in until it latches, its hook toward the clip." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pen-refill-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 1,582 control steps (79 s of simulation); Isaac Sim takes about a minute
  to start.

## Scripted demo

```bash
python build_assets.py                                # once: writes assets/
python demo.py task.py demo_out --hires               # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py nospring_out --no-spring       # steps 2-3 without the spring
python demo.py task.py half_out --stop-at-refill      # spring and refill in, no clicker
```

The right hand does all three steps; the left stays at its ready pose.

1. **Spring.** The hand picks the spring up by its top, holds it straight over the barrel's back end and lowers its
   lower end 16 mm down the bore, re-centring it on the bore's axis on the way. It lets go and the spring drops to the
   seat in the nose.
2. **Refill.** The hand lifts the new refill out of the stand, point down, holds it straight over the barrel's back end
   and lowers its point 27 mm down the bore, re-centring on the way. It lets go and the refill drops through the
   spring and rests on it by its stop.
3. **Clicker.** The hand lifts the clicker out of the stand and turns it, hand and all about the vertical, until its
   hook faces the clip. It lowers the clicker into the back end over the refill's end and lets go; the clicker settles
   8 mm out. Then the finger tips take the button cap and turn it until the hook faces the clip again (it turns a
   little on the way in), and the hand goes back to its ready pose.

The demo reads the parts' and hands' poses from the simulator to steer (as a person watches the pen); the arms are
moved only by joint commands through the standard 14-value interface.

| Mode | Score |
|---|---|
| (default) | 1.0: success at 1,582 steps (a repeat run identical); spring in, refill through it, clicker latched |
| `--no-spring` | 0: no spring in the pen. The refill drops 8 mm further into the nose, and the clicker jams 44 mm out, tilted |
| `--stop-at-refill` | 0.5: spring in and the refill through it, no clicker |
| Dream-1 (`dream-1-beta`, policy run, cameras 320 x 176) | 0.0: ran the full 2,400 steps; the right arm went over to the stand and moved among the parts, but the spring was never put into the barrel |

## Scoring

*Spring in place*: the spring's centre is within 3.25 mm of the barrel's axis (inside its bore) and within 5 cm of the
nose tip, or it is round the refill's front. *Refill in*: the refill's point is within 1.5 mm of the barrel's axis and
8 to 30 mm from the nose tip, the two in line within 6 degrees, and the spring is round the refill's front (its centre
within 1.2 mm of the refill's axis, against its stop). *Clicker latched*: its button is 6 to 10 mm out of the barrel
(8 mm when latched), its lower end within 1.5 mm of the barrel's axis, in line within 6 degrees, and its hook within
20 degrees of the clip's side. *Let go*: both hands' tool points at least 4 cm from both ends of every part. *Still*:
no part moving faster than 1 cm/s.

| Score | When |
|---|---|
| 1.0 (success) | Spring in, refill through it, clicker latched, hands let go and nothing moving, together for 1 s. This ends the trial |
| 0.75 | All three, but a hand still on the pen or a part moving |
| 0.5 | Spring in and the refill through it, clicker not latched |
| 0.25 | Spring in only |
| 0 | Spring not in place, whatever else is done (including a pen put together without it) |

- **The 1 s hold:** `score()` reports the state at each step, so it reads 1.0 as soon as the conditions hold. The hold
  is enforced by yam_arena's task-file scorer: the trial ends with success only once `success()` has been true for
  `SUCCESS_HOLD_S` = 1 s of consecutive control steps (`yam_arena/inspect_embodiment.py`).
- **Real trial:** the barrel is clear, so the spring in the nose and the refill's point through it can be seen without
  touching the pen. Measure the button's height above the barrel (6 to 10 mm, with a ruler) and check the hook is
  under the clip. Then pick the pen up and click it: the point comes out and goes back only if the spring is in.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Pen:** a Clip Click retractable ballpoint (clear one-piece barrel, comfort grip), sold in packs, e.g. 8 for about $8:
  https://www.walmart.com/ip/Clip-Click-Retractable-Ball-Point-Pens-with-Comfort-Grip-Medium-Point-1-0-mm-Black-Ink-8-Count/451012109.
  Its clicker pulls out of the back end when its hook is pressed out of the notches under the clip, and the refill and
  spring then tip out. Sizes as measured from a pen of this design: barrel 12.7 cm, 8 mm across widening to 10 mm at
  the back, 1.5 cm nose cone; refill 10.3 cm with a 3 mm ink tube, a brass point 6 mm out of the tube, and its spring
  stop (two small bumps) 2.2 cm from the point; spring 3 cm long and about 4 mm across; clicker 5 cm.
- **New refill:** a plain click-pen refill without a spring, e.g. 30 for about $7:
  https://www.walmart.com/ip/CCOCC-30-Pieces-Retractable-Ballpoint-Pen-Refills-1Mm-Medium-Point-4-2-Inch-Black-Ink-Refills-Smooth-Writing-for-Click-Pens/20912020170.
  They are 10.7 cm long; trim 4 mm off the clear back end with scissors to 10.3 cm, as the listing suggests for fitting
  a pen. The pen's own spring is used again.
- **Stand:** a grey base 3 cm (x) by 16 cm (y), 4 mm thick, centred at (25, -2), with three tubes standing on it in a
  row along y, 6 cm apart: bores 7.6 mm (2 cm tall) at (25, 4), 8.3 mm (4 cm tall) at (25, -2) and 5.2 mm (3 cm tall)
  at (25, -8). Fix it to the table (tape or putty), and hold the barrel snugly in its hole (a snug hole or a little
  putty). Any stand that holds the parts upright at these positions will do.
- **Start:** the barrel nose-down in the 8.3 mm tube with its clip on the robot's left (+y); the clicker button-up in
  the 7.6 mm tube with its hook on the same side; the new refill point-down in the 5.2 mm tube; the spring standing on
  end at (20, -14).
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control. Parts made of convex pieces.
- **Sizes** follow the measured pen above, with these changes.
- **Spring:** a rigid 3 cm tube (it doesn't compress), 5.2 mm across instead of about 4 mm, with a 3.5 mm bore that
  opens into a 5.1 mm funnel at each end. Rigid contacts need it: a thin-walled 4 mm spring passes through the barrel's
  wall when dropped, sits too far off the bore's axis for the refill's point to find, and wedges in the nose cone. Its
  falling speed is capped at 0.15 m/s for the same reason.
- **Barrel:** held fixed by the stand (it can't tip or be lifted out); its bore is 6.5 mm all the way to the back end
  (the real one widens to 7 mm in its last 1.7 cm), and the inside of the nose has a flat seat for the spring 12.5 mm
  from the tip instead of the cone.
- **Refill:** its spring stop is a 5 mm collar instead of two small bumps, and its back end is rounded off so that it
  finds the clicker's funnel.
- **Clicker:** 0.35 mm loose a side in the bore (the real one is a close fit), and its latch is left out: the hook
  doesn't spring out into the notches. Instead its turning in the bore is damped, and *latched* means seated 8 mm out
  with the hook toward the clip.
- **Stand:** fixed in place.

## Reset between trials

Nothing is used up: pull the clicker out, tip the refill and spring out of the barrel, and stand the barrel, clicker and
refill back in their holes and the spring on its mark.
