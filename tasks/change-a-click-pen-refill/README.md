# Change a click pen's refill: spring onto the new refill's tip, front barrel over both, then join it to the back half

A click ballpoint pen lies in pieces in a desk pen stand, ready for its new refill. The new refill stands point-up in
one socket, the front barrel stands nose-up in another and the back half stands open end up in the third. The
refill's small spring stands on end on the table beside the stand. The instruction is:
**"Put the new refill in the pen: the spring onto its tip, the front barrel over them, then push the front barrel into
the back half."**

Changing a pen's refill is a familiar desk job, and it takes two hands:
- **The spring:** a 12 mm coil has to be dropped over the refill's 2 mm tip so that it slides down onto the refill's
  shoulder. Dropped off-centre, it falls beside the refill.
- **The front barrel:** it goes on over the refill's point and the spring, and has to stay in line for 6 cm, or the
  spring jams between them.
- **Joining:** the refill and spring are loose inside the front barrel, so one hand holds the barrel and the other
  holds the refill while they lift them together and lower the refill into the back half. Then one hand lets go and
  the other pushes the barrel home.

The parts must go together in this order: a pen put together without its spring looks finished but doesn't click.

Board neighbours: the four-colour pen slider (#2987) pushes a slider into a catch; cap-and-sort pens (#360) and a
lancet into a lancing device (#2259) put one part into or onto another. None assembles three loose parts along one
axis in a set order, with a part held in each hand.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the pen's parts and the stand, the monitor, `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the scoring thresholds |
| `build_assets.py` | Writes `assets/*.usda` (needs `pxr`) |
| `assets/` | The refill, spring, front barrel, back half and stand |
| `scenekit.py` | USD bodies for yam_arena task files (from `child-resistant-cap`) |
| `demokit.py` | Helpers for the scripted demo (from `child-resistant-cap`) |
| `demo.py` | The scripted full-score demo and two shortcut variants |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.0, Isaac Sim 6.1, Python 3.12).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the new refill in the pen: the spring onto its tip, the front barrel over them, then push the front barrel into the back half." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pen-refill-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 1,172 control steps (59 s of simulation); Isaac Sim takes about a minute to start.

## Scripted demo

```bash
python build_assets.py                                # once: writes assets/
python demo.py task.py demo_out --hires               # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py nospring_out --no-spring       # steps 2-4 without the spring
python demo.py task.py half_out --stop-at-barrel      # spring and front barrel on, never joined to the back half
```

1. **Spring.** The left hand picks the spring up from where it stands, holds it 3 mm over the refill's point and lets
   it go. It drops over the point and slides down to the refill's shoulder.
2. **Front barrel.** The right hand lifts the front barrel, nose up, out of the stand, brings its open end over the
   refill's point and lowers it over refill and spring. It lets go 4.5 cm down, and the barrel settles on the spring
   with the refill's point just inside its nose.
3. **Carry.** The right hand takes the front barrel again, and the left hand, coming in from the side, pinches the
   refill where it shows between the stand and the barrel. Together they lift refill, spring and barrel out of the
   stand and bring the refill's end over the back half.
4. **Join.** They lower the refill into the back half. The left hand lets go and the right hand pushes the front
   barrel into the back half's sleeve, lets go, and both hands go back to their ready poses.

No part is turned in the hand. The demo reads the parts' and hands' poses from the simulator to steer (as a person
watches the pen); the arms are moved only by joint commands through the standard 14-value interface.

| Mode | Score |
|---|---|
| (default) | 1.0: success at 1,172 steps (a repeat run identical); spring on, front barrel over it, joined to the back half |
| `--no-spring` | 0: no spring on the refill. Without it the front barrel slid further down the bare refill, and the halves weren't joined |
| `--stop-at-barrel` | 0.5: spring on and the front barrel over it, never joined to the back half |
| Dream-1 (`dream-1-beta`, policy run, cameras 320 x 176) | 0.0: ran the full 2,400 steps; both arms moved over to the stand and the parts, but the spring was never put on the refill |

## Scoring

*Spring on the refill*: the spring's centre is within 1.2 mm of the refill's axis, between the end of the point's
taper (5 mm from the point) and the refill's shoulder (18 mm). *Front barrel over the refill*: the refill's point is
within 1.5 mm of the front barrel's axis, between 1 mm outside and 8 mm inside its nose tip, and the two are in line
within 6 degrees. *Joined*: the front barrel's open end is within 1.5 mm of the back half's sleeve floor and of its
axis, the two halves are in line within 6 degrees, and the refill's back end is inside the back half. *Let go*: both
hands' tool points at least 4 cm from both ends of every part. *Still*: no part moving faster than 1 cm/s.

| Score | When |
|---|---|
| 1.0 (success) | Spring on the refill, front barrel over it, joined to the back half, hands let go and nothing moving, together for 1 s. This ends the trial |
| 0.75 | All three, but a hand still on the pen or a part moving |
| 0.5 | Spring on the refill and the front barrel over it, not joined to the back half |
| 0.25 | Spring on the refill only |
| 0 | Spring not on the refill, whatever else is done (including a pen joined without it) |

- **Real trial:** once the hands are away, check the two halves are pushed together (no gap over 1.5 mm, with a ruler)
  without moving the pen; then pick it up and click it: the point comes out and goes back only if the spring is on the
  refill. For partial credit, look into the front barrel's nose (the point shows there) and at the spring on the refill.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Pen:** a two-piece click ballpoint whose halves push or screw together: front barrel 6 cm long and 10 mm across
  with a 3 mm nose hole; back half 7 cm long and 14 mm across, holding the click mechanism. Its spare refill (11.5 cm
  long, 5 mm ink tube, 2 mm front section) and its spring (12 mm long).
- **Stand:** a grey base 3 cm (x) by 11 cm (y), 4 mm thick, with three tubes 3 cm tall standing on it in a row along
  y, 4 cm apart: bores 7, 12 and 16 mm (outside 14, 18 and 22 mm), the 7 mm one on the left. The base's centre at
  (30, -4), so the tubes are at (30, -2), (30, -6) and (30, -10). Fix it to the table (tape or putty).
- **Start:** the refill standing point-up in the 7 mm tube, the front barrel nose-up in the 12 mm tube, the back half
  open end up in the 16 mm tube, the spring standing on end at (24, 6).
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control. Parts made of convex pieces.
- **Spring:** a rigid 12 mm tube of coils (it doesn't compress). Its bore is 4 mm, about 1 mm wider than a real pen
  spring's on a 2 mm front section, because rigid simulated contacts need the clearance to slide: with a real
  spring's fit it jams on the refill.
- **Halves:** they push together into a sleeve with 0.8 mm of play; a real pen's halves screw or snap together. The
  click mechanism in the back half is left out, so the refill rests on its floor.
- **Refill:** smooth plastic ink tube, friction 0.25.
- **Stand:** fixed in place.

## Reset between trials

Nothing is used up: take the pen apart and stand its parts back in the stand, the spring on its mark.
