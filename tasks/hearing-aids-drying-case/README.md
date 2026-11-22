# Open two hearing aids' battery doors for the night

Two behind-the-ear hearing aids lie on the table with their battery doors shut, next to an open drying case. The
instruction is: **"Open the battery door on each hearing aid, then lay both aids in the drying case for the night,
doors open."**

Every night, hearing-aid users put their aids in a drying case, and the makers' instructions say to open each battery
door first: the zinc-air cell stops draining and moisture can get out. It is a two-handed job:
- the aid weighs 3 g and its door is a 10 mm flap that clicks shut; pulled by its nail lip, an aid that nothing holds
  just slides or turns on the table, so one hand holds the aid while the other swings the door open;
- the door must go past its click (12 deg) or it springs shut, and it must stay open while the aid is carried and
  laid down.

The scored skill is the door: an aid laid in the case with its door shut scores nothing. Board neighbours: the
AirPods case task (#2784) seats earbuds stem-first in their case's wells and shuts its lid; here the case is a plain
tray (either aid anywhere in it) and the difficulty is opening a 10 mm hinged door on a 3 g part, one hand on the
part and the other on the door.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the two aids, the case, the door detent, the monitor (doors, which aids lie in the case, the hands), `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the scoring thresholds |
| `build_assets.py` | Writes `assets/*.usda` (needs `pxr`) |
| `assets/` | The two aids (each an articulation: shell and battery door on a hinge) and the drying case |
| `scenekit.py` | USD bodies and articulations with per-joint drives for yam_arena task files (from `child-resistant-cap`) |
| `demokit.py` | Helpers for the scripted demo (from `child-resistant-cap`) |
| `demo.py` | The scripted full-score demo, and a variant that leaves the doors shut (0) |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.0, Isaac Sim 6.1, Python 3.12).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the battery door on each hearing aid, then lay both aids in the drying case for the night, doors open." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/aids-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 1,028 control steps (51 s of simulation); Isaac Sim
  takes about a minute to start.

## Scripted demo

```bash
python build_assets.py                              # once: writes assets/
python demo.py task.py demo_out --hires             # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py nodoors_out --no-doors       # the aids laid in the case with their doors shut: 0
```

For each aid, the farther one first:
1. **Pick it up.** The left hand comes down over the aid, closes across its back and front 2 cm up the shell (where
   its sides are parallel) and lifts it.
2. **Hold it out.** It holds the aid's bottom end out toward the right hand, 5 cm above the table.
3. **Open the door.** The right hand comes in from the right, pointing across and 30 deg down, closes its fingertips
   either side of the door's nail lip, and turns with the door about its hinge pin to 95 deg; then it lets go and
   backs off. The door stays open.
4. **Lay it in the case.** The left hand turns the aid square to the tray, lowers it onto the tray's floor (the first
   aid on the far side, the second on the near side), opens just clear of it and rises.

The demo reads the aids', doors' and hands' poses from the simulator to steer (as a person watches the aid); the arms
are moved only by joint commands through the standard 14-value interface.

**Doors left shut** (`--no-doors`): the same picks and placings without opening the doors: both aids in the case,
0.

## Scoring

*Door open*: 60 degrees or more. *In the case*: the aid's centre (2 cm up its shell) at least 4 mm inside the tray's
walls, resting on the tray's floor (its centre at most 3 mm above lying flat) and within 20 degrees of lying on its
side. *Let go*: every finger pad of both grippers (each pad's tip and a point 3 cm back) at least
2 cm from both aids. *Still*: the aids moved less than 2 mm over the last second.

| Score | When |
|---|---|
| 1.0 (success) | Both aids in the case with their doors open, let go and still, together for 1 s. This ends the trial |
| 0.25 per open door | At the end |
| 0.25 more per open-door aid in the case | At the end |
| at most 0.75 | Without success (for example everything done but an aid still held) |
| 0 | No door open, including both aids laid in the case with their doors shut |

- **Real trial:** read each door's angle with a protractor card laid against the aid (or from an overhead photo against
  a 60 degree line), and check each aid lies flat on the tray's floor clear of a line drawn 4 mm inside its walls.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Hearing aids:** two behind-the-ear hearing aids that take size 13 cells (a display or demo pair, about 4 cm shells);
  glue an 8 mm nail lip (a 7 x 8 mm piece of 1.5 mm plastic card) on each battery door. Lay them on their sides
  with their bottom ends' centres at (36, 3) and (24, 3), bottom ends toward the robot's right.
- **Drying case:** an electric hearing-aid dryer with an open tray (for example a Dry & Store unit), or any 10 x 8.5 cm
  box with a 9.2 x 7.4 cm tray 8 mm deep, centred at (30, 15.5), lid open.
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Aids:** free two-part articulations, 3 g: the shell (convex pieces, the hook as capsules) and the battery door
  (0.6 g) on a hinge pin. The door's click is a spring pulling it shut within 12 degrees of shut (about 0.5 N at the
  lip at 12 degrees); beyond that it holds where it is put, like the friction of a real door's pin.
- The nail lip stands 8 mm out so that the gripper's 15 mm-wide fingertips can take it clear of the door's end; real
  lips stand 1-2 mm out and are opened with a fingernail.
- **Case:** one rigid body (300 g) with its lid fixed open. No drying, no electronics, no separate cell.

## Reset between trials

Close both battery doors (click them shut) and lay the aids back on their marks; stand the case on its marks with its
lid open.
