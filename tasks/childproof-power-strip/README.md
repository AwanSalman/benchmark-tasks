# Childproof a free-standing power strip

A tower power strip stands free on the table with a desk lamp plugged into its bottom outlet; three child-safety
outlet caps stand in a foam tray. The instruction is: **"Childproof the power strip: put a safety cap into every
outlet that is not in use, then let go."**

Fitting safety caps is one of the first things parents do when a toddler starts crawling. On a light, free-standing
strip it takes both hands:
- a cap only goes home against the outlet's contact springs (about 3 N here), and the strip is light (0.5 kg) and
  tall (30 cm): pushed with one hand it tips back and the cap stops short;
- so one hand holds the strip while the other lines each cap's two blades up with a pair of 3.5 mm slots and pushes it
  fully in;
- the outlet in use is left alone: the lamp stays plugged in, and the strip must end where it stood.

Childproofing outlets has been tried on the board before (#1547, not accepted) with plain cylinders dropped into
open square wells; here the outlets are real NEMA 5-15 slots with sprung contacts, the caps have real blades and are
held in by those contacts, and the strip has to be held while they go in.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the strip, the caps, their tray and the lamp, the monitor (which cap is home where, the plug, the strip, the hands), `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the scoring thresholds |
| `build_assets.py` | Writes `assets/*.usda` (needs `pxr`) |
| `assets/` | The strip (an articulation: its body and 16 sprung contact leaves), the three caps, the tray, the lamp (fixed base, cord, plug) |
| `scenekit.py` | USD bodies and articulations with per-joint drives for yam_arena task files (from `child-resistant-cap`, plus `fix_root` for articulations fixed where spawned) |
| `demokit.py` | Helpers for the scripted demo (from `child-resistant-cap`) |
| `demo.py` | The scripted full-score demo, and a one-handed variant that scores 0 |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.0, Isaac Sim 6.1, Python 3.12).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Childproof the power strip: put a safety cap into every outlet that is not in use, then let go." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/strip-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 1,107 control steps (55 s of simulation); Isaac Sim
  takes about a minute to start.

## Scripted demo

```bash
python build_assets.py                              # once: writes assets/
python demo.py task.py demo_out --hires             # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py onehand_out --no-brace       # the same moves with the right hand alone: 0
```

1. **Hold the strip.** The left hand comes in from the robot's left above the outlets, fingers either side of the
   strip, and closes on its front and back faces 28 cm up.
2. **Take a cap.** The right hand comes down on a cap in the tray, closes on its grip ridge, lifts it out and turns
   it so its blades point at the strip.
3. **Line it up.** It brings the cap square in front of a free outlet, 8 cm, 3 cm and then 1.2 cm out, measuring where
   the cap sits in the hand at each stage.
4. **Push it home.** It pushes along the outlet's axis, steering each step on the cap's measured offset, until the
   cap's face is on the strip's (or it stops moving in); then it opens just clear of the ridge and backs off.
5. Steps 2-4 for the other two caps, bottom free outlet first.
6. **Let go.** The left hand opens and moves away; the right hand goes back to its ready pose.

The demo reads the caps', strip's and hands' poses from the simulator to steer (as a person watches the cap and the
slots); the arms are moved only by joint commands through the standard 14-value interface.

**One hand only** (`--no-brace`): the same moves without the left hand. Each push tips the strip back on its feet
before the cap is home; the caps stop 2.6 to 2.7 mm short of the face (one falls out), and it scores 0.

## Scoring

Measured relative to the strip. *Home*: a cap's back face within 2 mm of the strip's face (10 of its 12 mm blades
in), its centre within 2.5 mm of the outlet's centre and its axis within 5 degrees of square, in one of the three
outlets the lamp is not using. *Where it stood*: the strip's base within 10 mm of its start, leaning no more than 5
degrees. *Plugged in*: the lamp's plug no more than 3 mm out. *Let go*: every finger pad of both grippers (each pad's
tip and a point 3 cm back) at least 2 cm from the strip with its plug and from every cap. *Still*: the strip and caps
moved less than 2 mm over the last second.

| Score | When |
|---|---|
| 1.0 (success) | All three caps home, the lamp plugged in, the strip where it stood, let go and still, together for 1 s. This ends the trial |
| 0.25 per cap | At the end, each cap home in a free outlet (so 0.75 for three caps still held, or with the hands not clear) |
| at most 0.5 | The strip shoved more than 10 mm or tipped more than 5 degrees (or knocked over), or the lamp's plug pulled more than 5 mm out |
| 0 | No cap home, including pushing caps in without holding the strip |

- **Real trial:** read each cap against a steel rule laid across the strip's face (a cap is home if the rule's gap
  to its face is 2 mm or less), the strip's base against tape marks round its feet, and the plug against a scribed
  line 3 mm out on its body.
- There is only one free outlet per cap and only three caps, so pulling the lamp's plug gains nothing and is penalised.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Power strip:** a tower (vertical) power strip with widely spaced outlets on one face, about 6 x 6 x 30 cm and
  0.5 kg, standing on its feet at (43, -6), outlets toward the robot. Tower strips with outlets 5 to 6 cm apart (sold
  for bulky adapters) are common; one with outlets on several faces works with the other faces taped over.
- **Safety caps:** standard two-blade child-safety outlet plugs (for example Safety 1st Outlet Plugs, or any NEMA 1-15
  outlet plug cap with a thumb grip), three of them, standing blades down in a slitted foam block (a cut piece of
  packing foam) at (26, -21.5).
- **Lamp:** any small desk lamp with a two-blade plug, base at (66, 15), its cord laid in front of the strip as in the
  screenshot, plugged fully into the bottom outlet. Unplug the strip from the wall for the trial.
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Strip:** a free rigid body (0.5 kg) on four rubber feet (friction 0.6). Its slots are real openings into cavities
  behind the face. In each slot, two rigid brass leaves on sprung slides (1,200 N/m, preloaded so each presses about 4 N
  on a blade; friction 0.2), with 17-degree lead-in ramps, grip the blade from both sides: they hold the caps and
  the plug in and set the push needed (about 3 N for a cap). Real contacts are bent spring strips, and real caps often
  need 10 N or more.
- **Caps:** rigid, 6 g, friction 1.0, blades with chamfered tips.
- **Lamp:** fixed base; its cord is 18 rigid 3 cm links on soft hinges whose rest shape is the way it lies; the plug
  is a rigid 30 g body on the last hinge.
- No electricity. Dimensions are representative of common tower strips and outlet caps, not measured from one product.

## Reset between trials

Pull the caps out and stand them back in the tray; stand the strip back on its marks; push the lamp's plug fully back
into the bottom outlet with its cord laid as in the screenshot.
