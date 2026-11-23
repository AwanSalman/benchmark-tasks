# Shell two peanuts: one hand holds each pod, the other bends it till it cracks; nuts in the bowl, shells on a saucer

Two in-shell peanuts lie on the table next to a white snack bowl and a small blue saucer. The instruction is:
**"Shell the two peanuts: the nuts in the white bowl, the shells on the blue saucer."**

Shelling peanuts by hand is a familiar snack-time job, and it takes two hands:
- **Cracking:** a pod is cracked by bending it at its waist. One hand holds the pod by one half while the other
  bends the far half sideways; a pod pushed on its own just rolls or slides.
- **Emptying:** once cracked, the shell comes apart in two halves with a nut loose in each. Each half has to be tipped
  open end down over the bowl, and given a shake if the nut sticks, without dropping the nut on the table.
- **Sorting:** the empty halves go on the saucer, not in the bowl with the nuts.

Board neighbours: cracking walnuts in a hand nutcracker (#1423, #2247, both closed) uses a tool and far more force;
breaking supports off a 3D print (`twist-supports-off-printed-crank`) and snapping a board out of a panel
(`snap-board-out-of-panel`) break a part off a rigid workpiece but have nothing to empty or sort afterwards.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the two pods, bowl and saucer, the crack, the monitor, `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the scoring thresholds |
| `build_assets.py` | Writes `assets/*.usda` (needs `pxr`) |
| `assets/` | A shell half (`lobe.usda`), a kernel, the bowl and the saucer |
| `scenekit.py` | USD bodies for yam_arena task files (from `child-resistant-cap`), plus an `after_spawn` hook used to define each pod's crack joint |
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
  --instruction "Shell the two peanuts: the nuts in the white bowl, the shells on the blue saucer." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/peanuts-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 1,295 control steps (65 s of simulation); Isaac Sim
  takes about a minute to start.

## Scripted demo

```bash
python build_assets.py                                # once: writes assets/
python demo.py task.py demo_out --hires               # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py whole_out --whole              # both peanuts put in the bowl uncracked
python demo.py task.py mixed_out --shells-in-bowl     # shelled over the bowl, empty halves dropped in the bowl too
```

For each peanut, the nearer one first:
1. **Pick it up.** The left hand comes down over the pod and closes across its back half, then lifts it.
2. **Hold it over the bowl.** It holds the pod's waist out over the bowl, 8.5 cm above the table.
3. **Crack it.** The right hand comes in from the right, pointing across and 20 degrees down, closes on the front half
   and pushes its far end 6 mm sideways; the pod bends at the waist and cracks. The right hand draws its half back.
   Its nut usually drops into the bowl here.
4. **Empty the back half.** The left hand moves its half 3 cm toward its own side of the bowl, tilts it 18 degrees open
   end down and gives it three small shakes, and more while the nut is still in; the nut slides out into the bowl.
   It lays the empty half on the saucer and goes back to its ready pose.
5. **Empty the front half.** The right hand turns its half open end down over the bowl, shakes it twice (more while
   the nut is still in), lays it on the saucer and goes back to its ready pose.

Each half is laid on its own spot on the saucer, 2 cm or so from the centre, so a half laid later doesn't knock an
earlier one off.

The hands take turns over the bowl: turned open end down there, the right hand's wrist reaches the place where the
left hand holds its half.

The demo reads the pods', halves', nuts' and hands' poses from the simulator to steer (as a person watches the pod); the arms
are moved only by joint commands through the standard 14-value interface.

| Mode | Score |
|---|---|
| (default) | 1.0: success at 1,295 steps (a repeat run identical); both pods cracked at the first push, all four nuts in the bowl, all four halves on the saucer |
| `--whole` | 0.0: both pods lie uncracked in the bowl, their nuts still inside |
| `--shells-in-bowl` | 0.5: all four nuts in the bowl, but the four empty halves are in it too (capped) |
| Dream-1 (`dream-1-beta`, policy run, cameras 320 x 176) | 0.0: ran the full 2,400 steps; both arms moved to the peanuts and the bowl, but no nut was shelled into the bowl |

**Robustness check.** Since real peanuts vary, the demo was also run with the simulated pod changed one way at a
time (each a separate run, the scene otherwise unchanged). It scored 1.0 in each: the crack threshold halved (1.5
degrees or 0.4 mm) and doubled (6 degrees or 1.6 mm); stickier nuts (kernel-on-shell friction 0.4, 1,346 steps);
pods 3.8 cm and 4.4 cm long. With 3.6 cm pods it scored 0.5 (a nut fell beside the bowl), so the real-world build
asks for pods of at least 3.8 cm.

## Scoring

*Cracked*: the pod's two halves have separated at the waist. *Kernel in the bowl*: it rests inside the bowl's walls,
below its rim, and is not inside a shell half. *Shell half on the saucer*: its middle rests inside the saucer's rim.
*Let go*: every finger pad (each pad's tip and a point 3 cm back) at least 3 cm from every nut and shell piece.
*Still*: nothing moved more than 2 mm over the last second.

| Score | When |
|---|---|
| 1.0 (success) | All four kernels in the bowl, all four shell halves on the saucer, no shell piece in the bowl, the bowl not knocked, let go and still, together for 1 s. This ends the trial |
| 0.15 per kernel | in the bowl, at the end |
| 0.075 per shell half | of a cracked pod, on the saucer, at the end |
| at most 0.5 | if any shell piece is in the bowl, or the bowl was moved 3 cm or tipped 20 degrees |
| at most 0.9 | without success (for example everything done but a hand still on a shell) |
| 0 | nothing shelled, including whole peanuts put in the bowl |

- **Real trial:** once the hands are away, count the nuts in the bowl and check none is still in a shell; count the
  shell pieces on the saucer and look for any in the bowl; photograph the bowl and the saucer from above. Ignore shell
  flakes under 5 mm and loose bits of papery skin.
- **Uneven splits:** a real pod may split into more than two pieces. Each piece of 5 mm or more counts as shell: a
  pod's two halves count as on the saucer only if all its pieces are, and any piece in the bowl caps the score as
  above. A kernel that breaks counts as in the bowl only if all its pieces of 5 mm or more are in the bowl.
- **Void trial:** if a pod turns out, once cracked, to have held fewer than two kernels, or a kernel grown fast to its
  shell, the trial is void: note it and rerun with fresh pods.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Peanuts:** roasted, unsalted in-shell peanuts of the large Virginia type, all from one sealed bag kept dry at room
  temperature and used within a week of opening. Two fresh pods per trial, laid along y, back half toward +y, with
  their waists at (25, 16) and (35, 16).
- **Choosing pods:** take pods with two lobes and a visible waist, 3.8 to 4.5 cm long and 1.3 to 1.7 cm across, that
  rattle when shaken (two loose kernels). Reject pods with a crack, hole or soft spot, single-lobe and three-kernel
  pods, and any that don't rattle.
- **Batch check:** before a session, crack five spare pods from the bag by hand, bending each at its waist. Use the
  bag if at least four crack at the waist into two main pieces with their kernels loose; otherwise (pods damp and
  bending without cracking, or crumbling) use a fresh bag.
- **Bowl:** a white snack bowl about 12 cm across and 5 cm tall, centred at (30, 0).
- **Saucer:** a small saucer about 10 cm across, centred at (18, -3).
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Pod:** two hollow shell halves (0.4 g each; the wall and round end as convex pieces), 4.0 cm long, 1.5 cm across
  at its lobes and 1.24 cm at its pinched waist, with a 9 x 11.5 mm kernel (0.6 g) loose in each half. The waist is drawn but
  not modelled for contact: the halves collide as straight 1.5 cm tubes, so a nut slides out without climbing the
  real shell's slight inner ridge at the waist.
- **The crack:** the halves start joined at the waist by a fixed joint. The task switches the joint off, for good,
  once the pod is bent 3 degrees or pulled or twisted 0.8 mm apart there (PhysX's own joint break thresholds didn't
  act in this setup, so the task applies the threshold itself). Real pods also crack under a squeeze and along
  uneven lines, and can shed small bits; here each pod always splits into the same two halves at the waist.
- **Kernels:** their papery skins are left out; the kernel-on-shell friction is 0.3, so a nut slides out of a half
  tilted about 17 degrees or more, as loose roasted nuts do.
- **Bowl and saucer:** rigid, loose on the table.

## Reset between trials

Peanuts are used up: lay two fresh, intact pods on their marks. Empty the bowl and the saucer and stand them back on
their marks.
