# Refit a kettle's limescale filter: lower it into the slot behind the spout, curve out, past the rim; put the lid back

A KitchenAid 1.25 L electric kettle stands on the bench, off its power base, with its lid on. Its limescale filter,
rinsed, lies flat on the bench beside it. The filter is a small curved mesh plate that sits inside the kettle, in a
slot behind the spout. KitchenAid's instruction for putting it back is: "Open the Lid. Slide the filter down into the
slot behind the spout, ensuring that the curve facing outward on the filter matches the curve of the electric kettle."

One hand takes the lid off by its knob and holds it up out of the way, while the other puts the filter in. The hand is
wider than the lid opening, so it cannot follow the filter in: it holds the filter low on its grip tab, reaches over
the rim, lines the filter's two side edges up with the slot's two guides about 1 cm below the rim, with the filter's
curve facing outward, lowers it into the guides and lets it slide down to its stop. Then the lid goes back on.

## Run

From this folder (Isaac Sim 6.1, Isaac Lab 3, Isaac Lab-Arena 0.3.0, `yam_arena`):

```
python build_assets.py                          # writes assets/kettle.usda, lid.usda, base.usda, filter.usda (committed)
python demo.py task.py OUT                      # scripted demo: per-camera videos, setup.png, result.json in OUT
python demo.py task.py OUT --hires              # the same at 1280 x 704
python demo.py task.py OUT --no-record --drop   # the routes in the table below: --drop, --no-correct, --hold, --no-lid
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=640 -E cam_height=352 --instruction "Put the limescale filter back in the kettle and put the lid back on." \
    --scorer task_score --max-steps 1600                                  # a Dream-1 run
```

Instruction given to the policy: "Put the limescale filter back in the kettle and put the lid back on." Nothing is
random: one fixed layout. One demo run takes about 2 minutes of wall time on one RTX 6000 Ada (most of it Isaac Sim
start-up).

## Scoring

Judged from the state now, and only while both hands' tool points are at least 4 cm from the filter and from the lid,
and the filter and the lid are still (slower than 1 cm/s); until then the score is 0. Nothing is carried over from an
earlier moment.

| State | Score |
|---|---|
| Filter seated and lid on | 1.0 |
| Filter seated, lid not on | 0.5 |
| Filter in its slot, curve outward, but not down on its stop (lid on or not) | 0.25 |
| Anything else | 0.0 |
| The kettle tipped past 20 degrees at any moment | 0.0 |

- Seated: in its slot, curve outward, standing on its stop within 1.5 mm and within 6 degrees of its seated
  orientation.
- In its slot: the filter's origin is within the slot's play of its seated place across the slot (1.5 mm radially,
  1.3 mm sideways), it leans less than 15 degrees, and its bottom edge is at least 5 mm down the guides.
- Lid on: its centre within 6 mm of the opening's centre (its plug is in the opening), resting on the top within 2 mm,
  and level within 5 degrees.

The run ends with success once 1.0 has held for 1 s. The lid starts on, so leaving it on without the filter scores 0.

Real trial: once the hands are away, check the lid sits flat in the opening; then lift it and look down through the
opening. Seated: the filter's top edge is level with the top of the two guides and its curve follows the wall. In its
slot: both side edges are behind the guides but the top edge stands above them. The kettle tipping over (past 20
degrees) is a 0.

## The routes, and what they score

`demo.py` modes, each run on the final scene (RTX 6000 Ada); the score is the harness's `task_score`.

| Mode | What it does | Score |
|---|---|---|
| (default) | the right arm grips the lid's knob, lifts the lid off and holds it up over the power base while the left arm pinches the filter's grip tab; the left carries the filter upright over the kettle correcting by its measured pose, lowers it until its bottom edge is 6 mm down the guides and lets go: it slides to its stop; the left goes home and the right puts the lid back on, correcting by where the lid hangs in the hand | 1.0 (636 steps, 31.8 s; a repeat run identical) |
| `--no-lid` | the same, but the lid is set down on the bench instead of back on the kettle | 0.5 |
| `--drop` | let go of the filter upright, curve outward, 25 mm over the guides and 4 mm off its slot | 0.0: it wedges across the slot's mouth, tilted 49 degrees, 45 mm above its stop |
| `--no-correct` | the filter moves planned from its starting pose, without correcting by where it really hangs in the hand | 0.0: it misses the guides, the hand knocks the kettle (moved 28 mm, tilted 9 degrees) and the filter ends on the bench |
| `--hold` | lower the filter into the guides but keep holding its grip tab | 0.0: never judged, the hand stays 10 mm from it |

In every run but `--no-correct` the kettle stayed put (at most 0.1 degrees of tilt and 0.5 mm of travel).

Turned round (curve facing inward), the filter cannot enter the slot at all: its 1.75 mm bow each way makes it about
5 mm deep across, against the slot's 2.5 mm.

One arm alone could do the job by setting the lid down on the bench first and picking it up again after; the demo
uses both arms at once, as a person does.

## Why it is hard

- Two arms on one kettle: one takes the lid off its knob and keeps it clear of the other arm's path while the other
  works through the opening, then puts it back so its plug drops into the opening.
- The hand is wider than the 104 mm lid opening, so it never goes in: the filter is held by the top of its grip tab and
  lowered from above the rim, its bottom edge out of the hand's reach. Its two side edges must meet the two guides,
  1 mm of play radially and 0.8 mm sideways, 10 mm below the rim, where the cameras see them only past the hand.
- The YAM arm reaches only about 0.20 m above the bench with its hand pointing straight down; the rim is 0.23 m up and
  the knob's top 0.262 m. The hands have to come in tilted (the left 45 degrees forward with the filter hanging upright
  under it, the right 35 degrees toward the knob), which means picking the filter up off the bench with the hand
  tilted back.
- The wrist is soft: a 4 g filter held at arm's length hangs a few millimetres and up to 10 degrees away from where the
  hand was told to put it, so the demo corrects by where the filter actually is; the lid likewise.
- Orientation: curve outward, grip tab inward. Turned round, it does not fit.

## Modelling

- The kettle is one free rigid body: a 170 mm cylinder (the real body is a dome, 17.8 cm across at its widest), its
  rim 230 mm up, a 3 mm stainless wall with the spout hole through it, the spout, the handle, a floor and a top with a
  104 mm lid opening pushed toward the spout. 1.14 kg, as published.
- The filter's slot: the filter's outer face rests on the wall's inside, between two guide lips (3 mm thick,
  overlapping the filter's edges by 2.5 mm, 1 mm radial play) and two side walls (0.8 mm play a side), all flared open
  by 1 mm over their top 3 mm (a moulded lead-in), over a stop ledge 60 mm below the rim. Estimates from product
  photos; KitchenAid publishes none.
- The filter: a curved plate 34 mm across (chord), 50 mm tall and 1.5 mm thick, bent to the wall's 82 mm inner radius,
  4 g, with a 20 mm grip tab standing in from its inner face (its top 12 mm above the rim once seated); its bottom
  outer corners are chamfered 1 mm. The mesh is a solid plate. Estimates from product photos.
- The lid: a free rigid aluminium disc 112 mm across and 6 mm thick, 90 g, with a 92 mm locating plug 6 mm deep under
  it (6 mm of play in the opening), its spout-side part raised 16 mm over the seated filter's grip tab, and a knob
  24 mm across and 26 mm tall (rim 230 + lid 6 + knob 26 = the published 262 mm). Estimates from product photos.
- The power base: a free black puck 160 mm across, 22 mm tall, 350 g; its cord is not modelled. Estimates.
- Frictions: kettle and base on bench 0.6, filter (polypropylene) 0.35, lid 0.5 (estimates).
- Physics: PhysX convex hulls, 240 Hz, control at 20 Hz. The filter uses speculative continuous collision and a 1 mm
  contact offset, so a dropped filter (moving about 3 mm per physics step) cannot sink into the kettle's 3 mm walls.
  The demo reads object poses from the simulator; a policy sees only the cameras.

## Sizes

| Part | In the scene | Published | Source |
|---|---|---|---|
| kettle height | rim 230 mm; 262 mm to the top of the knob with the lid on | 26.2 cm | KitchenAid 5KEK1222 spec sheet |
| kettle width x depth | 170 mm body; 238 mm handle to spout tip | 17.8 x 22.6 cm | KitchenAid 5KEK1222 spec sheet |
| kettle mass | 1.14 kg | 1.14 kg (net) | KitchenAid 5KEK1222 spec sheet |
| capacity | not modelled (empty) | 1.25 L | KitchenAid 5KEK1222 spec sheet |
| removable filter, removable lid, aluminium lid | yes, yes, yes | yes, yes, yes | KitchenAid 5KEK1222 spec sheet |
| lid opening, slot, filter size and mass, grip tab, lid shape and mass, power base | 104 mm; see Modelling | none published | estimates from product photos |

## Real trial

Kit: a KitchenAid 1.25 L electric kettle (KEK1222 in the US, 5KEK1222 in Europe) with its power base, empty and
unplugged, the kettle off the base. Set the kettle on the bench at its mark, spout away from the robot, lid on; the
base at its mark. Lay the filter flat at its mark, outer (convex) face down. Judge from the trial video once both
grippers are 4 cm clear and nothing is moving (see Scoring). Reset: lift the lid off, lift the filter out by its top,
as KitchenAid describes ("Grasp the top of the Filter behind the Spout and then pull out"), lay it back on its mark and
put the lid back on; about 30 s (estimate); nothing is consumed.

Forces and safety: the filter weighs a few grams, the lid about 90 g and the kettle 1.14 kg; nothing is hot (the
kettle is empty, cold and unplugged), sharp or fragile.

## Sources

- KitchenAid, 1.25 L Water Kettle 5KEK1222 product sheet (www.kitchenaid.eu), https://cdn.shopify.com/s/files/1/0716/9140/3576/files/5KEK1222_1.pdf:
  "Product dimensions H×W×D 26.2 × 17.8 × 22.6 cm", "Net weight 1.14 kg", "Kettle capacity 1.25 L", "Removable
  limescale filter yes", "Removable lid yes", "Dome style kettle", "aluminium lid". Checked 22 Nov 2026.
- KitchenAid product help, "Cleaning the Filter - Electric Kettle",
  https://producthelp.kitchenaid.com/Countertop_Appliances/Other_Countertop_Appliances/Kettles/Electric_Kettle/Cleaning_and_Maintenance/Cleaning_the_Filter_-_Electric_Kettle:
  "Open the Lid.", "Slide the filter down into the slot behind the spout, ensuring that the curve facing outward on the
  filter matches the curve of the electric kettle." and "Grasp the top of the Filter behind the Spout and then pull out
  to remove." Checked 22 Nov 2026.
- KitchenAid, 1.25 L Electric Kettle (KEK1222), https://www.kitchenaid.com/countertop-appliances/kettles/electric-kettles/p.KEK1222ER.html.
- Slot, filter, lid opening, lid shape, grip tab, power base and friction figures are estimates from product photos.
- All meshes are built in `build_assets.py` from boxes and arcs; no third-party meshes are used.

KitchenAid is a trademark of Whirlpool Corporation, named for identification only.

## Licence

MIT
