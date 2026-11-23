# Get 40 cents out of a car coin holder: slide a quarter, a dime and a nickel out from under their spring-loaded lips

A round car coin holder, the kind that sits in a cup holder, stands in a shallow tray on the bench. It has four
spring-loaded tubes, one each for quarters, dimes, nickels and pennies. A spring under each stack pushes the top coin up
against a lip. The lip covers the coin's rim except for an opening on top and a slot that runs out to the holder's edge.
Owners take coins out as they would in a car: a thumb presses on the top coin through the opening and slides it out
sideways, under the lip, along the slot and over the edge. The next coin then comes up to the lip.

The task is to get exactly 40 cents into the tray. With the coins in this holder, the only way to make 40 cents is a
quarter, a dime and a nickel: the hand has to recognise the tubes by coin size and colour and take one coin from each
of three tubes, and none from the penny tube.

## Run

From this folder. The runs used Python 3.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0,
Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and inspect-robots-dream 0.1.0, with
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) at commit
`bd62771e60320433893bd1d2df0fc79b5ddcd6d1`:

```
python build_assets.py                          # writes assets/holder.usda, the four coins and tray.usda (committed)
python demo.py task.py OUT                      # scripted demo: per-camera videos, setup.png, result.json in OUT
python demo.py task.py OUT --hires              # the same at 1280 x 704
python demo.py task.py OUT --route overpay      # the routes in the table below: overpay, short
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "Take 40 cents out of the coin holder into the tray." \
    --scorer task_score --max-steps 1200                                  # a Dream-1 run
```

Instruction given to the policy: "Take 40 cents out of the coin holder into the tray." Nothing is random: one fixed
layout. One demo run takes about 3 minutes of wall time on one RTX 6000 Ada (most of it Isaac Sim start-up).

## Scoring

Judged from the state now, and only while both hands' tool points are at least 4 cm from every coin that is out of its
tube, and those coins are still (slower than 1 cm/s); until then the score is 0. A coin is in the tray when its centre
is inside the tray's wall and within 12 mm of its floor; in its tube while its centre is within 3 mm of the tube's axis
and under the lip.

| State | Score |
|---|---|
| Exactly 40 cents in the tray | 1.0 |
| Over 40 cents in the tray | 0.25 |
| Under 40 cents: 0.25 for each of the quarter, dime and nickel in the tray | 0 to 0.5 |
| Any coin on the bench (or off both tray and holder) | at most 0.5 |
| The holder tipped past 20 degrees at any moment | 0.0 |

Overpaying scores below a near miss on purpose: the instruction asks for 40 cents, not at least 40. The job is
to count out exactly 40 cents: taking extra coins is a counting error, and the change left in the holder is what
the next toll needs.

The run ends with success once 1.0 has held for 1 s.

Real trial: once the hands are away and nothing is moving, count the coins in the tray. Exactly 40 cents, nothing on
the bench and the holder never knocked over: 1.0.

## The routes, and what they score

`demo.py` modes, each run on the final scene (RTX 6000 Ada); the score is the harness's `task_score`.

| Mode | What it does | Score |
|---|---|---|
| (default) | the left arm slides a dime and then a nickel out, the right arm a quarter | 1.0 (40 cents; success at 745 steps, 37.3 s) |
| `--route overpay` | two dimes, a nickel and a quarter | 0.25 (50 cents) |
| `--route short` | a dime and a quarter | 0.5 (35 cents, two of the three coins) |

A Dream-1 run (320 x 176 cameras, 1,200 steps) scored 0.0: its hands moved about near their start and never reached
the holder.

For each coin the hand closes its fingers and sets the closed fingertips down on the top coin at the back of the lip's
opening, pressing lightly. It then draws them along the slot, keeping them a few millimetres ahead of the coin's centre.
Friction carries the coin under the lip, along the slot and over the edge, and it drops into the tray. Pressed too
lightly the fingertips slip over the coin; pressed too hard they push the whole stack down against its spring and the
coin catches under the lip. When the fingertips slip ahead of the coin, the demo lifts them, sets them back on the coin
and presses a little harder (up to 4 times). In every run the holder stayed put (at most 0.07 degrees of tilt, no
measurable travel).

The dime is the hardest coin: it is the thinnest and lightest. Pushed second, after the quarter, it once stuck about
8 mm out after four re-grips; pushed first, as in the routes above, it came out every time.

## Why it is hard

- Counting: the policy has to pick the three coins that make 40 cents, telling the four tubes apart by coin size
  (18 to 24 mm) and colour (the penny is copper), and stop when it has 40 cents.
- The coins are held under lips 1.6 mm wide and can only be slid out, by friction, through a slot a little narrower
  than the coin. The fingertips must press the coin hard enough not to slip but not so hard that the stack sinks:
  the spring under each stack gives only about 0.8 N.
- The YAM wrist is soft: a fingertip told to press 1.5 mm into the coin presses only about 0.3 mm, and slips forward
  over the coin if the arm moves too fast. A policy must watch the coin move and push again when it stops.
- The coins are small (the dime is 1.35 mm thick) and seen by the cameras mostly past the hand.

## Modelling

- The holder is one articulation: a black plastic body 76.2 mm across and 101.6 mm tall, 175 g, on a rubber base, as
  listed for the Seven Sparta A-1231. Its four tubes sit in a 2 x 2 cluster, each tube's exit pointing outward. Each
  tube's coin stack rests on a follower plate on a prismatic joint, driven up by a soft spring (30 N/m, 0.8 N at a full
  stack, damping 10 N s/m), with a bottom stop at the full stack.
- Each tube's lip: a top plate 1.5 mm thick, with an opening 3.2 mm smaller than the coin, a slot 3.2 mm narrower than
  the coin running out to the holder's edge, a channel under the lip one coin thick plus 0.6 mm, side guides, and a
  wall lowered to the channel floor on the exit side. Estimates from product photos and owners' reviews; the maker
  publishes no drawings.
- Coins: US Mint diameter, thickness and mass, each a 32-sided prism. 4 quarters, 2 dimes, 2 nickels and 2 pennies,
  fewer than a full holder.
- The tray: a light grey dish 170 mm across with an 8 mm wall, 150 g, under the holder.
- Frictions: coin on coin and coin on the holder's moulded plastic 0.25 (combining with the larger of the two
  materials, so a fingertip on a coin uses the fingertip's 0.5), holder base on tray 0.6, tray on bench 0.5. Estimates.
- Physics: PhysX convex hulls, 240 Hz, control at 20 Hz. Many coins touching one articulation needs more GPU solver
  partitions than PhysX's default, so the scene raises them to 32 (`scenekit.py`). The demo reads object poses from the
  simulator; a policy sees only the cameras.

## Sizes

| Part | In the scene | Published | Source |
|---|---|---|---|
| holder | 76.2 mm across, 101.6 mm tall, 175 g | 3 x 3 x 4 in (7.62 x 7.62 x 10.16 cm), 6.2 oz | Seven Sparta A-1231 listings |
| quarter | 24.26 mm, 1.75 mm thick, 5.670 g | 24.26 mm, 1.75 mm, 5.670 g | US Mint coin specifications |
| dime | 17.91 mm, 1.35 mm thick, 2.268 g | 17.91 mm, 1.35 mm, 2.268 g | US Mint coin specifications |
| nickel | 21.21 mm, 1.95 mm thick, 5.000 g | 21.21 mm, 1.95 mm, 5.000 g | US Mint coin specifications |
| penny | 19.05 mm, 1.52 mm thick, 2.500 g | 19.05 mm, 1.52 mm, 2.500 g | US Mint coin specifications |
| lips, slots, channels, springs, tray | see Modelling | none published | estimates from product photos and owners' reviews |

## Real trial

Kit: a four-tube spring-loaded car coin holder (for example the Seven Sparta A-1231), a shallow dish 17 cm across, and
4 quarters, 2 dimes, 2 nickels and 2 pennies loaded into their tubes. Stand the holder in the dish at its mark, the
quarter tube on the robot's right, toward the robot. Judge from the trial video once both grippers are 4 cm clear and nothing is
moving (see Scoring). Reset: press each coin back into its tube from above, about 30 s (estimate); nothing is consumed.

Forces and safety: the coins weigh a few grams each and the holder 175 g; nothing is hot, sharp or fragile.

## Sources

- Seven Sparta car coin holder A-1231, retail listings (Desertcart): "3 x 3 x 4 inches" (7.62 x 7.62 x 10.16 cm),
  6.2 oz, ABS plastic, tension springs, separate tubes for quarters, dimes, nickels and pennies, non-slip rubber base.
  An owner's review says coins slide out fairly easily when the holder is half to four-fifths full. Checked 22 Nov 2026.
- United States Mint, Coin Specifications, https://www.usmint.gov/learn/coins-and-medals/coin-specifications:
  diameter, thickness and mass of the cent, nickel, dime and quarter.
- Lip, slot, channel, spring and tray figures are estimates from product photos and owners' reviews.
- All meshes are built in `build_assets.py` from discs, prisms and polygons; no third-party meshes are used.

Seven Sparta is a trademark of its owner, named for identification only.

## Licence

MIT
