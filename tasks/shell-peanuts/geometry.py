"""Dimensions (m, kg) and positions for the peanut task. Station frame: x forward from the midpoint between the arm
bases, y to the robot's left, z up; TOP is the table top.

A lobe's own frame: origin on the pod's axis at the waist (where the pod cracks), x along the axis toward the lobe's
closed end. A kernel's own frame: origin at its centre, x along its length.
"""

from __future__ import annotations

TOP = 0.7475

# -- an in-shell peanut (Virginia type, two kernels): 40 mm long, 15 mm across -------------------------------------
LOBE_L = 0.0125              # the lobe's straight wall, from the waist to where its round end begins
POD_R = 0.0075               # outside radius
WAIST_R = 0.0062             # outside radius at the waist (12.4 mm across)
WAIST_L = 0.005              # the shell widens from the waist to POD_R over this length
SHELL_T = 0.0012             # shell wall
END_SEGMENTS = 4             # the round end as this many convex slices
WALL_SEGMENTS = 12
LOBE_MASS = 0.0004
SHELL_FRICTION = (0.9, 0.8)  # dry, ridged shell

KERNEL_L = 0.0115
KERNEL_R = 0.0045            # 9 mm across
KERNEL_MASS = 0.0006
KERNEL_AT = 0.0065           # the kernel's centre this far from the waist, inside its lobe


# -- layout -----------------------------------------------------------------------------------------------------------
# the two pods lie on the bench on the robot's left, along y, back lobes toward +y; their waists at:
PODS = {"p1": (0.25, 0.16), "p2": (0.35, 0.16)}

# -- the bowl the kernels go in, and the saucer for the shells ------------------------------------------------------
BOWL_R = 0.060               # 12 cm across at the rim (outside)
BOWL_H = 0.050
BOWL_WALL = 0.004
BOWL_BASE = 0.005
BOWL_MASS = 0.25             # glazed stoneware snack bowl
BOWL_XY = (0.30, 0.0)
DISH_R = 0.050               # a 10 cm saucer, 1.8 cm deep
DISH_H = 0.018
DISH_WALL = 0.004
DISH_BASE = 0.004
DISH_MASS = 0.15
DISH_XY = (0.18, -0.03)
CERAMIC_FRICTION = (0.5, 0.45)
KERNEL_FRICTION = (0.30, 0.25)   # the kernel's papery skin on the smooth inside of the shell (min-combined)

# -- scoring ----------------------------------------------------------------------------------------------------------
CRACK_BEND_DEG = 3.0         # the shell gives once the pod is bent this far at the waist
CRACK_GAP = 0.0008           # or pulled/twisted this far apart there
CLEAR = 0.03                 # every finger pad this far from every kernel and shell piece at the end
STILL = 0.002                # nothing moves more than this over the last second
BOWL_MOVED = 0.03            # the bowl moved more than this from its mark, or tipped past 20 deg: capped
