"""Dimensions (m, kg) and positions of the scene, shared by build_assets.py, task.py and demo.py.

Station frame: x forward from the midpoint between the arm bases, y to the robot's left, z up; TOP is the table top.

A hearing aid's own frame: origin at the centre of its bottom end (the battery-door end), x along its body toward the
ear hook, y from its back (the side away from the face) to its front, z across its thickness; lying on its side on
the table, z is up.
"""

from __future__ import annotations

import math

TOP = 0.7475

# -- a behind-the-ear hearing aid ------------------------------------------------------------------------------------
AID_L = 0.040                 # the shell, bottom end to the top of its spine (the hook adds about 14 mm)
AID_D = 0.012                 # back to front
AID_T = 0.009                 # thickness (side to side)
AID_MASS = 0.0030             # with its cell
DOOR_L = 0.010                # the battery door is the shell's bottom 10 mm
DOOR_MASS = 0.0006
HINGE = (0.0, AID_D / 2)      # its hinge pin: across the thickness at the bottom end's front corner
TAB = (0.0015, 0.007, 0.008)  # the door's nail lip: an upright fin on its bottom end at the back: 1.5 mm thick, 7 mm tall, 8 mm out
DOOR_LIMIT = 105.0            # deg, open
DETENT = 12.0                 # deg: inside this the door snaps shut; beyond it, it stays where it is put
DOOR_K = 0.03                 # N m/rad, the detent spring (about 0.5 N at the lip at the detent's edge)
DOOR_DAMPING = 0.0004
HOOK_R = 0.0018               # the ear hook's tube
HOOK_ARC = ((0.040, 0.000), (0.046, 0.002), (0.050, 0.006), (0.052, 0.011), (0.051, 0.016))   # its centreline (x, y)

AID_Z = TOP + AID_T / 2 + 0.0005   # resting on its side

# the test layout (feasibility gate): one aid lying in front of the robot, its door end toward the right arm
GATE_AID = (0.30, 0.0)        # its bottom end's centre

# -- the scene ---------------------------------------------------------------------------------------------------------
# Both aids lie on their sides, bottom (door) end toward the right arm (-y), the ear hook toward the robot (-x).
AIDS = ("left", "right")                     # a left and a right aid (identical in shape)
AID_START = {"left": (0.36, 0.030), "right": (0.24, 0.030)}   # each bottom end's centre
WORK = (0.30, 0.0, TOP + 0.0545)             # where the left hand holds an aid's bottom end while its door is opened

# the drying case: a white box standing on the table, an open tray in its top, its lid open
CASE_C = (0.30, 0.155)                       # centre of its footprint
CASE_SIZE = (0.104, 0.085, 0.022)            # x, y, height
CASE_MASS = 0.30
WELL = (0.092, 0.074, 0.008)                 # the tray: x, y, depth
PLACE_DX = {"right": -0.025, "left": 0.025}  # where the demo lays each aid in the tray (any place in it counts)
WELL_FLOOR = TOP + CASE_SIZE[2] - WELL[2]
LID_T = 0.004
LID_OPEN = 100.0                             # deg, from shut (leaning back past upright)

# where the demo lays an aid: bottom end 17 mm in from the tray's -y end, 6 mm beyond its place's centre in x
IN_WELL = (0.006, -WELL[1] / 2 + 0.017)

# scoring
DOOR_OPEN = 60.0                             # deg: a door counts as open from here
IN_MARGIN = 0.004                            # in the tray: the aid's centre at least 4 mm inside its walls
IN_Z = 0.003                                 # resting on the tray's floor: its centre at most 3 mm above lying flat
FLAT = 20.0                                  # deg from lying on its side
LET_GO = 0.02
STILL = 0.002
AID_CENTRE = (0.020, 0.0, 0.0)               # the aid's centre in its own frame
