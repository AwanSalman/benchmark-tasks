"""Dimensions and positions for the kettle limescale-filter task. Metres, kilograms, station frame (x forward from the
arm bases, y to the robot's left, z up). Pure Python.

The kettle is a KitchenAid 1.25 L electric kettle (KEK1222; EU 5KEK1222): 26.2 cm tall, 17.8 cm wide, 22.6 cm deep,
1.14 kg (KitchenAid's spec sheet). Its limescale filter is a curved mesh plate that slides down a slot behind the
spout, "ensuring that the curve facing outward on the filter matches the curve of the electric kettle" (KitchenAid).
Sizes KitchenAid does not publish (wall, opening, slot, filter) are estimates from product photos.

Kettle frame: origin on the table under the kettle's axis, +x toward the spout. Filter frame: origin at the middle of
the plate's bottom edge on its inner face, +x outward through the plate (toward the spout when seated), z up.
"""

from __future__ import annotations

import math

TOP = 0.7475                      # station table top

# -- the kettle ----------------------------------------------------------------------------------------------------
KETTLE_XY = (0.218, 0.050)        # spout (+x) away from the robot; the filter's seat ends up near x 0.30
KETTLE_MASS = 1.14                # KitchenAid
KETTLE_FRICTION = (0.6, 0.5)      # rubber feet on the bench (estimate)
HEIGHT_PUBLISHED = 0.262          # KitchenAid, to the top of the closed lid's knob
R_OUT, WALL = 0.085, 0.003        # body 17 cm across (the published 17.8 cm width includes the spout's lip)
R_IN = R_OUT - WALL
H = 0.230                         # rim of the lid opening (the lid and its knob on top bring it to 262 mm)
FLOOR = 0.010
OPEN_C, OPEN_R = (0.034, 0.0), 0.052   # lid opening (circle), pushed toward the spout
TOP_T = 0.004

# the spout hole through the wall, in front of the filter
HOLE_HALF_W, HOLE_Z = 0.013, (0.180, 0.215)

# the filter's slot: the filter's outer face rests on the wall's inside; two guide lips (one each side) hold its side
# edges against the wall, with a side wall beyond each edge, and a stop ledge under it
F_W, F_H, F_T = 0.034, 0.050, 0.0015   # filter: chord width, height, thickness
SLOT_PLAY = 0.0010                # lip to the filter's inner face (radial play)
LIP_T = 0.0030
LIP_OVER = 0.0025                 # lip overlaps the filter's edge by this much
SIDE_PLAY = 0.0008                # filter edge to side wall, each side
GUIDE_Z = (H - 0.060, H - 0.010)  # stop ledge top .. lip top (the seated filter's top is level with the lips' top)
LEAD = 0.003                      # top 3 mm of the lips and side walls flare open, by 1 mm (a moulded lead-in)
FLARE = 0.0010

# the lid lifts off (KitchenAid: "Removable lid: yes", aluminium). It starts on the kettle: a disc resting on the top,
# a locating plug under it dropping into the opening (it centres the lid), and a knob on top
LID_R, LID_T = 0.056, 0.006       # covers the opening with 4 mm to spare
PLUG_R, PLUG_D = 0.046, 0.006     # 6 mm play in the opening; its edge stays 2 mm inside the wall behind the spout
DOME_X = 0.020                    # the lid's front (spout side, lid x > 20 mm) is raised ...
DOME_UP = 0.016                   # ... by 16 mm, clearing the seated filter's grip tab (12 mm over the rim)
KNOB_R, KNOB_H = 0.012, 0.026     # rim 230 + lid 6 + knob 26 = the published 262 mm
LID_MASS = 0.090                  # aluminium (estimate)
LID_ON_XY = 0.006                 # lid on: its centre within 6 mm of the opening's centre ...
LID_ON_Z = 0.002                  # ... resting on the top (within 2 mm) ...
LID_ON_DEG = 5.0                  # ... and level within 5 degrees
# the kettle's power base (the kettle stands off it), on the bench to the robot's right
BASE_R, BASE_H, BASE_MASS = 0.080, 0.022, 0.35   # estimates from product photos
BASE_XY = (0.340, -0.240)
# the handle, at the back
HANDLE_X = (-R_OUT - 0.040, -R_OUT - 0.020)
HANDLE_HALF_Y = 0.014
HANDLE_Z = (0.050, 0.215)

# -- the filter ----------------------------------------------------------------------------------------------------
F_SEGMENTS = 6                    # the curved plate as 6 flat strips
FIN_L, FIN_T, FIN_Z = 0.020, 0.0015, (0.035, 0.072)   # grip tab on the inner face: depth (inward), thickness, z span
F_MASS = 0.004
F_FRICTION = (0.35, 0.30)         # polypropylene frame (estimate)
FILTER_XY = (0.190, 0.220)        # where it lies on the bench, outer face down, grip tab up (tab toward +x)

# -- scoring -------------------------------------------------------------------------------------------------------
SEATED_Z = 0.0015                 # on its stop within 1.5 mm
UPRIGHT_DEG = 6.0                 # seated: within 6 degrees of its seated orientation
LEAN_DEG = 15.0                   # in its slot, not down: within 15 degrees (it may lean in the slot's play)
IN_SLOT_Z = GUIDE_Z[1] - GUIDE_Z[0] - 0.005   # in its slot: bottom edge at least 5 mm below the lips' top
CLEAR = 0.040                     # judged only once both hands' tool points are this far from the filter and lid
STILL = 0.01                      # ... and the filter and lid move slower than this (m/s)
TIP_DEG = 20.0                    # kettle ever tipped past this: 0


def half_angle():
    """Half the angle the filter's chord spans on the wall's inside."""
    return math.asin(F_W / 2 / R_IN)


def seat_origin_k():
    """The seated filter's origin in the kettle frame (inner face, middle of the bottom edge)."""
    return (R_IN - F_T, 0.0, GUIDE_Z[0])


def lid_on_k():
    """The lid's origin (bottom face of the disc, on its axis) when on: over the opening's centre, resting on the top."""
    return (OPEN_C[0], OPEN_C[1], H)
