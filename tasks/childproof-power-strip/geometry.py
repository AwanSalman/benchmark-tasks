"""Dimensions (m, kg) and positions of the scene, shared by build_assets.py, task.py and demo.py.

Station frame: x forward from the midpoint between the arm bases, y to the robot's left, z up; TOP is the table top.
"""

from __future__ import annotations

TOP = 0.7475

# -- the power strip: an upright tower of four NEMA 5-15 outlets on its front face, which faces the robot ------------
# Its frame: origin at the centre of its bottom, x away from the robot (into the outlets), z up.
STRIP_X, STRIP_Y = 0.43, -0.06
STRIP_D, STRIP_W, STRIP_H = 0.060, 0.060, 0.300          # depth (x), width (y), height
STRIP_MASS = 0.50
STRIP_FRICTION = 0.6                                     # its rubber feet on the table
FOOT_H = 0.003                                           # four rubber feet, 12 mm square, under its corners
FRONT = -STRIP_D / 2                                     # the front face, strip frame
OUTLET_Z = (0.065, 0.120, 0.175, 0.230)                  # outlet centres above the table (55 mm apart)
OUTLET_FACE = (0.030, 0.036)                             # each outlet's raised face (y, z), visual
BRACE_Z = 0.280                                          # where the other hand holds the strip, above the outlets
PLUG_OUTLET = 0                                          # the lamp is plugged into the bottom outlet
SLOT_DY = 0.00635                                        # slot centres +-12.7 mm / 2 from the outlet's centre
SLOT_W, SLOT_H = 0.0035, 0.0090                          # slot openings (y, z): 1 mm round a 1.5 mm blade
FACE_T = 0.003                                           # the face plate's thickness at the slots
SLOT_DEPTH = 0.019                                       # from the face to the slot's floor
# the receptacle's contacts: in each slot a pair of sprung leaves, one each side, pinch the blade between them
LEAF_X0, LEAF_X1 = 0.0040, 0.0170                        # along the slot, from the face
LEAF_LEAD = (0.004, 0.0012)                              # their lead-in ramps: 1.2 mm over the first 4 mm (17 deg)
LEAF_T = 0.0030
LEAF_GAP = 0.0008                                        # at rest they leave 0.8 mm between them (< the blade)
LEAF_TRAVEL = 0.0022
CAV_HALF = LEAF_GAP / 2 + LEAF_T + LEAF_TRAVEL           # behind the face each slot widens into a cavity this wide
LEAF_K = 1200.0                                          # N/m
LEAF_PRELOAD = 0.0030                                    # its spring target lies 3 mm past its stop (4 N on a blade)
LEAF_MASS = 0.005
CONTACT_FRICTION = 0.2                                   # smooth brass contacts and slot walls on the blades

# -- the safety caps: a round face, a grip ridge on its front and two flat blades on its back -------------------------
# A cap's frame: origin at the centre of its back face (the face that meets the outlet), x along the blades.
CAP_R, CAP_T = 0.019, 0.003
RIDGE_L, RIDGE_T, RIDGE_H = 0.024, 0.006, 0.014          # along y, across (z when plugged in), out from the face
BLADE_L, BLADE_T, BLADE_W = 0.012, 0.0015, 0.0063        # length (x), thickness (y), width (z)
BLADE_TIP = 0.003                                        # the blades' chamfered tips
CAP_MASS = 0.006
CAP_FRICTION = 1.0
CAPS = 3

# -- the tray the caps start in: a foam block with slits, each cap blades down, ridge up ---------------------------
TRAY_X, TRAY_Y = 0.26, -0.215
TRAY_SIZE = (0.070, 0.170, 0.020)
CAP_DY = (-0.055, 0.0, 0.055)                            # the caps along the tray
SLIT_W, SLIT_L, SLIT_DEPTH = 0.0035, 0.0090, 0.015

# -- the lamp, its cord and plug -------------------------------------------------------------------------------------
LAMP_X, LAMP_Y = 0.66, 0.15
LAMP_BASE_R, LAMP_BASE_T = 0.055, 0.020
CORD_R = 0.0035
CORD_LINK = 0.030
CORD_K, CORD_DAMPING = 0.002, 0.003                      # N m/rad, N m s/rad (Isaac Lab actuator units)
CORD_LINK_MASS = 0.006
PLUG_D, PLUG_W, PLUG_H = 0.026, 0.022, 0.034             # the plug's body: depth (x), width, height
PLUG_BLADE_L = 0.0159
PLUG_MASS = 0.030

# -- scoring ------------------------------------------------------------------------------------------------------
SEATED = 0.0020          # a cap's back face within 2 mm of the strip's face (10 of its 12 mm blades in)
CENTRED = 0.0025         # its centre within 2.5 mm of the outlet's centre
SQUARE = 5.0             # deg, its axis to the outlet's
PLUG_IN = 0.003          # the lamp plug no more than 3 mm out
PLUG_OUT = 0.005         # more than 5 mm out: pulled
STRIP_MOVED = 0.010      # the strip's base moved more than 10 mm
STRIP_TILT = 5.0         # deg
LET_GO = 0.02            # every finger pad 2 cm clear of the caps, the plug and the strip
STILL = 0.002            # the caps and strip moved less than 2 mm over the last second
