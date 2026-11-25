"""Dimensions (m, kg) and positions, shared by build_assets.py, task.py and demo.py.

The pen is a one-piece-barrel click ballpoint (of the Clip Click type, sizes measured from a pen of this design): a clear barrel with a fixed nose, a
clicker unit that latches into the barrel's back end, a 3 mm refill and a loose spring that sits in the nose.

Station frame: x forward from the midpoint between the arm bases, y to the robot's left, z up; TOP is the table top.
Each part's own frame has x along its axis:
- barrel: origin at the nose tip, x toward the open back end (up while it stands nose-down);
- refill: origin at the writing point, x toward its back end;
- clicker: origin at its lower (inner) end, x toward the button;
- spring: origin at its centre.
"""

from __future__ import annotations

TOP = 0.7475

# -- the refill: brass point, 3 mm plastic ink tube, two small bumps (the spring stop) 22 mm from the point ----------
REFILL_L = 0.103
POINT_L = 0.006               # brass outside the tube
POINT_TAPER = 0.002           # the point's taper from the ball (0.8 mm) to the brass's 2 mm
POINT_R0, BRASS_R = 0.0004, 0.0010
TUBE_R = 0.0015               # ink tube 3 mm across
STOP_AT = 0.022               # the spring stop's front face, from the point
STOP_L, STOP_R = 0.001, 0.0025   # a 5 mm collar stands in for the real refill's two small bumps
END_TAPER, END_R = 0.003, 0.0006   # the back end's plug rounds off to 1.2 mm over its last 3 mm
REFILL_MASS = 0.002

# -- the spring: a steel coil, loose in the barrel's nose; in the pen it sits round the refill's front, against the stop
# Real: 30 mm long, 4.0 mm outside, about 3.3 mm inside. Its bore here is 3.5 mm (0.2 mm wider): rigid simulated contacts
# need the clearance to slide on the 3 mm tube.
SPRING_L = 0.030
SPRING_RI, SPRING_RO = 0.00175, 0.0026     # 5.2 mm outside here (real 4.0): it stays near the 6.5 mm bore's axis
SPRING_SEGMENTS = 10
SPRING_LEAD = (0.0008, 0.003)             # a funnel at each end: the bore opens 1.6 mm wider over the last 3 mm (it catches the refill's point) ...
SPRING_ROUND = 0.0                        # ... and the outside doesn't round in
SPRING_MASS = 0.0004

# -- the barrel: one clear piece, 12.7 cm; outside 10 mm at the back, 8 mm from the middle down, a 1.5 cm nose cone ---
BARREL_L = 0.127
NOSE_L = 0.015                # the nose cone: outside 2.5 -> 8 mm, inside 1 mm (the nose hole) -> 6.5 mm
NOSE_TIP_RO, NOSE_HOLE_R = 0.00125, 0.0005
NOSE_SOLID = 0.0125           # the nose is solid plastic up to here round its 1 mm point hole (the spring lands on it)
FRONT_RO, FRONT_RI = 0.0040, 0.00325      # 8 mm outside, 6.5 mm bore from the nose cone up to a funnel at the back end
BACK_START = 0.077            # the back 5 cm: outside widening to 10 mm
WIDE_AT = 0.1233              # (the real pen's last 1.7 cm is 7 mm across; here the bore stays 6.5 mm)
BACK_RO, BACK_RI = 0.0050, 0.00325         # the bore stays 6.5 mm to the back end (no step to catch the spring)
STEP_L = 0.002                # (where the real bore steps out to 7 mm; here it stays 6.5 mm)
MOUTH_CH = 0.0015             # the back end's moulded chamfer
CLIP_FROM, CLIP_W, CLIP_T = 0.089, 0.003, 0.0012   # the pocket clip: 3.8 cm long, on the barrel's +y side
BARREL_SEGMENTS = 16
BARREL_MASS = 0.004

# -- the clicker unit: 5 cm; button cap 1 cm, two guide rings, a hollow body that takes the refill's back end ---------
CLICKER_L = 0.050
CAP_L, CAP_R = 0.010, 0.0025
BODY_R = 0.0028               # 5.6 mm, in the 6.5 mm bore: 0.45 mm a side
RING_R, RING_L = 0.0029, 0.001            # guide rings at the cap's foot and 1 cm below: 0.35 mm a side (real: a close fit)
HOLLOW_R, MOUTH_R, MOUTH_L = 0.0020, 0.0028, 0.004   # the refill's 3 mm end goes into it, through a funnel
HOOK_AT, HOOK_L = 0.010, 0.012            # the click hook on the +y side, 1.2 cm long (faces the clip's notches)
HOOK_R = 0.0029               # (the real hook springs out into the notch; here it stays inside the bore)
CLICKER_MASS = 0.001

# -- the assembled pen (unclicked) -------------------------------------------------------------------------------------
# The spring sits in the nose cone where its outside meets the cone; the refill's stop rests on the spring's top; the
# clicker rests on the refill's back end with its button 8 mm out of the barrel (2 mm when clicked).
SPRING_SEAT = NOSE_SOLID     # the spring's lower end, from the tip: on the nose's flat inner seat (no wedging in the cone)
SPRING_TOP = SPRING_SEAT + SPRING_L
POINT_IN = SPRING_TOP - STOP_AT          # the refill's point, from the nose tip
REFILL_END = POINT_IN + REFILL_L
CLICKER_OUT = 0.008                      # the button out of the barrel, unclicked
CLICKER_IN_AT = BARREL_L + CLICKER_OUT - CLICKER_L    # the clicker's lower end, from the nose tip
HOLLOW_L = REFILL_END - CLICKER_IN_AT    # the refill's end bears on the hollow's floor

# -- the desk pen stand: a slab with three tubes in a row along y ----------------------------------------------------
HOLDER_AT = (0.25, -0.02)     # the base's centre
BASE_X, BASE_Y = 0.03, 0.16
SOCKET_Y = {"clicker": 0.06, "barrel": 0.0, "refill": -0.06}   # along y from HOLDER_AT, 6 cm apart
SOCKET_RI = {"clicker": 0.0038, "barrel": 0.00415, "refill": 0.0026}
SOCKET_RO = {"clicker": 0.0058, "barrel": 0.0065, "refill": 0.0046}
SLAB_T = 0.004
SOCKET_H = {"clicker": 0.020, "barrel": 0.040, "refill": 0.030}   # tube heights above the slab
SPRING_XY = (0.20, -0.14)     # the spring stands on end on the table, right of the stand

# -- scoring ----------------------------------------------------------------------------------------------------------
SPRING_OFF = 0.00325          # spring in the nose: its centre within the 6.5 mm bore ...
SPRING_NOSE = 0.050           # ... and its centre within this of the nose tip
SPRING_ON_OFF = 0.0012        # or on the refill: within this of the refill's axis, centre at most 22 mm from the point
POINT_OFF = 0.0015            # refill in: its point within this of the barrel's axis ...
POINT_RANGE = (0.008, 0.030)  # ... this far from the nose tip (through the spring; 20 mm when it rests on it)
AXIS_DEG = 6.0
CLICKER_RANGE = (0.006, 0.010)            # button out of the barrel (8 mm when latched)
CLICKER_OFF = 0.0015
HOOK_DEG = 20.0               # the clicker's hook turned within this of the clip's side (the notches)
CLEAR = 0.04                  # both hands' tool points this far from every pen part at the end
STILL = 0.01                  # m/s
