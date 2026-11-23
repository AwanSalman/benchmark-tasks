"""Dimensions (m, kg) and positions, shared by build_assets.py, task.py and demo.py.

Station frame: x forward from the midpoint between the arm bases, y to the robot's left, z up; TOP is the table top.
A refill's own frame: origin at its writing point, x along it toward its back end. The spring's: origin at its centre,
x along its axis.
"""

from __future__ import annotations

TOP = 0.7475

# -- a retractable ballpoint pen's refill: a brass point, a narrow front section and the ink tube ---------------------
REFILL_L = 0.115
POINT_L = 0.005               # the point's taper, from 0.8 mm across to the front section
POINT_R0 = 0.0004
FRONT_R = 0.0010              # the front section: 2.0 mm across, up to the shoulder
SHOULDER = 0.018              # from the point
TUBE_R = 0.0025               # the ink tube: 5 mm across
REFILL_MASS = 0.003

# -- the pen's spring: a steel coil that rides on the front section, against the shoulder ----------------------------
# Its bore is 1 mm wider than a real pen spring's: rigid simulated contacts need the clearance to slide.
SPRING_L = 0.012
SPRING_RI, SPRING_RO = 0.0020, 0.0025     # bore 4.0 mm round the 2 mm front section, outside 5.0 mm
SPRING_SEGMENTS = 10
SPRING_LEAD = (0.0003, 0.0005)            # the round wire of its end turns: the bore opens 0.4 mm over the last 0.6 mm
SPRING_MASS = 0.0005


# -- the pen's front barrel: a hollow tube whose bore narrows in a cone to the nose bore ------------------------------
# Its own frame: origin at the nose tip, x up the barrel (toward its open back end). Held tip-down, x points down.
BARREL_RO = 0.0050            # 10 mm across outside
BARREL_RI = 0.0035            # bore 7 mm: the 5 mm ink tube has 1 mm a side
BARREL_L = 0.060
NOSE_L = 0.008                # the nose bore from the tip: 3 mm across, the 2 mm front section has 0.5 mm a side
NOSE_RI = 0.0015
NOSE_TIP_RO = 0.0025          # the outside tapers to 5 mm at the tip
BARREL_SEGMENTS = 16
BARREL_MASS = 0.006
MOUTH_CH = 0.0007             # the open end's moulded chamfer: the bore opens 0.7 mm over its last 0.7 mm
SEAT_RI = 0.0035              # the full 7 mm bore down to the cone (the 5 mm spring has 1 mm a side)
SEAT_STEP = 0.0020            # the 45 deg cone from the nose bore up to the bore: the spring's end rests on it, centred
SEAT_L = 0.008                # straight bore above the cone
FUNNEL_L = 0.001              # (no funnel: the seat is the full bore)

# -- the pen's back half: a tube closed at its button end; its open end has a sleeve the front barrel presses into ---
# Its own frame: origin at the closed (button) end, x along it toward the open end.
BACK_L = 0.070
BACK_RO = 0.0070              # 14 mm across
BACK_RI = 0.0035              # bore 7 mm, like the front barrel's
SLEEVE_RI = 0.0058            # the open end's sleeve: the 10 mm front barrel has 0.8 mm a side (a press fit, simplified)
SLEEVE_L = 0.008
SLEEVE_CH = 0.0012             # the sleeve's moulded chamfer: it opens 1.2 mm over its last 1.5 mm
BACK_FLOOR = 0.003
BACK_MASS = 0.007

# -- the desk pen stand: a slab with three sockets (refill, front barrel, back half) ---------------------------------
HOLDER_AT = (0.30, -0.04)     # the slab's centre; the sockets 0.02 to its left, under it, and 0.02 / 0.06 to its right
SOCKET_Y = {"refill": 0.02, "barrel": -0.02, "back": -0.06}   # along y from HOLDER_AT
SLAB_T = 0.004
SOCKET_H = 0.030              # above the slab
SPRING_XY = (0.24, 0.06)      # the spring stands on end on the table, left of the stand

# -- scoring ----------------------------------------------------------------------------------------------------------
SPRING_OFF = 0.0012           # the spring's centre within this of the refill's axis ...
NOSE_OFF = 0.0015             # the refill's point within this of the barrel's axis, 1 mm above to 8 mm below its nose tip
NOSE_ABOVE, NOSE_BELOW = 0.001, 0.008
JOIN_GAP = 0.0015             # the front barrel's open end within this of the back half's sleeve floor
JOIN_OFF = 0.0015
AXIS_DEG = 6.0                # front barrel and back half in line within this
CLEAR = 0.04                  # both hands' tool points this far from every pen part at the end
STILL = 0.01                  # m/s
