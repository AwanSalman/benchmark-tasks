"""Dimensions and positions for the coin-holder task. Metres, kilograms, station frame (x forward from the arm bases,
y to the robot's left, z up). Pure Python."""

from __future__ import annotations

import math

TOP = 0.7475                      # station table top

# -- the coins (US Mint figures) --------------------------------------------------------------------------------------
# name: (diameter, thickness, mass, colour, value in cents)
SILVER, COPPER = (0.70, 0.71, 0.73), (0.72, 0.42, 0.26)
COINS = {
    "quarter": (0.02426, 0.00175, 0.00567, SILVER, 25),
    "dime": (0.01791, 0.00135, 0.002268, SILVER, 10),
    "nickel": (0.02121, 0.00195, 0.00500, SILVER, 5),
    "penny": (0.01905, 0.00152, 0.00250, COPPER, 1),
}
STACK = {"quarter": 4, "dime": 2, "nickel": 2, "penny": 2}     # coins in each tube
COIN_SIDES = 32                   # collider: 32-sided prism with the disc's area (64 corners, PhysX's hull limit)
# Coins combine friction with "max": a fingertip on a coin uses the fingertip's value, coin on coin this one
# (cupronickel and copper on each other, dry; estimate).
COIN_FRICTION = (0.25, 0.22)
STACK_GAP = 0.00005               # between stacked coins at spawn


def coin_ids(c):
    """Tube c's coins, the top one first."""
    return [f"{c}{i}" for i in range(STACK[c])]


ALL_COINS = [k for c in COINS for k in coin_ids(c)]


def kind(coin_id):
    return coin_id.rstrip("0123456789")


def value(coin_id):
    return COINS[kind(coin_id)][4]


# -- the holder -------------------------------------------------------------------------------------------------------
# A round car cup-holder coin holder, 76.2 mm across and 101.6 mm tall (as listed for the Seven Sparta A-1231), black
# ABS, with four spring-loaded tubes in a 2 x 2 cluster. Holder frame: origin on its underside under its axis.
HOLDER_XY = (0.300, 0.000)
HOLDER_R = 0.0381
HOLDER_H = 0.1016
HOLDER_MASS = 0.175
HOLDER_FRICTION = (0.6, 0.5)      # rubber base (estimate)
SLICK = (0.25, 0.22)              # moulded ABS bores, lips and channels (estimate)
PLATE_T = 0.0015                  # top plate (the lips) thickness
TUBE_DEPTH = 0.030                # bores below the plate's underside
WALL = 0.0025
FLOOR_T = 0.003
GUIDE_T = 0.002

TUBES = {                         # tube centres, holder frame
    "quarter": (-0.015, -0.0135),
    "dime": (-0.015, 0.0135),
    "nickel": (0.015, 0.0135),
    "penny": (0.015, -0.0135),
}
LIP = 0.0016                      # each lip overhangs its coin's rim by 1.6 mm (estimate)

FOLLOWER_T = 0.002
FOLLOWER_MASS = 0.002
FOLLOWER_K = 30.0                 # N/m: a long soft spring, nearly constant force over the stack
FOLLOWER_FULL = 0.8               # N up on a full stack (about 0.6 N on the last coin)
FOLLOWER_DAMPING = 10.0           # N s/m: a stack rises gently (at most about 8 cm/s) as its top coin leaves


def plate_under():
    return HOLDER_H - PLATE_T


def tube_dims(c):
    """Tube c: bore radius, lip opening radius, slot width, where the mouth opens (from the tube's centre along its
    exit), exit channel height and width."""
    d, t = COINS[c][0], COINS[c][1]
    return dict(bore_r=d / 2 + 0.0004, hole_r=d / 2 - LIP, slot_w=d - 2 * LIP, mouth_s=0.29 * d / 2,
                channel_h=t + 0.0006, channel_w=d + 0.0010)


def exit_dir(c):
    """Unit vector (holder frame, xy) from the holder's axis through tube c: its coins leave that way, over the rim."""
    x, y = TUBES[c]
    n = math.hypot(x, y)
    return (x / n, y / n)


def f_low(c):
    """The follower joint is authored under the full stack (0 there); its top stop is at -f_low(c)."""
    return -(STACK[c] * (COINS[c][1] + STACK_GAP)) + 0.0003


# -- the tray it stands in --------------------------------------------------------------------------------------------
TRAY_R, TRAY_H, TRAY_WALL, TRAY_FLOOR = 0.085, 0.008, 0.0025, 0.0025
TRAY_MASS = 0.150
TRAY_FRICTION = (0.5, 0.45)

# -- scoring ----------------------------------------------------------------------------------------------------------
TARGET_CENTS = 40
NEEDED = ("quarter", "dime", "nickel")   # with these stacks the only way to make exactly 40 cents
IN_TUBE = 0.003                   # a coin is still in its tube while its centre is this close to the tube's axis
TIP_DEG = 20.0                    # holder ever tilted past this: 0
CLEAR = 0.04                      # both hands' tool points this far from every coin out of its tube
STILL = 0.01                      # m/s
