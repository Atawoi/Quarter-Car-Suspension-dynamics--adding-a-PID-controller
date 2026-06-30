"""
parameters.py

Physical parameters for the quarter-car model.
Represents one front corner of a sports car.

All values are in SI units (kg, N/m, N.s/m) unless noted.
Matches the System Parameters table in the project poster/report.
"""

# ── Masses ────────────────────────────────────────────────────
ms = 300        # sprung mass, kg   — 1/4 car body mass
mu = 30         # unsprung mass, kg — wheel + suspension assembly

# ── Stiffness ─────────────────────────────────────────────────
ks = 15000      # suspension spring stiffness, N/m
kt = 200000     # tire stiffness, N/m (~13x stiffer than the
                # suspension spring — this is why the wheel follows
                # the road closely while the body does not)

# ── Damping ───────────────────────────────────────────────────
b = 1500        # passive damper coefficient, N.s/m

# ── Road input ────────────────────────────────────────────────
bump_height   = 0.040   # m  (40 mm half-sine bump)
bump_start    = 0.5     # s  (time bump begins)
bump_duration = 0.08    # s  (bump width)

# ── Derived passive system properties ────────────────────────
# wn   = sqrt(ks / ms)               natural frequency, rad/s
# zeta = b / (2 * sqrt(ks * ms))     damping ratio (dimensionless)
#
# wn   = sqrt(15000/300) = 7.07 rad/s  (≈ 1.13 Hz)
# zeta = 1500 / (2*sqrt(15000*300)) = 0.35  (underdamped)
#
# A damping ratio below 1.0 means the system oscillates after a
# disturbance before settling. 0.35 is quite underdamped — this is
# the baseline problem the PID controller is designed to address.
