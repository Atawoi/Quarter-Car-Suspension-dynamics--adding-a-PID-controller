"""
passive_model.py

Passive quarter-car equations of motion, derived from Newton's
second law applied separately to the sprung mass (body) and the
unsprung mass (wheel).

Free body diagram — sprung mass (body):
    Only the suspension spring and damper act on it.
    ms * zs_ddot = -ks*(zs - zu) - b*(zs_dot - zu_dot)

Free body diagram — unsprung mass (wheel):
    Suspension spring/damper act from above (opposite sign to body,
    by Newton's third law), tire spring acts from below connecting
    to the road.
    mu * zu_ddot = +ks*(zs - zu) + b*(zs_dot - zu_dot) - kt*(zu - zr)

State vector: y = [zs, zs_dot, zu, zu_dot]
    y[0] = zs      body displacement, m
    y[1] = zs_dot  body velocity, m/s
    y[2] = zu      wheel displacement, m
    y[3] = zu_dot  wheel velocity, m/s
"""

from parameters import ms, mu, ks, b, kt
from road_input import road_bump


def passive(t, y):
    """
    Equations of motion for the passive quarter-car system.
    Returns derivatives of the state vector for use with
    scipy.integrate.solve_ivp.
    """
    zs, zs_dot, zu, zu_dot = y
    zr = road_bump(t)

    # Relative displacement and velocity across the suspension
    rel_disp = zs - zu          # suspension compression, m
    rel_vel  = zs_dot - zu_dot  # rate of compression, m/s

    # Body:  ms*zs_ddot = -ks*(zs-zu) - b*(zs_dot-zu_dot)
    zs_ddot = (-ks * rel_disp - b * rel_vel) / ms

    # Wheel: mu*zu_ddot = +ks*(zs-zu) + b*(zs_dot-zu_dot) - kt*(zu-zr)
    zu_ddot = (ks * rel_disp + b * rel_vel - kt * (zu - zr)) / mu

    return [zs_dot, zs_ddot, zu_dot, zu_ddot]
