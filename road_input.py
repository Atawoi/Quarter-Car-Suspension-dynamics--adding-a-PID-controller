"""
road_input.py

Defines the road profile that excites the quarter-car model.
A single half-sine bump is used as the test input — representative
of a pothole edge or speed bump.
"""

import numpy as np
from parameters import bump_height, bump_start, bump_duration


def road_bump(t, height=bump_height, start=bump_start, width=bump_duration):
    """
    Returns road surface height z_r at time t.

    Parameters
    ----------
    t      : float, current simulation time, s
    height : float, bump height, m (default 0.04 m = 40 mm)
    start  : float, time the bump begins, s
    width  : float, duration of the bump, s

    Returns
    -------
    float : road height at time t, m. Zero everywhere except during
            the bump window, where it follows a smooth half-sine
            profile (no sharp edges — physically realistic).
    """
    if start <= t <= start + width:
        return height * np.sin(np.pi * (t - start) / width)
    return 0.0
