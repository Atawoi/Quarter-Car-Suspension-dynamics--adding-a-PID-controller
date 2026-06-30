"""
metrics.py

Ride-comfort metrics computed from a simulated displacement time
history. Peak displacement alone understates a controller's value —
these additional metrics (RMS displacement, settling time, peak
acceleration) give a fuller picture, matching the comparison table
in the project report.
"""

import numpy as np


def peak_displacement(z):
    """Maximum absolute displacement over the simulation, same
    units as the input array."""
    return np.max(np.abs(z))


def rms_displacement(z):
    """Root-mean-square displacement — a standard ride-comfort
    metric (lower RMS = smoother average ride, less cumulative
    jostling, not just a lower single peak)."""
    return np.sqrt(np.mean(np.array(z) ** 2))


def peak_acceleration_from_states(t, zs, zu, ks, b, ms, u=None):
    """
    Computes body acceleration directly from the equation of motion
    rather than by finite-differencing the displacement signal.
    A second derivative taken by finite difference amplifies any
    numerical noise in the position data; recomputing zs_ddot from
    the same physics used to generate the trajectory is far more
    accurate and is the recommended approach here.

        zs_ddot = (-ks*(zs-zu) - b*(zs_dot-zu_dot) - u) / ms

    Parameters
    ----------
    t  : array of time values, s
    zs : array of body displacement, m
    zu : array of wheel displacement, m
    ks, b, ms : suspension parameters
    u  : array of controller force at each time step, N
         (pass zeros, or None, for the passive case)

    Returns
    -------
    float : peak absolute body acceleration, m/s^2
    """
    zs = np.asarray(zs)
    zu = np.asarray(zu)
    zs_dot = np.gradient(zs, t)
    zu_dot = np.gradient(zu, t)

    if u is None:
        u = np.zeros_like(zs)

    accel = (-ks * (zs - zu) - b * (zs_dot - zu_dot) - u) / ms
    return np.max(np.abs(accel))


def settling_time(t, z, threshold=0.02, reference=None):
    """
    Time after which the response stays within +/- threshold of
    its reference value (default: 2% of peak displacement, matching
    the "Settling time (+/-2%)" metric in the report).

    Parameters
    ----------
    t         : array of time values, s
    z         : array of displacement values, same length as t
    threshold : fraction of the reference value used as the
                settling band (default 0.02 = 2%)
    reference : value to settle around — defaults to peak |z| if
                not provided

    Returns
    -------
    float : time at which the response has settled and stays
            within the band for the remainder of the simulation, s
    """
    z = np.asarray(z)
    if reference is None:
        reference = np.max(np.abs(z))
    band = threshold * reference

    # walk backward from the end to find the last time the response
    # was outside the band — settling time is one step after that
    for i in range(len(z) - 1, -1, -1):
        if abs(z[i]) > band:
            return t[min(i + 1, len(t) - 1)]
    return t[0]
