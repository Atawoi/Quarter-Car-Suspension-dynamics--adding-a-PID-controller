"""
pid_model.py

PID-controlled active quarter-car equations of motion.

GAIN DERIVATION (analytical, not trial-and-error)
--------------------------------------------------
Step 1 — Passive natural frequency:
    wn = sqrt(ks/ms) = sqrt(15000/300) = 7.07 rad/s

Step 2 — Passive damping ratio:
    zeta = b / (2*sqrt(ks*ms))
         = 1500 / (2*sqrt(15000*300))
         = 0.35   --> underdamped, body bounces after a bump

Step 3 — Target damping ratio for comfortable, non-oscillatory
         return to equilibrium: zeta_target = 0.70

Step 4 — Required total damping to hit that ratio (at unchanged wn,
         i.e. without adding extra stiffness):
    b_effective = 2 * zeta_target * sqrt(ks*ms)
                = 2 * 0.70 * sqrt(15000*300)
                = 2969 N.s/m

Step 5 — Kd is the additional damping needed:
    Kd = b_effective - b = 2969 - 1500 = 1469 ≈ 1500 N.s/m

Step 6 — Kp = 0.
    Kp would add stiffness (k_eff = ks + Kp), which raises the
    natural frequency wn. Raising wn while leaving damping unchanged
    *lowers* the damping ratio (zeta = b / (2*sqrt(k_eff*ms))),
    making the system MORE underdamped and the response WORSE.
    Verified experimentally: every nonzero Kp tested increased
    peak body displacement rather than reducing it.

Step 7 — Ki = 0.
    Ki eliminates steady-state error — useful when a system settles
    permanently offset from its target (e.g. cruise control on a
    hill). A road bump is a transient input: the spring naturally
    returns zs to zero with no persistent offset, so there is no
    steady-state error for Ki to correct. Including a nonzero Ki
    causes integral windup — the accumulated error from the bump
    discharges as a corrective force *after* the bump has already
    passed, actively degrading the transient response. This was
    confirmed experimentally (nonzero Ki produced -60% to -86%
    "improvement", i.e. made the response worse).

CONTROL LAW
-----------
    error      = zs           (body displacement from rest —
                                what the driver actually feels)
    error_dot  = zs_dot
    u(t)       = Kp*error + Kd*error_dot + Ki*integral(error)

The controller force is subtracted from the body equation and added
to the wheel equation (equal and opposite, consistent with the
spring/damper sign convention). Physically, the actuator sits between
body and wheel; in a real implementation this maps to an MR-damper
modifying b in real time rather than injecting a separate force, but
the net dynamic effect is equivalent for this analysis.
"""

from parameters import ms, mu, ks, b, kt

# ── Analytically derived gains (see docstring above) ───────────
Kp = 0       # no added stiffness — see Step 6
Kd = 1500    # doubles effective damping, targets zeta ≈ 0.70
Ki = 0       # not needed for a transient bump input — see Step 7


def active_pid(t, y, road_func):
    """
    Equations of motion for the PID-controlled active quarter-car
    system. State vector extended with an integral-error term.

    State vector: y = [zs, zs_dot, zu, zu_dot, int_error]
        y[0] = zs         body displacement, m
        y[1] = zs_dot      body velocity, m/s
        y[2] = zu          wheel displacement, m
        y[3] = zu_dot      wheel velocity, m/s
        y[4] = int_error   running integral of body displacement

    Parameters
    ----------
    road_func : callable, road_bump(t) — passed in explicitly so
                this module has no circular import on road_input.
    """
    zs, zs_dot, zu, zu_dot, int_error = y
    zr = road_func(t)

    rel_disp = zs - zu
    rel_vel  = zs_dot - zu_dot

    # Error signal = body displacement from rest (what we minimize)
    error     = zs
    error_dot = zs_dot

    # PID control force
    u = Kp * error + Kd * error_dot + Ki * int_error

    # Body: passive forces + controller force pushing body to rest
    zs_ddot = (-ks * rel_disp - b * rel_vel - u) / ms

    # Wheel: equal and opposite reaction to the controller force
    zu_ddot = (ks * rel_disp + b * rel_vel - kt * (zu - zr) + u) / mu

    int_error_dot = error

    return [zs_dot, zs_ddot, zu_dot, zu_ddot, int_error_dot]
