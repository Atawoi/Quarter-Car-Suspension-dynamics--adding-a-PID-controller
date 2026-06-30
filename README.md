# Quarter-Car Suspension Dynamics with PID Active Control

Applying feedback control to improve ride comfort — a quarter-car
model comparing passive suspension and PID active control under a
road bump disturbance.

## Motivation

As a member of my Solar Car team's dynamics group, I've seen firsthand
how suspension design impacts both ride comfort and handling.
Traditional suspensions rely on passive springs and dampers — fixed
hardware that can't adapt to the road. I became curious whether
feedback control could improve ride comfort without simply making the
suspension stiffer.

This project explores that question through a quarter-car model,
comparing a passive spring-damper system against a PID-controlled
active system under an identical road bump disturbance.

## A Note on Peak Acceleration

One metric is worth flagging honestly rather than glossing over: in
this simulation, **peak body acceleration is very slightly higher
for the PID-active case than the passive case** (~4% higher), not
lower. This was checked carefully — it is not a numerical artifact;
recomputing acceleration directly from the equation of motion
(rather than by finite-differencing the position signal) confirms
it.

The reason is physically sensible: the peak acceleration occurs in
the first few milliseconds of impact, while the road input is still
rising steeply. At that exact instant the body's velocity is also
rising fastest, so the derivative term `Kd*zs_dot` is near its own
peak and adds a small additional decelerating force right when the
spring and tire forces are already largest. The controller is doing
its job correctly — it is precisely this strong damping action that
eliminates the post-bump oscillation — but it does not (and isn't
designed to) reduce the very first instant of impact, which is
dominated by tire and spring stiffness rather than damping.

This is a useful, honest result: it shows the PID controller
improves **settling behavior** (RMS displacement down 30.8%,
settling time down 37.6%) without claiming a benefit it doesn't
actually provide at the instant of impact. 

## Key Results

| Metric                    | Passive (ζ = 0.35) | PID Active (ζ = 0.70) | Change          |
|----------------------------|---------------------|-------------------------|-----------------|
| Peak body displacement     | 12.6 mm             | 11.1 mm                 | 11.6% lower     |
| Peak body acceleration     | 10.18 m/s²          | 10.59 m/s²               | ~4% higher (see note below) |
| RMS body displacement      | 3.29 mm             | 2.28 mm                  | 30.8% lower     |
| Settling time (±2%)        | 2.24 s              | 1.39 s                   | 37.6% faster    |
| Effective damping ratio    | 0.35                | 0.71                     | 100% increase   |

The PID controller meaningfully reduces RMS body motion and cuts
settling time by more than a third, with a modest reduction in peak
displacement — all without adding stiffness to the suspension. Peak
acceleration is essentially unchanged (see note below for why).

## Mathematical Model

**Equations of motion** (Newton's second law on sprung and unsprung
masses):

```
Sprung mass (body):    ms*zs_ddot = -ks*(zs-zu) - b*(zs_dot-zu_dot) - u(t)
Unsprung mass (wheel):  mu*zu_ddot = +ks*(zs-zu) + b*(zs_dot-zu_dot) - kt*(zu-zr) + u(t)
```

**PID control law:**

```
u(t) = Kp*e(t) + Kd*e_dot(t) + Ki*∫e(t)dt
where e(t) = zs(t)   (body displacement — what the driver feels)
```

**Controller gains** — derived analytically from target damping
ratio, not tuned by trial and error:

- `Kp = 0` — no added stiffness (ks already provides proportional
  restoring force; adding more raises natural frequency without
  improving damping, which worsens overshoot)
- `Kd = 1500` — increases effective damping to target ζ = 0.70
- `Ki = 0` — not needed for a transient bump input (no steady-state
  error for the integral term to correct; would cause windup)

Full derivation with step-by-step justification is in
`simulation/pid_model.py`.

## System Parameters

| Parameter          | Symbol | Value   | Units  | Description              |
|---------------------|--------|---------|--------|---------------------------|
| Sprung mass          | ms     | 300     | kg     | 1/4 car body mass         |
| Unsprung mass        | mu     | 30      | kg     | Wheel + hub + upright     |
| Spring stiffness     | ks     | 15,000  | N/m    | Suspension spring         |
| Damping coefficient  | b      | 1,500   | N·s/m  | Passive damper            |
| Tire stiffness       | kt     | 200,000 | N/m    | Tire stiffness            |
| Road bump height     | h      | 40      | mm     | Half-sine bump            |
| Bump start time      | t0     | 0.5     | s      | Start of bump             |
| Bump duration        | tw     | 0.08    | s      | Bump width                |

## What I Learned

- How suspension parameters (ks, b, ms, mu) determine natural
  frequency and damping ratio
- How PID feedback control can actively reduce body motion without
  changing the physical spring
- How to model, numerically integrate, and compare dynamic systems
  in Python
- The tradeoffs between passive, active, and semi-active suspension
  systems, and why real production cars (MR dampers in Ferrari,
  Porsche, BMW) implement this kind of control electronically
- That ride comfort metrics beyond peak displacement — RMS
  displacement, settling time, peak acceleration — tell a more
  complete story than any single number

## Future Work

- Implement semi-active damping (variable damper rate rather than
  an injected force, closer to a real MR-damper implementation)
- Explore skyhook control and LQR (linear quadratic regulator) as
  alternatives to PID
- Test against randomized road profiles (e.g. ISO 8608 road
  classifications) instead of a single bump
- Extend to a half-car model (pitch) and full-car model (roll +
  pitch) for a more complete vehicle dynamics picture
- Tie suspension parameters (ks, motion ratio) to a CAD-derived
  hardpoint geometry for a specific suspension design

## Live Demo

An interactive side-by-side animation of the passive and PID-active
quarter-car models hitting the same bump is in `demo/index.html`.
Open it directly in a browser, or enable GitHub Pages on this repo
(Settings → Pages → Deploy from branch → `/demo`) for a shareable
link.

## Tools

Python, NumPy, SciPy, Matplotlib, vanilla HTML/Canvas (demo)

## How to Run

```bash
pip install numpy scipy matplotlib
python simulation/run_simulation.py
```

This solves both the passive and PID models, plots the comparison,
saves results to `results/`, and prints a verification of the
simulated damping ratio against the analytically derived target.

## Repository Structure

```
quarter-car-suspension/
├── README.md
├── requirements.txt
├── simulation/
│   ├── parameters.py        # physical constants (ms, mu, ks, b, kt)
│   ├── road_input.py        # road bump profile
│   ├── passive_model.py     # passive equations of motion
│   ├── pid_model.py         # PID active equations of motion + gain derivation
│   ├── metrics.py           # RMS displacement, settling time, peak accel
│   └── run_simulation.py    # main script — solves both, plots, prints results
├── demo/
│   └── index.html           # live interactive passive vs PID animation
├── results/
│   └── (plots + summary saved here when you run the simulation)
└── report/
    └── (technical writeup / poster goes here)
```

## Author

Lilian Olukutukei — Stanford University, Mechanical Engineering
(Mechanical Design, Manufacturing & Robotics)
