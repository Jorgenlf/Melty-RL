# Meltybrain Simulator: Visualization and Manual Control

- **Date:** 2026-10-29
- **Status:** Approved, pending implementation plan

## Goal

Add an interactive real-time visualization and keyboard control layer on top of
the existing planar meltybrain plant, so the bot can be driven manually and its
behavior inspected step by step.

## Background

The plant already exists and is numba-jitted:

- `src/state_space.py` — parameters plus `state_dot(state, wheel_forces)` for
  the six-state model `x = [X, Y, theta, vx, vy, omega]`.
- `src/rksolver.py` — `rk4_step(f, x, u, dt)` and `simulate(f, x0, u, dt, n)`,
  both jitted, with `f` passed as a first-class jitted function.
- `src/geomutils.py` — `ssa`, `Rz`, frame transforms.
- `src/bot.py` — `Bot` wrapper exposing named state properties.

Arena frame is standard Cartesian: `+X` right, `+Y` up, `theta` counterclockwise
from `+X`, kept unwrapped. Wheel forces are `[F_left, F_right]` along the body
x-axis, with the left wheel at `+r_w` and the right at `-r_w` on the body y-axis.

## Operating point

The simulator targets **2000–4000 RPM** for a 3 kg disk.

| Quantity | Value |
| --- | --- |
| Spin rate | 33.3–66.7 rev/s |
| Angular velocity | 209–419 rad/s |
| Max differential torque | `tau_max = 2 * r_w * f_wheel_max = 3.53` N m |

### Plant parameters

| Symbol | Value | Basis |
| --- | --- | --- |
| `m` | 3.0 kg | Given |
| `R_body` | 0.15 m | 300 mm disk |
| `I` | 0.03375 kg m^2 | Uniform disk, `0.5*m*R^2` |
| `r_w` | 0.12 m | Wheels near the rim for torque arm |
| `f_wheel_max` | 14.7 N | Traction limit, `mu ~ 1.0` on the per-wheel share of 29.4 N weight |
| `b_v` | 2.4 N s/m | Tuned for a 3.9 m/s unsaturated terminal speed |
| `b_omega` | 8.42e-3 N m s/rad | Tuned so full torque asymptotes to 4000 RPM |

```
omega_ss = tau_max / b_omega = 3.528 / 8.42e-3 = 419 rad/s = 4000 RPM
```

Spin-up time constant is `I / b_omega = 0.03375 / 8.42e-3 = 4.0 s`, a plausible
spin-up for a disk this size. 2000 RPM is the half-torque asymptote.

## Timing

- **Physics timestep:** `dt = 1e-4` s. This yields 150 steps/revolution at
  4000 RPM and 300 at 2000 RPM, and resolves the drive square wave's switching
  phase to roughly 2.4 degrees.
- **Render rate:** 60 Hz, so about 167 substeps per frame.
- **Accumulator:** frame time does not divide evenly into `dt`, and frames can
  hitch. A fixed-step accumulator carries the remainder so physics stays
  deterministic and independent of frame pacing.

Cost is roughly 85 us of jitted work per frame against a 16.7 ms budget.

## Control model: pure melty

Below spin, only forward and backward motion is possible; there is no sideways
authority. The drive law is therefore uniform across all spin rates, with no
mode switching.

### Drive law

Given a commanded arena-frame direction `psi_d` and throttle `a` in `[0, 1]`:

```
f_b = a * f_wheel_max * sign(cos(theta - psi_d))
```

Both wheels push forward through the half revolution where the body x-axis
points within 90 degrees of `psi_d`, and reverse through the other half. Mean
thrust over a revolution is `(2/pi) * a * f_wheel_max = 9.36` N along `psi_d`,
giving an unsaturated terminal speed of `9.36 / b_v = 3.90` m/s.

A sinusoidal `cos(theta - psi_d)` modulation is smoother but about 36% weaker;
the square wave matches real hardware and is the chosen default.

**The drive law must be evaluated inside the integration loop at every RK4
stage.** `theta` advances roughly a full revolution per rendered frame, so a
once-per-frame evaluation would be meaningless.

### Mixing

```
F_L = f_b/2 - tau_cmd/(2*r_w)
F_R = f_b/2 + tau_cmd/(2*r_w)
```

### Saturation: uniform scaling

When a command is infeasible, scale the wheel vector toward the origin rather
than clipping each wheel independently:

```
s = min(1, f_wheel_max / max(|F_L|, |F_R|))
F_L, F_R = s*F_L, s*F_R
```

Because `f_b` and `tau` are both linear in the wheel forces, `s` scales both
equally and the spin-to-translation ratio is preserved. Independent clipping
would instead starve translation completely at full spin command.

At full spin plus full translation: `F_L = -7.35, F_R = 22.05`, so `s = 2/3`,
giving `F_L = -4.9, F_R = 14.7` — 67% of both commanded thrust and commanded
torque. Terminal spin therefore sags to about 2667 RPM while translating, which
is the real tradeoff: translation costs RPM.

The 2/3 figure is parameter-independent. Because `f_b` is capped at
`f_wheel_max` while `tau_max/(2*r_w)` also equals `f_wheel_max`, the commanded
wheel pair is always in a 1:2 ratio at full command.

Since the square wave flips `f_b` twice per revolution, `s` oscillates at twice
the spin frequency. Inertia averages this out, but `s` is surfaced on the HUD.

## Architecture

```
src/state_space.py   # existing plant; retune b_omega only
src/rksolver.py      # unchanged
src/geomutils.py     # unchanged
src/bot.py           # + run(f, u, n_substeps) -> trajectory
src/control.py       # NEW: jitted drive law
src/viz.py           # NEW: pygame rendering
src/main.py          # NEW: loop, input, mode state
```

### `src/control.py`

```
wheel_forces(theta, cmd) -> [F_L, F_R]     # jitted; modulation, mix, scaling
closed_loop(state, cmd)  -> state_dot      # jitted; calls ss.state_dot(state, wheel_forces(state[2], cmd))
```

`cmd = [psi_d, throttle, tau_cmd]`. This matches the existing
`simulate(f, x0, u, dt, n)` signature with no changes — `u` carries the command
instead of raw wheel forces — so a whole frame integrates inside compiled code.

### `src/bot.py`

Add `run(f, u, n_substeps)`: calls `simulate`, stores the final row as the new
state, and returns the full trajectory. The trajectory doubles as the data for
the intra-frame motion trail.

### Dependency direction

`bot` knows the plant, `control` knows the drive law, `main` is the only glue.
`bot` does not import `control`. Each unit is testable without pygame.

### Numba placement

Jit only where it pays. `state_space.state_dot`, `control.wheel_forces` and
`control.closed_loop` are `njit(cache=True)` and run roughly 670 times per
frame. `rksolver.rk4_step` and `simulate` are `njit` without caching, since
caching is not supported alongside first-class function arguments.

`geomutils` is deliberately **not** jitted: it is called a handful of times per
frame from Python properties, where dispatch overhead exceeds the work. Cross
module jit-to-jit calls were verified to compile and cache cleanly.

## Input

| Key | Action |
| --- | --- |
| Arrow keys | Arena-frame direction `psi_d`; diagonals combine; throttle = 1 while held |
| Space (hold) | Spin-up torque |
| Left Shift (hold) | Reverse torque / brake |
| `C` | Toggle camera follow |
| `P` | Pause / resume |
| `N` | Advance one physics substep (`1e-4` s) |
| `M` | Advance one frame (167 substeps) |
| `R` | Reset |
| `Esc` | Quit |

Arrow keys are read as held state via `pygame.key.get_pressed()` and combined
into a direction vector, which is normalized to yield `psi_d`. They are *not*
turn commands; the bot's own rotation is invisible to the driver, matching how
a real melty is flown.

Key auto-repeat is enabled so `N` and `M` can be held to scrub.

## Rendering

Library: **pygame-ce**, chosen for real-time frame pacing plus held-key input.
Alternatives considered: matplotlib `FuncAnimation` (event-only keyboard, poor
frame rate), and pyglet/arcade/raylib (comparable capability, heavier install or
thinner 2D API). None offered an advantage.

At 60 fps the bot turns 0.55–1.1 revolutions per frame, so any drawn heading
marker is pure aliasing.

- **Chassis:** filled circle.
- **Blur ring:** replaces discrete wheel markers above about 5 rev/s.
- **Beacon:** a dot drawn at the peak-thrust phase, `theta = psi_d`. This mimics
  a real melty's translation LED and appears stationary when phase-locked. It
  replaces the heading line.
- **Trail:** deque of past positions, faded.
- **Command arrow:** `psi_d` drawn from the bot center.
- **Grid:** infinite; no arena bounds or collision in this version.

### Camera

Toggled with `C`:

- **Follow** (default): camera centered on the bot.
- **Fixed:** camera centered on world origin.

### HUD

Running: RPM, rev/s, speed, `psi_d`, throttle, saturation scale `s`,
substeps/frame.

Paused: numeric dump of `X`, `Y`, `theta`, `vx`, `vy`, `omega`, both wheel
forces, and `s`. This is the primary debugging payload of stepping mode.

## Testing

All tests headless, with no pygame dependency:

1. **Coast decay** matches `v(t) = v0 * exp(-b_v*t/m)`.
2. **Spin-up** matches the first-order analytic solution for constant `tau`.
3. **Melty translation** drift direction matches `psi_d` within tolerance over
   several revolutions, across a sweep of spin rates in 2000–4000 RPM.
4. **Saturation** at full spin plus full translation yields `s = 2/3` and
   non-zero `f_b`, guarding the regression that independent clipping caused.
5. **Timestep convergence:** trajectories at `dt = 1e-4` and `5e-5` agree within
   tolerance.

## Out of scope

Deferred to later work:

- Arena bounds, walls, collisions.
- Closed-loop RPM governor. Spin is open-loop here.
- Heading estimation. The simulator knows `theta` exactly; a real melty must
  estimate rotational phase from accelerometers and will drift.
- Motor dynamics, traction limits, wheel slip.
- Gamepad input.
