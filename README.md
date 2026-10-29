# Melty-RL

A planar simulator for a **meltybrain** combat robot: a two-wheeled chassis that
translates while spinning about its own vertical axis, with a real-time pygame
view and keyboard control.

Default plant is a 3 kg, 300 mm disk operating at 2000–4000 RPM.

Design rationale, parameter derivations and rejected alternatives are in
[docs/design.md](docs/design.md).

## Running

Run from the **repository root**, as a module:

```powershell
uv run python -m src.main
```

> Running `python main.py` from inside `src/` will fail. The modules import each
> other as `src.<name>`, so Python needs the repository root on its path and the
> `-m` form to resolve the package.

First launch pauses for a second or so while numba compiles the integrator.

## Controls

| Key | Action |
| --- | --- |
| Up / Down (hold) | Drive forward / backward along the **front** |
| Left / Right (hold) | Steer: rotate the front counterclockwise / clockwise |
| Space (hold) | Spin up |
| Shift (hold) | Brake / reverse spin |
| `P` | Pause / resume |
| `N` | While paused: advance one physics substep (0.1 ms) |
| `M` | While paused: advance one frame (167 substeps, roughly one revolution) |
| `C` | Toggle camera follow / fixed at origin |
| `R` | Reset, including the front back to +X |
| `Esc` | Quit |

### The "front" of a meltybrain

The chassis spins continuously, so the bot has no physical front. The front is a
**reference heading** the driver steers, starting along +X, and the controls are
mapped to it:

- **Up/Down** translate along that reference, not along the chassis.
- **Left/Right** precess the reference at 180 deg/s. This is how you change
  direction; there is no strafe.

The only way to see where the front points is the LEDs.

### LEDs

A **green** LED and a **red** one are bolted to the chassis at body angles 0 and
180 degrees, so they ride around with the stripe between the wheels rather than
sitting at a fixed arena angle. They light over a 28-degree arc as the chassis
sweeps past the reference heading, exactly as on real hardware.

Because they only fire while aligned, every flash lands at the reference: green
forward, red aft. Persistence of vision does the rest, and the pair appears to
hover at a fixed angle while the chassis underneath is a blur. When they are not
firing they show as dim outlines, still riding the rim at the true chassis angle.

At operating RPM the flash rate exceeds the 60 Hz frame rate, so the LEDs look
continuously lit. Below roughly 5 rev/s you can watch them blink. Steer with
left/right and the flash position rotates around the rim, showing you exactly
where forward is.

Flash detection scans every physics substep, not just the frame endpoint. At
3000 RPM the chassis turns about a full revolution per frame, so sampling once
would catch the flash at random.

You need spin before steering means anything. With the bot stationary the LEDs
only light if the chassis happens to be parked on the reference, and the bot
cannot translate sideways, so only forward and backward do anything. Hold
`Space` for a few seconds first.

### Reading the display

- **Rim tick** — true heading. Above roughly 17 rev/s it aliases against the
  60 Hz frame rate and jitters. That is honest, not a bug; pause and use `N` to
  watch the phase advance properly.
- **Blur ring** — replaces the wheel markers above 5 rev/s.
- **Green / red dots** — front and rear LEDs. See above.
- **Green arrow** — commanded translation direction, shown while driving.
- **`sat s`** — saturation scale, turns red below 1.0. See *Saturation* below.
- **`front` / `led`** — reference heading in degrees, and which LEDs fired this
  frame.

Pausing switches the HUD to a numeric dump of the full state plus both wheel
forces, which is the point of stepping mode.

## Tunable parameters

### Plant — `src/state_space.py`

| Symbol | Default | Meaning |
| --- | --- | --- |
| `m` | 3.0 | Chassis mass, kg |
| `R_body` | 0.15 | Disk radius, m. Also the drawn size. |
| `I` | derived | Moment of inertia, `0.5*m*R_body**2` |
| `r_w` | 0.12 | Wheel offset from the spin axis, m |
| `b_v` | 2.4 | Translational viscous drag, N s/m |
| `b_omega` | 8.42e-3 | Rotational viscous drag, N m s/rad |
| `f_wheel_max` | 14.7 | Peak tractive force per wheel, N |
| `tau_max` | derived | `2*r_w*f_wheel_max` |

`I` and `tau_max` derive from the others, so changing `m`, `R_body`, `r_w` or
`f_wheel_max` propagates correctly.

Two relations are worth knowing before you tune:

```
terminal spin       omega_ss = tau_max / b_omega          (419 rad/s = 4000 RPM)
spin-up time const  I / b_omega                           (4.0 s)
mean melty thrust   (2/pi) * f_wheel_max                  (9.36 N)
terminal speed      mean thrust / b_v                     (3.90 m/s)
```

So `b_omega` sets top RPM, and `b_v` sets top speed. Raising `f_wheel_max` raises
both.

### Simulation and view

| Constant | File | Default | Meaning |
| --- | --- | --- | --- |
| `DT` | `src/main.py` | 1e-4 | Physics timestep, s |
| `RENDER_HZ` | `src/main.py` | 60 | Frame rate |
| `FRAME_SUBSTEPS` | `src/main.py` | 167 | Substeps advanced by `M` |
| `MAX_SUBSTEPS` | `src/main.py` | 4000 | Ceiling so a stalled frame cannot spiral |
| `STEER_RATE` | `src/main.py` | pi | How fast left/right precess the front, rad/s |
| `PIXELS_PER_M` | `src/viz.py` | 250 | Zoom |
| `GRID_SPACING_M` | `src/viz.py` | 0.5 | Grid pitch, m |
| `BLUR_THRESHOLD_REV_S` | `src/viz.py` | 5.0 | Spin rate at which wheels become a blur ring |
| `LED_ARC` | `src/viz.py` | 14 deg | Half-width of the arc over which an LED fires |
| `TRAIL_MAX` | `src/viz.py` | 600 | Trail length |

`DT = 1e-4` gives 150 steps per revolution at 4000 RPM. Going much coarser will
under-resolve the drive square wave, which switches twice per revolution.

## How it works

State is `x = [X, Y, theta, vx, vy, omega]` in a standard Cartesian arena frame:
`+X` right, `+Y` up, `theta` counterclockwise from `+X` and left unwrapped.

Translation comes from phase-modulating thrust against the heading:

```
psi_d = psi_ref            driving forward
psi_d = psi_ref + pi       driving backward
f_b   = throttle * f_wheel_max * sign(cos(theta - psi_d))
```

Both wheels push forward through the half revolution whose body x-axis points
within 90 degrees of `psi_d`, and reverse through the other half, so the bot
drifts along `psi_d`. This is evaluated **inside** the integrator at every RK4
stage, since `theta` advances about a full revolution per rendered frame.

`psi_ref` is the steerable front; `theta` is the physical chassis angle. They
are unrelated except that the LEDs fire when `theta` passes `psi_ref`.


### Saturation

Spin and translation share one force budget. When a command exceeds it, the
wheel vector is scaled toward the origin rather than clipped per wheel:

```
s = min(1, f_wheel_max / max(|F_L|, |F_R|))
```

Because thrust and torque are both linear in the wheel forces, `s` scales both
equally and the spin-to-translation ratio survives. At full spin plus full
translation `s = 2/3`, so terminal spin sags from 4000 to about 2667 RPM while
you drive. Translation costs RPM, exactly as on real hardware.

## Layout

```
src/state_space.py   plant parameters and state_dot        (jitted, cached)
src/control.py       melty drive law and closed-loop RHS   (jitted, uncached)
src/rksolver.py      RK4 step and fixed-step integrator    (jitted, uncached)
src/geomutils.py     angle and frame helpers               (plain Python)
src/bot.py           state wrapper with named properties
src/viz.py           pygame rendering
src/main.py          loop, input, mode handling
```

`bot` knows the plant, `control` knows the drive law, `main` is the only glue.
Everything except `viz` and `main` runs headless, so the physics is testable
without a display.

### A note on numba

`control.py` and `rksolver.py` are **deliberately not cached**. Numba keys its
cache on the defining file, and these freeze constants imported from
`state_space.py` — a cached build would survive edits there and silently run
with stale parameters. Uncached, they recompile each start and always reflect
the current values.

`geomutils.py` is not jitted at all. It is called a few times per frame from
Python, where dispatch overhead exceeds the arithmetic.

If you ever do see parameter changes failing to take effect, clear the caches:

```powershell
Get-ChildItem -Path src -Recurse -Directory -Filter __pycache__ | Remove-Item -Recurse -Force
```
