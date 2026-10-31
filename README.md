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

## Tests

```powershell
uv run pytest -q
```

`tests/test_physics.py` is headless and needs no display. It pins the 4002 RPM
operating point, the `PHASE_LEAD` calibration that makes translation track the
commanded heading, the coast-versus-brake behaviour at zero spin, the motor
current clamp, and timestep convergence. It runs in about 5 seconds.

## Controls

| Key | Action |
| --- | --- |
| Up / Down (hold) | Drive forward / backward along the **front** |
| Left / Right (hold) | Steer: rotate the front counterclockwise / clockwise |
| Space (hold) | Spin up |
| Shift (hold) | Brake / reverse spin |
| `[` / `]` (hold) | Narrow / widen the driving portion |
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
- **`portion`** — fraction of each revolution a motor is powered for. 1 is pure
  spin, 0.5 is maximum translation. Trim it with `[` and `]`.
- **`translate`** — whether the drive law is really translating. Red `NO` means
  a guard is suppressing it: no spin command, or below 2000 RPM.
- **`duty`** — effective duty per wheel, zero while a wheel is gated off.
- **`gate`** — `ON` or `COAST` per wheel. This is the translation pulse.
- **`wheel`** — left and right wheel speeds in RPM.
- **`front` / `led`** — reference heading in degrees, and which LEDs fired this
  frame.

Pausing switches the HUD to a numeric dump of the full state plus both wheel
speeds, which is the point of stepping mode.

## Tunable parameters

### Plant — `src/state_space.py`

| Symbol | Default | Meaning |
| --- | --- | --- |
| `m` | 3.0 | Chassis mass, kg |
| `R_body` | 0.15 | Disk radius, m. Also the drawn size. |
| `I` | derived | Moment of inertia, `0.5*m*R_body**2` |
| `r_w` | 0.12 | Wheel offset from the spin axis, m |
| `b_v` | 0.5 | Residual translational drag, N s/m |
| `b_omega` | 3.4e-3 | Residual rotational drag, N m s/rad |
| `f_wheel_max` | 14.7 | Peak tractive force per wheel, N |

`I` derives from the others, so changing `m` or `R_body` propagates correctly.

Both drag terms are now **residual only** — aero, bearings and skid scrub. The
motors are modelled explicitly, so back-EMF sets terminal spin, not `b_omega`.
Raising `b_omega` lowers top RPM below the motor's own ceiling; it cannot raise
it past that ceiling.

### Motors — `src/motor.py`

Two PROPDRIVE v2 2836 1200KV outrunners, direct drive.

| Symbol | Default | Meaning |
| --- | --- | --- |
| `V_bat` | 14.8 | Pack voltage, V (4S nominal) |
| `KV` | 1200 | RPM per volt |
| `R_motor` | 0.06 | Winding resistance, ohm (estimated) |
| `i_max` | 48 | ESC current limit, A |
| `r_wheel` | 0.030 | Wheel radius, m |
| `J_w` | 3.0e-5 | Rotor plus wheel inertia, kg m^2 |
| `eps_slip` | 0.3 | Friction regularisation, m/s |

`KV * V_bat` sets the hard spin ceiling: 17,760 RPM at the shaft. `b_w` loads
the shaft even with no traction load, dropping free speed to 17,653 RPM, which
is **4413 RPM at the body** and leaves no room for a reduction. There is
deliberately no gear-ratio constant — adding one would also need rotor inertia
reflected as `G^2*J_rotor` and `b_w` rescaled. Shrinking `eps_slip` makes the
contact stiffer and eats RK4 stability margin.

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
| `PHASE_LEAD` | `src/control.py` | 83.5 deg | Drive phase advance against the spin |
| `MIN_TRANSLATION_OMEGA` | `src/control.py` | 2000 RPM | Below this, spin up only; no translation |
| `DRIVE_PORTION` | `src/main.py` | 0.5 | Starting motor-on portion while driving |

`DT = 1e-4` gives 150 steps per revolution at 4000 RPM. Going much coarser will
under-resolve the drive square wave, which switches twice per revolution.

## How it works

State is `x = [X, Y, theta, vx, vy, omega, w_L, w_R]` in a standard Cartesian
arena frame: `+X` right, `+Y` up, `theta` counterclockwise from `+X` and left
unwrapped. The trailing pair are wheel speeds, carried because contact force
depends on slip against the body motion.

Translation comes from gating one motor off over part of each rotation. Neither
motor is ever reversed, the gated-off one coasts rather than braking, and duty
is always full:

```
psi_d    = psi_ref            driving forward
psi_d    = psi_ref + pi       driving backward
phi      = ssa(theta - psi_d + PHASE_LEAD)
half_arc = pi * portion
gate_pos = |phi| <= half_arc              # wheel thrusting along +body x
gate_neg = |ssa(phi - pi)| <= half_arc
u_L, u_R = -sign(spin), +sign(spin)
```

`portion` is the fraction of each revolution a motor is powered for: 1 keeps
both on the whole turn (max spin, zero translation), 0.5 gives two
complementary half-turn arcs (max translation). This follows
[OpenMelt 2](https://github.com/nothinglabs/openmelt2). It is evaluated
**inside** the integrator at every RK4 stage, since `theta` advances about a
full revolution per rendered frame.

Only the *sign* of `spin` is used. A motor at part duty while its wheel is at
spin speed sits below its own back-EMF and regen brakes rather than driving —
break-even is 90% duty at the operating point — so spin rate is controlled by
arc width, never by amplitude. Open Melt's `BINARY_THROTTLE` works the same
way.

`psi_ref` is the steerable front; `theta` is the physical chassis angle. They
are unrelated except that the LEDs fire when `theta` passes `psi_ref`.

### Phase lead

`PHASE_LEAD` in `src/control.py` is 83.5 degrees, applied against the spin so
the bot *travels* along the commanded heading. Without it the course sits 83
degrees off and the bot is not drivable.

Only about 20 degrees of that is thrust lag. The rest corrects the slip
coupling, which rotates the settled course away from the thrust direction, so
the constant is calibrated against where the bot ends up rather than against
where it pushes. To recalibrate after changing the plant: set `PHASE_LEAD` to
0, drive at a steady `portion` until speed converges, measure the course error,
and put it back.

The lead follows the sign of actual `omega`, not the commanded spin, because
it is the body sweeping past the powered arc that creates the lag.

### Translation needs spin

Two guards make the drive predictable, both mirroring real melty firmware:

- **Below 2000 RPM** (`MIN_TRANSLATION_OMEGA`) translation is not attempted and
  both motors stay powered. Lag grows with spin rate, so the single lead
  constant only holds near the operating point.
- **At zero spin command** both motors coast. Releasing `Space` while holding a
  direction gives a clean spin-down with no translation, not a reverse. A motor
  gated on at zero duty would short its back-EMF into a 48 A brake and push the
  bot backwards.

So you need spin before `Up` does anything useful — hold `Space`. The HUD
`translate` line tells you which state you are in.

### Motors

Duty is not force. Each wheel runs a motor model and the contact force comes
out of slip, never from the command:

```
i     = gate * clip((u*V_bat - k_e*w_wheel)/R_motor, -i_max, i_max)
F     = f_wheel_max * tanh((r_wheel*w_wheel - v_contact)/eps_slip)
w_dot = (k_t*i - r_wheel*F - b_w*w_wheel)/J_w
```

`v_contact` is the ground speed of the contact patch, `vx_body -+ omega*r_w`.
The body state feeds back into the wheels, so this is a loop rather than a
feedforward chain. `gate = 0` is a true coast — no current — which is a
different thing from `u = 0`, which would short the back-EMF into a regen
brake.

Three consequences you will feel while driving:

- **Wheels cannot reverse mid-spin.** Flipping a wheel from +1681 to −1681
  rad/s takes about 2 s even at full braking current — some 267 drive
  half-periods. Commanding it simply does not happen.
- **Translation is slow.** The best case is 0.57 m/s. The gated-off wheel
  keeps producing spin-sustaining force as it coasts down, cancelling much of
  the powered wheel's thrust.
- **Translation costs RPM,** though far less than braking would: 4003 RPM down
  to 3649 at `portion` 0.5.

| Portion | Terminal spin | Speed | Course error |
| --- | --- | --- | --- |
| 1.00 | 4003 RPM | 0 | — |
| 0.90 | 3960 RPM | 0.10 m/s | −1.0 deg |
| 0.75 | 3877 RPM | 0.27 m/s | −0.3 deg |
| 0.50 | 3649 RPM | 0.57 m/s | +1.7 deg |
| 0.35 | 2830 RPM | 0.72 m/s | +3.5 deg |
| 0.20 | 2000 RPM | 0.67 m/s | +2.9 deg |

Measured after a 90 s settle. The 0.35 row needs most of that — it still reads
2784 RPM at 30 s — so short runs overstate it.

The last row sits on the 2000 RPM floor rather than its own terminal spin: ask
for more translation than the spin can sustain and the guard hands back full
power. The speed column is the least trustworthy part of the model — it is set
by the slip coupling, not by `b_v`.

## Layout

```
src/state_space.py   body plant and state_dot              (jitted, cached)
src/motor.py         per-wheel motor, slip, contact force  (jitted, uncached)
src/control.py       melty drive law and closed-loop RHS   (jitted, uncached)
src/rksolver.py      RK4 step and fixed-step integrator    (jitted, uncached)
src/geomutils.py     angle and frame helpers               (plain Python)
src/bot.py           state wrapper with named properties
src/viz.py           pygame rendering
src/main.py          loop, input, mode handling
```
`state_space` owns the body, `motor` owns one wheel, `control` owns the drive
law and glues them in compiled space, `main` is the only glue in Python space.
Everything except `viz` and `main` runs headless, so the physics is testable
without a display.

### A note on numba

`motor.py`, `control.py` and `rksolver.py` are **deliberately not cached**. Numba keys its
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
