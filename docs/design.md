# Meltybrain Simulator: Visualization and Manual Control

- **Status:** Implemented. Keep in step with the code.

This records *why* the simulator is built the way it is: where the plant
parameters come from, why saturation scales rather than clips, why the timestep
is what it is, and which approaches were rejected. See the README for how to run
and drive it.

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
| Contact patch speed at 4000 RPM | `omega * r_w = 50.3` m/s |

### Plant parameters

| Symbol | Value | Basis |
| --- | --- | --- |
| `m` | 3.0 kg | Given |
| `R_body` | 0.15 m | 300 mm disk |
| `I` | 0.03375 kg m^2 | Uniform disk, `0.5*m*R^2` |
| `r_w` | 0.12 m | Wheels near the rim for torque arm |
| `f_wheel_max` | 14.7 N | Traction limit, `mu ~ 1.0` on the per-wheel share of 29.4 N weight |
| `b_v` | 0.5 N s/m | Residual aero and skid scrub only |
| `b_omega` | 3.4e-3 N m s/rad | Residual drag; lands terminal spin at 4000 RPM |

Both drag terms are *residual*. Before the motors were modelled they were
tuned to set terminal speed and spin, and `b_omega = 8.42e-3` was in effect a
stand-in for back-EMF. Now that `motor.py` models back-EMF explicitly, keeping
the old value would double-count it.

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
authority. The drive law carries a single mode switch, at
`MIN_TRANSLATION_OMEGA`, below which it stops attempting translation.

### Drive law

The command is `cmd = [psi_d, portion, spin]`. Only the **sign** of `spin` is
used and duty is always full; `portion` is the fraction of each revolution a
motor is powered for:

```
phi       = ssa(theta - psi_d + PHASE_LEAD)
half_arc  = pi * portion
gate_pos  = |phi| <= half_arc                 # wheel thrusting along +body x
gate_neg  = |ssa(phi - pi)| <= half_arc
u_L, u_R  = -sign(spin), +sign(spin)
```

`portion = 1` powers both motors for the whole turn: maximum spin and exactly
zero translation. `portion = 0.5` gives two complementary half-turn arcs, the
most translation available. Below 0.5 there are gaps where neither motor is
driven. The gated-off motor **coasts**; it is not braked.

**Duty is never partial.** A motor at part throttle while its wheel is at spin
speed sits below its own back-EMF and regen brakes instead of driving. At the
operating point the break-even duty is `k_t*w/V_bat = 0.904`, so anything under
about 90% brakes. Commanding `spin = 0.25` once drove the bot *backwards* at
0.845 m/s, and pinned terminal spin in a limit cycle on the
`MIN_TRANSLATION_OMEGA` boundary. Open Melt avoids this by construction: in
`BINARY_THROTTLE` the motor pin is fully on or off and the stick sets
`motor_on_portion`, never the voltage. Spin rate is controlled by arc width,
never by amplitude.

This follows [OpenMelt 2](https://github.com/nothinglabs/openmelt2), which
gates `motor_1_on()` / `motor_1_coast()` over an arc set by
`motor_on_portion`, with motor 2 trailing 180 degrees. Open Melt ties that
portion to its single RC throttle channel; here it is its own axis so spin
direction stays independently controllable.

**Coast, do not brake.** An earlier revision mixed translation in as a duty
offset, so the trailing wheel landed at duty 0. That is not coasting — it
shorts the back-EMF into a hard regen brake, 48 A of it at spin. It produced
more thrust but collapsed terminal spin from 4002 to 836 RPM at `portion` 0.5.
Coasting holds 3649 RPM. Open Melt notes the same preference, calling a fully
unpowered coast the best setting for translation.

**The drive law must be evaluated inside the integration loop at every RK4
stage.** `theta` advances roughly a full revolution per rendered frame, so a
once-per-frame evaluation would be meaningless.

### Phase lead

`PHASE_LEAD = 83.5 degrees`, applied against the spin so the bot *travels*
along `psi_d`. Without it the course sits 83 degrees off the command and the
bot is not drivable. Calibrated by zeroing the constant and measuring
steady-state course error; residual error afterwards is within 1.7 degrees at
every `portion`, for all four commanded directions, in both spin senses.

**Thrust lag is not the whole story, and is not even most of it.** Measuring
the instantaneous mean thrust direction over one revolution, with velocity
zeroed so there is no history to contaminate it, gives a lag of only 1 degree
at 400 RPM rising to about 20 degrees at 3500 RPM. The remaining ~63 degrees
comes from the slip coupling: at body speed `v` the contact velocity modulates
by `+-v` once per revolution, which rotates the settled course away from the
thrust direction. The command steers where the bot *ends up*, so the lead is
calibrated against course, not thrust.

The lead keys off the sign of **actual** `omega`, not commanded `spin`. The
body sweeping past the arc is what creates the lag, so the correction must
follow the real rotation. The two disagree while braking, and keying off the
command mis-phases thrust by `2*PHASE_LEAD`, roughly 167 degrees.

### Low-RPM cutoff

Because lag grows with spin rate, a single constant only holds near the
operating point. Steady operation always lands between 3649 and 3948 RPM, so
that is where it is calibrated; below it the correction over-rotates and the
bot wanders during spin-up.

`MIN_TRANSLATION_OMEGA` is therefore 2000 RPM, the bottom of the stated
operating band. Below it translation is not attempted and both motors stay
powered. Open Melt does the same through `MIN_TRANSLATION_RPM`, forcing
`motor_on_portion` to 1 when the rotation interval is too long.

This acts as a floor on spin: ask for a `portion` whose natural terminal spin
would fall below 2000 RPM and the guard hands back full power until it climbs
above again, settling exactly on the threshold. That is the intended
behaviour, not a defect — it stops a greedy translation command from stalling
the spin — and both branches drive, so it does not chatter.

This is the one mode switch in the drive law. Scaling the lead with RPM instead
would need a second calibration curve for the slip-coupling term, which is not
justified by anything measured so far.

### Zero spin coasts

At `spin = 0` both gates are forced off and both duties are zero. A motor gated
**on** at zero duty is not coasting — it shorts the back-EMF into a brake. At
the operating point that is a 13.4 V EMF across 0.06 ohm, clamped to the full
48 A limit, which reverses the contact force and drives the bot *backwards*:
measured course came out at 175 degrees against a commanded 0. The guard makes
releasing spin a pure coast-down with no translation authority, which matches
Open Melt, where zero throttle simply unpowers the motors.

## Motor model

Two PROPDRIVE v2 2836 1200KV brushless outrunners, one per wheel. Published:
1200 KV, 48 A max, 50 A ESC, 3S–4S LiPo, 82 g.

| Symbol | Value | Basis |
| --- | --- | --- |
| `V_bat` | 14.8 V | 4S LiPo nominal |
| `k_t`, `k_e` | 7.96 mN m/A | `60/(2*pi*KV)` |
| `R_motor` | 0.06 ohm | Estimated; HobbyKing does not publish it |
| `i_max` | 48 A | Published ceiling, matches the 50 A ESC |
| `r_wheel` | 0.030 m | 60 mm wheels |
| `J_w` | 3.0e-5 kg m^2 | Rotor bell plus wheel |
| `b_w` | 6.4e-6 N m s/rad | No-load losses as viscous drag |
| `eps_slip` | 0.3 m/s | Friction regularisation |

### Gearing is forced

Holding 4000 RPM needs a contact patch speed of `omega * r_w = 50.3` m/s, so a
30 mm wheel must turn at 1676 rad/s. No-load shaft speed is only
`KV * V_bat = 17,760` RPM = 1860 rad/s. **Any reduction puts the spin target out
of reach**, so the drivetrain is direct and the model has no gear ratio at all:
adding one would also mean reflecting rotor inertia as `G^2*J_rotor` and
rescaling `b_w`, so a bare `G` constant would be a trap. Measured terminal spin
is 4002 RPM against a 4412 RPM no-load ceiling.

### Current limit binds before traction

Stall current is `V_bat / R_motor = 247` A, far above the 48 A ESC ceiling, so
the limit that actually binds is current. Peak sustained force per wheel is
`k_t * i_max / r_wheel = 12.7` N, just under the 14.7 N traction cap — the
motor runs out before the tyre does.

### Force comes from slip

```
i   = gate * clip((u*V_bat - k_e*w_wheel)/R_motor, -i_max, i_max)
F   = f_wheel_max * tanh((r_wheel*w_wheel - v_contact)/eps_slip)
w_dot = (k_t*i - r_wheel*F - b_w*w_wheel)/J_w
```

`gate = 0` is a coast: the ESC floats the phases and no current flows. It is a
distinct mode from `u = 0`, which would be a regen brake.

`v_contact` is the ground speed of the contact patch along body x, which is
`vx_body -+ omega*r_w` for the left and right wheel. The body state therefore
feeds back into the wheel state: this is a loop, not a feedforward chain from
duty to force. The two wheel speeds are carried in the integrated state vector
for that reason.

Inductance is omitted. `L/R ~ 0.4` ms is small against the 7.5 ms drive
half-period, and carrying current as a state would add stiffness for little
behavioural gain.

`eps_slip` is pure regularisation, not a tyre model. It is the one soft
parameter in the set and the model is sensitive to it: equilibrium slip at full
spin is about 0.14 m/s, inside the band, so `eps_slip` sets the effective
contact stiffness and hence the operating point.

### Stiffness and timestep

The slip loop is the stiffest mode:

```
lambda ~ -r_wheel^2 * (f_wheel_max/eps_slip) / J_w = -1470 /s
```

At `dt = 1e-4` that gives `|lambda|*dt = 0.147` against an RK4 stability limit
near 2.8, so the existing timestep stands with wide margin. Shrinking
`eps_slip` is what would spend that margin.

### What the model reveals

These are measured from the simulator, not assumed:

- **A mid-spin wheel reversal is impossible.** Reversing a wheel from +1681 to
  −1681 rad/s at full braking current takes about 2 s, roughly 267 drive
  half-periods. The original force-space law performed that reversal for free
  twice per revolution whenever its torque command was small.
- **Thrust lags the gate**, by 1 degree at 400 RPM rising to about 20 degrees
  at 3500 RPM. That is only part of the course error; see *Phase lead*.
- **Translation is expensive and slow.** Even coasting rather than braking,
  `portion` 0.5 buys 0.57 m/s and costs 350 RPM. The gated-off wheel keeps
  producing spin-sustaining force as it coasts down, which cancels much of the
  powered wheel's thrust. Mean net thrust over a revolution is only about
  0.3 N.

| Portion | Terminal spin | Speed | Course error |
| --- | --- | --- | --- |
| 1.00 | 4003 RPM | 0 | — |
| 0.90 | 3960 RPM | 0.10 m/s | −1.0 deg |
| 0.75 | 3876 RPM | 0.27 m/s | −0.4 deg |
| 0.50 | 3649 RPM | 0.57 m/s | +1.7 deg |
| 0.35 | 2877 RPM | 0.71 m/s | +3.6 deg |
| 0.20 | 2000 RPM | 0.67 m/s | +3.1 deg |

The 0.20 row sits on the `MIN_TRANSLATION_OMEGA` floor rather than at its own
natural terminal spin.

The speed figures are the least trustworthy numbers in the model and are the
ones to validate against hardware. They are set by the slip coupling rather
than by `b_v`: at body speed `v` the contact velocity modulates by `+-v`, and
in the linear part of the friction law that acts like a drag of
`f_wheel_max/eps_slip = 49` N per m/s scaled by the fraction of the revolution
spent unsaturated. Lowering `b_v` or widening `eps_slip` are the levers.

## Architecture

```
src/state_space.py   # body plant; residual drag only
src/motor.py         # per-wheel motor, slip and contact force
src/rksolver.py      # unchanged
src/geomutils.py     # ssa and world_to_body only
src/bot.py           # run(f, u, n_substeps) -> trajectory; 8-state
src/control.py       # drive law + closed loop assembly
src/viz.py           # pygame rendering
src/main.py          # loop, input, mode state
```

State is `x = [X, Y, theta, vx, vy, omega, w_L, w_R]`.

### `src/control.py`

```
wheel_duties(theta, omega, cmd) -> [u_L, u_R, gate_L, gate_R]  # jitted
closed_loop(state, cmd)         -> state_dot                   # jitted
translating(omega, cmd)         -> bool                        # plain Python
```

`cmd = [psi_d, portion, spin]`. This matches the existing
`simulate(f, x0, u, dt, n)` signature with no changes — `u` carries the command
instead of raw wheel forces — so a whole frame integrates inside compiled code.

`closed_loop` is the only place the three layers meet: it resolves duties from
the drive law, hands each wheel its contact speed, collects the two forces, and
concatenates `state_space.state_dot` with the two wheel derivatives.

### `src/bot.py`

Add `run(f, u, n_substeps)`: calls `simulate`, stores the final row as the new
state, and returns the full trajectory. The trajectory doubles as the data for
the intra-frame motion trail.

### Dependency direction

`state_space` owns the body and the traction cap, `motor` owns one wheel,
`control` owns the drive law and glues them in compiled space, `main` is the
only glue in Python space. `motor` imports `f_wheel_max` from `state_space`, so
`state_space` must not import `motor`. `bot` does not import `control`. Each
unit is testable without pygame.

### Numba placement

Jit only where it pays. `state_space.state_dot` is `njit(cache=True)` and runs
roughly 670 times per frame.

`control.wheel_duties`, `control.closed_loop`, `motor.wheel_dot`,
`rksolver.rk4_step` and `rksolver.simulate` are `njit` **without** caching. Do
not add `cache=True` to them:

- The `control` and `motor` functions freeze constants imported from
  `state_space`, and numba keys its cache on the defining file alone. A cached
  build survives edits to `state_space.py` and silently runs with stale
  parameters. This was observed in practice: with `f_wheel_max` changed to
  10.0, `wheel_dot` kept returning values derived from the old 14.7.
- The `rksolver` functions take first-class function arguments, which caching
  does not support.

`state_dot` is safe to cache because the constants it freezes live in its own
file, so editing them invalidates the cache correctly.

`geomutils` is deliberately **not** jitted: it is called a handful of times per
frame from Python properties, where dispatch overhead exceeds the work. Cross
module jit-to-jit calls were verified to compile cleanly.

## Input

| Key | Action |
| --- | --- |
| Up / Down (hold) | Drive forward / backward along the front |
| Left / Right (hold) | Precess the front at `STEER_RATE`, default 180 deg/s |
| Space (hold) | Spin up |
| Left Shift (hold) | Reverse spin / brake |
| `[` / `]` (hold) | Trim the driving `portion` at `PORTION_RATE` |
| `C` | Toggle camera follow |
| `P` | Pause / resume |
| `N` | Advance one physics substep (`1e-4` s) |
| `M` | Advance one frame (167 substeps) |
| `R` | Reset, including the front and `portion` |
| `Esc` | Quit |

Controls are **body-relative**, against a steerable reference heading `psi_ref`
that starts along +X:

```
psi_d = psi_ref        driving forward
psi_d = psi_ref + pi   driving backward
```

Left and right rotate `psi_ref` rather than strafing. Strafing was rejected
because it leaves `psi_ref` fixed forever, which reduces the LEDs to decoration
and locks the bot to four cardinal directions.

`psi_ref` is a driver-side reference only. It never enters the plant, and is
unrelated to the chassis angle `theta` except that the LEDs fire when `theta`
passes it.

Keys are read as held state via `pygame.key.get_pressed()`. Auto-repeat is
deliberately **not** enabled: it fires repeated `KEYDOWN` events while a key is
held, which made holding `C`, `P` or `R` toggle the camera, flicker the pause
state and re-reset continuously. `N` and `M` are therefore one step per press.

## Rendering

Library: **pygame-ce**, chosen for real-time frame pacing plus held-key input.
Alternatives considered: matplotlib `FuncAnimation` (event-only keyboard, poor
frame rate), and pyglet/arcade/raylib (comparable capability, heavier install or
thinner 2D API). None offered an advantage.

At 60 fps the bot turns 0.55–1.1 revolutions per frame, so any drawn heading
marker is pure aliasing.

- **Chassis:** filled circle.
- **Blur ring:** replaces discrete wheel markers above about 5 rev/s.
- **LEDs:** a green and a red emitter bolted to the chassis at body angles 0 and
  pi, drawn on the rim and riding round with the heading stripe. They light over
  a `LED_ARC` half-width arc as the chassis sweeps past `psi_ref`, so every
  flash necessarily lands at the reference: green forward, red aft. Unlit, they
  render as dim outlines at the true chassis angle.
- **Trail:** deque of past positions, faded.
- **Command arrow:** `psi_d` drawn from the bot center, but only while the
  drive law will really translate, so it cannot promise motion that the zero
  spin or low-RPM guards are suppressing.
- **Grid:** infinite; no arena bounds or collision in this version.

Flash detection scans every substep of the frame's trajectory, not the frame
endpoint. At 3000 RPM the chassis turns about a revolution per frame, so a
single sample would catch the flash at random. Measured worst-case flash offset
from `psi_ref` was 13.97 degrees against a 14 degree arc.

### Camera

Toggled with `C`:

- **Follow** (default): camera centered on the bot.
- **Fixed:** camera centered on world origin.

### HUD

Running: RPM, rev/s, speed, `psi_d`, `portion`, spin sign, whether translation
is actually active, effective duty and gate state per wheel, wheel RPM, and
substeps/frame.

Paused: numeric dump of `X`, `Y`, `theta`, `vx`, `vy`, `omega` and both wheel
speeds. This is the primary debugging payload of stepping mode.

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
