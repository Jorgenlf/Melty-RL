import numba as nb
import numpy as np

from src.state_space import f_wheel_max, r_w, state_dot

# Not cached: these freeze constants imported from state_space, and numba keys
# its cache on this file alone, so a cached build would survive edits there and
# silently run with stale parameters.


@nb.njit
def wheel_forces(theta, cmd):
    """Melty drive law for cmd = [psi_d, throttle, tau_cmd].

    Returns [F_left, F_right, s], where s is the saturation scale applied.
    Thrust is a square wave against the heading phase, so both wheels push
    forward through the half revolution whose body x-axis points within 90
    degrees of psi_d, and reverse through the other half.
    """
    psi_d = cmd[0]
    throttle = cmd[1]
    tau_cmd = cmd[2]

    amplitude = throttle*f_wheel_max
    f_b = amplitude if np.cos(theta - psi_d) >= 0.0 else -amplitude

    half_tau = tau_cmd/(2.0*r_w)
    f_l = 0.5*f_b - half_tau
    f_r = 0.5*f_b + half_tau

    # Scale the wheel vector toward the origin rather than clipping each wheel,
    # so the spin-to-translation ratio survives an infeasible command.
    peak = max(abs(f_l), abs(f_r))
    s = 1.0
    if peak > f_wheel_max:
        s = f_wheel_max/peak
        f_l *= s
        f_r *= s

    out = np.empty(3)
    out[0] = f_l
    out[1] = f_r
    out[2] = s
    return out


@nb.njit
def closed_loop(state, cmd):
    """Plant RHS under the drive law, shaped for rksolver.simulate."""
    w = wheel_forces(state[2], cmd)
    return state_dot(state, w[:2])
