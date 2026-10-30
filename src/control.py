import numba as nb
import numpy as np

import src.motor as motor
from src.state_space import r_w, state_dot

# Not cached: these freeze constants imported from state_space and motor, and
# numba keys its cache on this file alone, so a cached build would survive edits
# there and silently run with stale parameters.


# Drive phase is advanced by this much against the spin so the bot travels
# along psi_d. Calibrated at the 3650-4000 RPM operating point by zeroing it
# and measuring steady-state course error. Only about 20 degrees of this is
# thrust lag; the rest corrects the slip coupling, which rotates the settled
# course away from the thrust direction.
PHASE_LEAD = np.float64(np.radians(83.5))

# Below this the lead calibration does not hold, so translation is not
# attempted and both motors stay powered. OpenMelt does the same via
# MIN_TRANSLATION_RPM, forcing motor_on_portion to 1 when spinning too slowly.
MIN_TRANSLATION_OMEGA = np.float64(2000.0*np.pi/30.0)


@nb.njit
def _ssa(angle):
    """Smallest signed angle, inlined because geomutils is not jitted."""
    return ((angle + np.pi) % (2.0*np.pi)) - np.pi


@nb.njit
def wheel_duties(theta, omega, cmd):
    """Melty drive law for cmd = [psi_d, portion, spin].

    Returns [u_L, u_R, gate_L, gate_R]. Only the sign of `spin` is used and
    duty is always full, because a part-throttle motor at spin speed sits below
    its own back-EMF and regen brakes instead of driving. `portion` is the
    fraction of each revolution a motor is powered for: 1 keeps both on for the
    whole turn (max spin, no translation), 0.5 gives two complementary
    half-turn arcs (max translation), and below that leaves gaps where neither
    is driven. This is OpenMelt's BINARY_THROTTLE scheme.
    """
    psi_d = cmd[0]
    portion = cmd[1]
    spin = cmd[2]

    out = np.empty(4)

    # No spin command means motors off. Gating one on at zero duty would short
    # the back-EMF into a full-current brake and thrust the bot backwards.
    if spin == 0.0:
        out[0] = 0.0
        out[1] = 0.0
        out[2] = 0.0
        out[3] = 0.0
        return out

    forward = spin > 0.0
    out[0] = -1.0 if forward else 1.0
    out[1] = 1.0 if forward else -1.0

    # Too slow to translate predictably: spin up on full power instead.
    if abs(omega) < MIN_TRANSLATION_OMEGA:
        out[2] = 1.0
        out[3] = 1.0
        return out

    # The lead compensates for the body sweeping past the arc, so it follows
    # the actual spin direction, not the commanded one. The two disagree while
    # braking, and keying off the command would mis-phase thrust by 2*lead.
    lead = PHASE_LEAD if omega >= 0.0 else -PHASE_LEAD
    phi = _ssa(theta - psi_d + lead)

    # The wheel driven to positive body x thrusts along psi_d, so it is the one
    # powered around phi = 0; its opposite number is powered half a turn later.
    half_arc = np.pi*portion
    gate_pos = 1.0 if abs(phi) <= half_arc else 0.0
    gate_neg = 1.0 if abs(_ssa(phi - np.pi)) <= half_arc else 0.0

    if forward:
        out[2] = gate_neg
        out[3] = gate_pos
    else:
        out[2] = gate_pos
        out[3] = gate_neg
    return out


def translating(omega, cmd):
    """Whether the drive law will actually produce translation right now.

    Not jitted: called once per frame to decide whether to draw the heading
    arrow, where njit dispatch would cost more than the comparisons.
    """
    return (cmd[2] != 0.0
            and abs(omega) >= MIN_TRANSLATION_OMEGA
            and cmd[1] < 1.0)
    return out


@nb.njit
def closed_loop(state, cmd):
    """Plant RHS for x = [X, Y, theta, vx, vy, omega, w_L, w_R]."""
    theta = state[2]
    omega = state[5]
    vx_b = np.cos(theta)*state[3] + np.sin(theta)*state[4]

    u = wheel_duties(theta, omega, cmd)

    # Wheels sit at +-r_w on the body y-axis, so spin adds -+omega*r_w to the
    # contact patch speed. The body state feeds back here: this is a loop, not
    # a feedforward chain from duty to force.
    left = motor.wheel_dot(state[6], u[0], u[2], vx_b - omega*r_w)
    right = motor.wheel_dot(state[7], u[1], u[3], vx_b + omega*r_w)

    forces = np.empty(2)
    forces[0] = left[1]
    forces[1] = right[1]

    out = np.empty(8)
    out[:6] = state_dot(state, forces)
    out[6] = left[0]
    out[7] = right[0]
    return out
