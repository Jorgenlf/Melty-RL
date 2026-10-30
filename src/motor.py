import numba as nb
import numpy as np

from src.state_space import f_wheel_max

# Not cached: this freezes f_wheel_max imported from state_space, and numba keys
# its cache on this file alone, so a cached build would survive edits there and
# silently run with stale parameters.

# PROPDRIVE v2 2836 1200KV brushless outrunner, one per wheel.
# Published: 1200 KV, 48 A max, 50 A ESC, 3S-4S LiPo, 82 g.
V_bat = np.float64(14.8)                 # V, 4S LiPo nominal
KV = np.float64(1200.0)                  # RPM/V
k_t = np.float64(60.0/(2.0*np.pi*KV))    # 7.96 mN m/A; k_e is numerically equal in SI
R_motor = np.float64(0.06)               # ohm, winding resistance; estimated, not published
i_max = np.float64(48.0)                 # A, published ceiling and the ESC limit

# Direct drive, and the model assumes it: any reduction puts the 4000 RPM spin
# target out of reach, since the contact patch must run at omega*r_w = 50.3
# m/s, which needs 1676 rad/s at a 30 mm wheel against 1860 rad/s of no-load
# shaft speed. Adding a ratio would also mean reflecting rotor inertia as
# G^2*J_rotor and rescaling b_w, so there is deliberately no G constant here.
r_wheel = np.float64(0.030)              # m, 60 mm diameter wheels
J_w = np.float64(3.0e-5)                 # kg m^2, rotor bell plus wheel
b_w = np.float64(6.4e-6)                 # N m s/rad, no-load losses as viscous drag

# Slip at which the friction law saturates. Pure regularisation, not a tyre
# model; it sets the stiffest eigenvalue, so shrinking it costs timestep margin.
eps_slip = np.float64(0.3)               # m/s


@nb.njit
def wheel_dot(w_wheel, duty, gate, v_contact):
    """One wheel; returns [w_dot, F_contact]. gate 0 coasts, 1 powers at duty.

    v_contact is the ground speed of the contact patch along the body x-axis,
    so v_slip is what the tyre sees.
    """
    # Coasting floats the phases and carries no current. This is a distinct
    # mode from duty 0, which would short the back-EMF into a hard regen brake.
    i = 0.0
    if gate > 0.0:
        # Quasi-static armature current: inductance is omitted because L/R ~
        # 0.4 ms is small against the drive half-period, and the ESC clamps.
        i = (duty*V_bat - k_t*w_wheel)/R_motor
        if i > i_max:
            i = i_max
        elif i < -i_max:
            i = -i_max

    # Force is set by slip, never commanded directly. This is what gives the
    # drive its lag and what makes a mid-spin wheel reversal impossible.
    v_slip = r_wheel*w_wheel - v_contact
    f = f_wheel_max*np.tanh(v_slip/eps_slip)

    out = np.empty(2)
    out[0] = (k_t*i - r_wheel*f - b_w*w_wheel)/J_w
    out[1] = f
    return out
