import numba as nb
import numpy as np

# Planar meltybrain parameters, sized for a 3 kg spinning disk.
# Arena frame is standard Cartesian: +X right, +Y up, theta measured
# counterclockwise from +X. The body x-axis is the "forward" translation axis.

m = np.float64(3.0)             # kg, chassis mass
R_body = np.float64(0.15)       # m, disk radius
I = np.float64(0.5*m*R_body**2) # kg m^2, uniform disk about the spin axis
r_w = np.float64(0.12)          # m, lateral offset of each wheel from the spin axis

b_v = np.float64(2.4)           # N s/m, translational viscous drag
b_omega = np.float64(8.42e-3)   # N m s/rad, rotational viscous drag

# Traction limit: mu ~ 1.0 against the per-wheel share of the 29.4 N weight.
f_wheel_max = np.float64(14.7)  # N, peak tractive force a single wheel can deliver

# Full differential torque, 3.53 N m, which asymptotes to 4000 RPM against b_omega.
tau_max = np.float64(2.0*r_w*f_wheel_max)

# Maps wheel forces [F_left, F_right] to the body wrench [f_b, tau]. Both wheels
# push along the body x-axis; the left wheel sits at +r_w and the right wheel at
# -r_w along the body y-axis, so a positive right wheel force gives a positive
# (counterclockwise) torque.
B_mat = np.array([[1.0, 1.0],
                  [-r_w, r_w]])

# Diagonals of the inverse mass matrix and of the viscous drag, on [vx, vy, omega].
M_inv_diag = np.array([1.0/m, 1.0/m, 1.0/I])
D_diag = np.array([b_v, b_v, b_omega])


def M_inv():
    """Inverse of the generalized mass matrix acting on [vx, vy, omega]."""
    return np.diag(M_inv_diag)


def B():
    """Maps wheel forces [F_left, F_right] to the body wrench [f_b, tau]."""
    return B_mat


def D(nu):
    """Viscous drag acting on nu = [vx, vy, omega]."""
    return D_diag*np.asarray(nu)


@nb.njit(cache=True)
def state_dot(state, input_):
    """Right hand side of the 6 ODEs for x = [X, Y, theta, vx, vy, omega].

    The constants above are frozen into the machine code at compile time, so a
    parameter change only takes effect after the cache is cleared.
    """
    theta = state[2]
    f_b = B_mat[0, 0]*input_[0] + B_mat[0, 1]*input_[1]
    tau = B_mat[1, 0]*input_[0] + B_mat[1, 1]*input_[1]

    out = np.empty(6)
    out[0] = state[3]
    out[1] = state[4]
    out[2] = state[5]
    out[3] = M_inv_diag[0]*(f_b*np.cos(theta) - D_diag[0]*state[3])
    out[4] = M_inv_diag[1]*(f_b*np.sin(theta) - D_diag[1]*state[4])
    out[5] = M_inv_diag[2]*(tau - D_diag[2]*state[5])
    return out