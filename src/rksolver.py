import numba as nb
import numpy as np


@nb.njit
def rk4_step(f, x, u, dt):
    """Advance a time-invariant ODE xdot = f(x, u) by one step with classical RK4.

    The input u is held constant across all four stages. A phase dependent drive
    law belongs inside f, so that every stage is evaluated with its own
    intermediate heading.

    Parameters:
        f: njit'd callable(x, u) -> np.ndarray. Right hand side of the ODE.
        x: np.ndarray. Current state.
        u: np.ndarray. Input applied over the step.
        dt: float. Step length.
    Returns:
        np.ndarray. State after dt.
    """
    k1 = f(x, u)
    k2 = f(x + 0.5*dt*k1, u)
    k3 = f(x + 0.5*dt*k2, u)
    k4 = f(x + dt*k3, u)
    return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)


@nb.njit
def simulate(f, x0, u, dt, n_steps):
    """Integrate n_steps of f from x0 and return the state trajectory.

    Rolling the loop into compiled code avoids a Python round trip per step.
    """
    traj = np.empty((n_steps + 1, x0.shape[0]))
    traj[0] = x0
    for k in range(n_steps):
        traj[k + 1] = rk4_step(f, traj[k], u, dt)
    return traj
