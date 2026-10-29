import numpy as np

import src.state_space as ss
import src.geomutils as geom
from src.rksolver import rk4_step, simulate


class Bot:
    """
    Planar meltybrain dynamics: a two wheeled chassis that translates while it
    spins about its own vertical axis.

    State x = [X, Y, theta, vx, vy, omega] in the arena frame
    (+X right, +Y up, theta counterclockwise from +X, kept unwrapped).
    Input u = [F_left, F_right], the tractive force of each wheel along the
    body x-axis.
    """

    def __init__(self, step_size, init_state=None):
        self.step_size = float(step_size)
        self.state = (np.zeros(6) if init_state is None
                      else np.asarray(init_state, dtype=np.float64).copy())
        self.input = np.zeros(2)

    def step(self, wheel_forces):
        self.input = np.clip(np.asarray(wheel_forces, dtype=np.float64),
                             -ss.f_wheel_max, ss.f_wheel_max)
        self.state = rk4_step(ss.state_dot, self.state, self.input, self.step_size)

    def state_dot(self, state, input_):
        """
        The right hand side of the 6 ODEs governing the meltybrain dynamics.
        """
        return ss.state_dot(state, input_)

    def run(self, f, u, n_substeps):
        """
        Advance n_substeps under the RHS f and return the intra-frame trajectory.
        """
        traj = simulate(f, self.state, u, self.step_size, n_substeps)
        self.state = traj[-1]
        return traj

    @property
    def position(self):
        """
        Returns the position of the bot in arena coordinates.
        """
        return self.state[0:2]

    @property
    def theta(self):
        """
        Returns the unwrapped heading, free to grow as the bot spins.
        """
        return self.state[2]

    @property
    def heading(self):
        """
        Returns the heading wrapped to [-pi, pi].
        """
        return geom.ssa(self.state[2])

    @property
    def velocity(self):
        """
        Returns the velocity of the bot in arena coordinates.
        """
        return self.state[3:5]

    @property
    def body_velocity(self):
        """
        Returns the velocity in the spinning body frame, i.e. [surge, sway].
        """
        return geom.world_to_body(self.state[2], self.velocity)

    @property
    def omega(self):
        """
        Returns the spin rate of the bot in rad/s.
        """
        return self.state[5]

    @property
    def spin_rate(self):
        """
        Returns the spin rate of the bot in revolutions per second.
        """
        return self.state[5]/(2*np.pi)

    @property
    def speed(self):
        """
        Returns the length of the velocity vector of the bot.
        """
        return np.linalg.norm(self.velocity)

    @property
    def chi(self):
        """
        Returns the course angle, i.e. the direction of travel in the arena frame.
        """
        vx, vy = self.velocity
        return np.arctan2(vy, vx)