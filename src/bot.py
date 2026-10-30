import numpy as np

import src.geomutils as geom
from src.rksolver import simulate

N_STATES = 8


class Bot:
    """
    Planar meltybrain dynamics: a two wheeled chassis that translates while it
    spins about its own vertical axis.

    State x = [X, Y, theta, vx, vy, omega, w_L, w_R] in the arena frame
    (+X right, +Y up, theta counterclockwise from +X, kept unwrapped). The
    trailing pair are wheel speeds in rad/s, carried here because contact force
    depends on slip against the body motion.
    """

    def __init__(self, step_size, init_state=None):
        self.step_size = float(step_size)
        self.state = (np.zeros(N_STATES) if init_state is None
                      else np.asarray(init_state, dtype=np.float64).copy())

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
    def wheel_speeds(self):
        """
        Returns the left and right wheel speeds in rad/s.
        """
        return self.state[6:8]

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