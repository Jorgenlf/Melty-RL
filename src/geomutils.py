import numpy as np

# Deliberately not jitted: these are called at most a few times per rendered
# frame from Python, where njit dispatch overhead exceeds the work itself.


def ssa(angle):
    """Returns the smallest signed angle in the range [-pi, pi]."""
    return ((angle + np.pi) % (2*np.pi)) - np.pi


def world_to_body(theta, vec):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([c*vec[0] + s*vec[1], -s*vec[0] + c*vec[1]])