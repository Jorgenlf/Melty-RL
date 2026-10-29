import numpy as np

# Deliberately not jitted: these are called at most a few times per rendered
# frame from Python, where njit dispatch overhead exceeds the work itself.


def deg2rad(deg):
    return deg * np.pi / 180


def rad2deg(rad):
    return rad * 180 / np.pi


def ssa(angle):
    """Returns the smallest signed angle in the range [-pi, pi]."""
    return ((angle + np.pi) % (2*np.pi)) - np.pi


def Rz(theta):
    """Rotation matrix from the body frame to the arena frame."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s],
                     [s, c]])


def body_to_world(theta, vec):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([c*vec[0] - s*vec[1], s*vec[0] + c*vec[1]])


def world_to_body(theta, vec):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([c*vec[0] + s*vec[1], -s*vec[0] + c*vec[1]])


def to_screen(position, origin, scale):
    """Maps an arena position (+Y up) to screen pixels (+Y down)."""
    x, y = position
    x0, y0 = origin
    return np.array([x0 + scale*x, y0 - scale*y])