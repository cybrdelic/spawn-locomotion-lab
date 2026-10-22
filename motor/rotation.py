"""Analytic Jacobian of an intrinsic XZY rotation matrix."""
import numpy as np
from scipy.spatial.transform import Rotation


def matrix_jacobian(angles):
    a, b, _ = angles
    sa, ca, sb, cb = np.sin(a), np.cos(a), np.sin(b), np.cos(b)
    axes = np.array([[1., 0., 0.], [0., -sa, ca], [-sb, ca*cb, sa*cb]])
    x, y, z = axes.T
    skew = np.zeros((3, 3, 3))
    skew[:, 0, 1], skew[:, 0, 2] = -z, y
    skew[:, 1, 0], skew[:, 1, 2] = z, -x
    skew[:, 2, 0], skew[:, 2, 1] = -y, x
    rotation = Rotation.from_euler('XZY', angles).as_matrix()
    return np.moveaxis(skew @ rotation, 0, -1).reshape(9, 3)
