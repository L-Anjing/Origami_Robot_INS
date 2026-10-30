"""Serial shape in W from independently observed plate attitudes; NumPy only."""
import numpy as np


def forward_chain(rotations_ws, link_offsets_s):
    """Return (...,7,3) marker offsets from V0 in W, not in root-local axes.

    First five offsets are parent-body to child-body displacements; the final
    offset defines the selected tip marker. No drive enables are joint angles.
    """
    displacements = np.einsum("...nij,nj->...ni", rotations_ws, link_offsets_s)
    zeros = np.zeros((*displacements.shape[:-2], 1, 3))
    return np.concatenate((zeros, np.cumsum(displacements, axis=-2)), axis=-2)


def rigid_transforms(rotations_ws, relative_points_w, root_position_w):
    transform = np.zeros((*rotations_ws.shape[:-2], 4, 4))
    transform[..., :3, :3] = rotations_ws
    transform[..., :3, 3] = relative_points_w[..., :6, :] + root_position_w[..., None, :]
    transform[..., 3, 3] = 1.
    return transform
