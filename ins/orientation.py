"""R_WS orientation; no NWU/NED convention is imported from a walking dataset."""
import warnings

import numpy as np
from scipy.spatial.transform import Rotation


def truth_rotations(quaternion_wxyz):
    shape = quaternion_wxyz.shape[:2]
    # SciPy normalizes roundoff; raw quaternion norm is reported before this step.
    return Rotation.from_quat(quaternion_wxyz[..., [1, 2, 3, 0]].reshape(-1, 4)).as_matrix().reshape(*shape, 3, 3)


def gyro_rotations(gyroscope, time, integration="trapezoid"):
    """Body angular rate: R[k] = R[k-1] Exp(omega_S dt).

    Only gyro measurements and documented R(0)=I are used. First output is at
    t[0]=0.001, so integrate omega[0] from t=0 rather than dropping that interval.
    Midpoint angular-rate integration neglects high-order coning terms.
    """
    result = np.empty((*gyroscope.shape[:2], 3, 3))
    state = Rotation.identity(gyroscope.shape[1])
    for k in range(len(time)):
        dt = time[k] if k == 0 else time[k] - time[k - 1]
        omega = gyroscope[k] if k == 0 or integration == "step_end" else (gyroscope[k - 1] + gyroscope[k]) / 2
        state = state * Rotation.from_rotvec(omega * dt)
        result[k] = state.as_matrix()
    return result


def euler_degrees(matrices):
    # Intrinsic physical RPY via extrinsic xyz: R = Rz(yaw) Ry(pitch) Rx(roll).
    # Euler angles can be singular; angular error uses rotations, not Euler subtraction.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected")
        angles = Rotation.from_matrix(matrices.reshape(-1, 3, 3)).as_euler("xyz")
    return np.degrees(np.unwrap(angles.reshape(*matrices.shape[:2], 3), axis=0))


def attitude_error_degrees(estimated, reference):
    relative = np.swapaxes(reference, -1, -2) @ estimated
    return np.degrees(Rotation.from_matrix(relative.reshape(-1, 3, 3)).magnitude()).reshape(estimated.shape[:2])
