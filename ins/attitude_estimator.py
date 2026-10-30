"""Small SI-unit six-axis complementary AHRS; NumPy only, no files or truth."""
import numpy as np


def quaternion_multiply(a, b):
    w = a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], axis=-1, keepdims=True)
    xyz = a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:] + np.cross(a[..., 1:], b[..., 1:])
    return np.concatenate((w, xyz), axis=-1)


def rotation_increment(vector):
    angle = np.linalg.norm(vector, axis=-1, keepdims=True)
    # sinc avoids a zero-rate branch and division by zero.
    return np.concatenate((np.cos(angle / 2), .5 * np.sinc(angle / (2 * np.pi)) * vector), axis=-1)


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = np.moveaxis(q, -1, 0)
    rows = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
            2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    return np.stack(rows, axis=-1).reshape(*q.shape[:-1], 3, 3)


class SixAxisAHRS:
    """R_WS, wxyz, rad/s and m/s². No north/west assumption and no yaw aid.

    Known initial alignment is supplied externally, not inferred during impacts.
    Accelerometer feedback is enabled only after a continuous norm/innovation
    gate. These are heuristics, not proof that the sensor has zero translation.
    Biases are fixed calibration inputs; this prototype does not estimate them.
    """
    def __init__(self, config, gravity=9.81, method="step_end", backend="six_axis", nodes=6):
        self.config, self.gravity, self.method, self.backend = config, gravity, method, backend
        self.quaternion = np.tile([1., 0., 0., 0.], (nodes, 1))
        self.gyro_bias = np.zeros((nodes, 3))
        self.accel_bias = np.zeros((nodes, 3))
        self.reliable_duration = np.zeros(nodes)
        self.feedback_used = np.zeros(nodes, dtype=bool)
        self.feedback_jacobian = np.tile(np.eye(3), (nodes, 1, 1))
        self.bias_feedback_jacobian = np.zeros((nodes, 3, 3))
        self.previous_gyro = None

    def update(self, gyro, force, dt, feedback_allowed=None):
        omega = np.asarray(gyro) - self.gyro_bias
        if self.method == "trapezoid" and self.previous_gyro is not None:
            interval_omega = .5 * (omega + self.previous_gyro)
        else:
            interval_omega = omega
        self.previous_gyro = omega.copy()
        q = quaternion_multiply(self.quaternion, rotation_increment(interval_omega * dt))
        force = np.asarray(force) - self.accel_bias
        magnitude = np.linalg.norm(force, axis=1)
        direction = force / np.maximum(magnitude[:, None], 1e-12)
        predicted_up_s = quaternion_matrix(q)[:, 2, :]
        angle = np.arccos(np.clip(np.sum(direction * predicted_up_s, axis=1), -1, 1))
        valid = ((np.abs(magnitude - self.gravity) <= self.config["force_tolerance_m_s2"]) &
                 (angle <= np.radians(self.config["innovation_limit_deg"])))
        if feedback_allowed is not None:
            valid &= np.asarray(feedback_allowed, dtype=bool)
        self.reliable_duration = np.where(valid, self.reliable_duration + dt, 0.)
        self.feedback_used = valid & (self.reliable_duration + 1e-10 >= self.config["minimum_reliable_s"])
        if self.backend == "gyro":
            self.feedback_used[:] = False
        correction = self.config["gain_s_inv"] * np.cross(direction, predicted_up_s)
        correction[~self.feedback_used] = 0.
        increment = rotation_increment(correction * dt)
        self.feedback_jacobian[:] = np.eye(3)
        self.bias_feedback_jacobian[:] = 0.
        # Jacobians of this deterministic gravity-feedback map, with gates held
        # fixed. Used by the single-reference ESKF, not by the shape renderer.
        for node in np.flatnonzero(self.feedback_used):
            jr = right_jacobian(correction[node] * dt)
            self.feedback_jacobian[node] = (quaternion_matrix(increment[node]).T +
                jr @ (self.config["gain_s_inv"] * skew(direction[node]) @ skew(predicted_up_s[node])) * dt)
            self.bias_feedback_jacobian[node] = (jr @ (self.config["gain_s_inv"] * skew(predicted_up_s[node]) @
                (np.eye(3) - np.outer(direction[node], direction[node])) / magnitude[node]) * dt)
        q = quaternion_multiply(q, increment)
        self.quaternion = q / np.linalg.norm(q, axis=1, keepdims=True)
        return self.quaternion.copy()


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def right_jacobian(vector):
    angle = np.linalg.norm(vector)
    cross = skew(vector)
    if angle < 1e-6:  # numerical series, not a sensor/detection threshold
        return np.eye(3) - .5 * cross + cross @ cross / 6
    return (np.eye(3) - (1 - np.cos(angle)) / angle**2 * cross +
            (angle - np.sin(angle)) / angle**3 * (cross @ cross))
