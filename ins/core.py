"""Single-reference causal INS / 15-state ESKF. NumPy only; no geometry or files."""
import numpy as np

from .attitude_estimator import (SixAxisAHRS, quaternion_matrix, quaternion_multiply,
                                 right_jacobian, rotation_increment, skew)


class INS:
    """Nominal p,v,q,bg,ba; local right error [dp,dv,dtheta,dbg,dba].

    Raw and ESKF share exactly the same propagation. covariance=False disables
    only the error covariance and measurement updates. The six-node shape AHRS
    is a separate chain and is never changed by this filter's ZUPT injection.
    Step-end p[k]=p[k-1]+v[k]dt matches the documented simulator convention.
    Real IMU recordings select constant_acceleration: p += v*dt + a*dt²/2.
    """
    def __init__(self, position, velocity, ahrs_config, filter_config,
                 gravity=9.81, backend="six_axis", covariance=True,
                 prescribed_orientation=False, position_integration="step_end"):
        if position_integration not in ("step_end", "constant_acceleration"):
            raise ValueError("Unknown position integration method")
        self.position_integration = position_integration
        self.position = np.array(position, dtype=float, copy=True)
        self.velocity = np.array(velocity, dtype=float, copy=True)
        if self.position.shape != (3,) or self.velocity.shape != (3,):
            raise ValueError("Single-reference p,v must have shape (3,)")
        self.config = filter_config
        self.attitude = SixAxisAHRS(ahrs_config, gravity, "step_end", backend, nodes=1)
        self.bg = np.array(filter_config["initial_gyro_bias_rad_s"], dtype=float)
        self.ba = np.array(filter_config["initial_accel_bias_m_s2"], dtype=float)
        self.quaternion = np.array([1., 0., 0., 0.])
        self.gravity = np.array([0., 0., -gravity])
        self.prescribed_orientation = prescribed_orientation
        self.P = None
        if covariance:
            c = filter_config["initial_std"]
            sigma = np.repeat([c["position_m"], c["velocity_m_s"],
                               np.radians(c["attitude_deg"]),
                               c["gyro_bias_rad_s"], c["accel_bias_m_s2"]], 3)
            self.P = np.diag(sigma**2)
            if prescribed_orientation:
                self.P[6:12, :] = 0.
                self.P[:, 6:12] = 0.
        self.acceleration = np.zeros(3)
        self.last_transition = np.eye(15)
        self.last_injection = np.zeros(15)

    def propagate(self, gyro, specific_force, dt, feedback_allowed=True,
                  noise_multiplier=1., quaternion_debug=None):
        gyro, force = np.asarray(gyro), np.asarray(specific_force)
        if gyro.shape != (3,) or force.shape != (3,) or not np.isfinite([gyro, force]).all():
            raise ValueError("Expected finite single-IMU gyro and specific force")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")
        if self.prescribed_orientation != (quaternion_debug is not None):
            raise ValueError("Prescribed quaternion allowed ONLY in explicit ground-truth debug mode")
        omega, corrected_force = gyro - self.bg, force - self.ba
        self.attitude.quaternion[0] = self.quaternion
        self.attitude.gyro_bias[0], self.attitude.accel_bias[0] = self.bg, self.ba
        if self.prescribed_orientation:
            q = np.asarray(quaternion_debug, dtype=float)
            if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
                raise ValueError("Invalid debug quaternion")
            self.quaternion = q / np.linalg.norm(q)
            self.attitude.feedback_used[:] = False
        else:
            self.quaternion = self.attitude.update(
                gyro[None], force[None], dt, [feedback_allowed])[0]
        rotation = quaternion_matrix(self.quaternion)
        self.acceleration = rotation @ corrected_force + self.gravity
        position_factor = 1. if self.position_integration == "step_end" else .5
        self.position += self.velocity * dt + position_factor * self.acceleration * dt**2
        self.velocity += self.acceleration * dt
        if self.P is not None:
            jr = right_jacobian(omega * dt)
            if self.prescribed_orientation:
                a_theta = b_theta = c_theta = np.zeros((3, 3))
                gyro_noise_theta = accel_noise_theta = np.zeros((3, 3))
            else:
                jf = self.attitude.feedback_jacobian[0]
                a_theta = jf @ quaternion_matrix(rotation_increment(omega * dt)).T
                b_theta = -jf @ jr * dt
                c_theta = self.attitude.bias_feedback_jacobian[0]
                gyro_noise_theta = -jf @ jr
                # The same accelerometer drives feedback and propagation.
                accel_noise_theta = c_theta / dt
            sensitivity = -rotation @ skew(corrected_force)
            phi = np.eye(15)
            phi[0:3, 3:6] = np.eye(3) * dt
            phi[6:9, 6:9], phi[6:9, 9:12], phi[6:9, 12:15] = a_theta, b_theta, c_theta
            phi[3:6, 6:9] = sensitivity @ a_theta * dt
            phi[3:6, 9:12] = sensitivity @ b_theta * dt
            phi[3:6, 12:15] = (sensitivity @ c_theta - rotation) * dt
            phi[0:3, 6:15] = position_factor * phi[3:6, 6:15] * dt
            gain = np.zeros((15, 12))
            gain[6:9, 0:3], gain[6:9, 3:6] = gyro_noise_theta, accel_noise_theta
            gain[3:6, 0:3] = sensitivity @ gyro_noise_theta * dt
            gain[3:6, 3:6] = sensitivity @ accel_noise_theta * dt - rotation
            gain[0:3, :6] = position_factor * gain[3:6, :6] * dt
            gain[9:12, 6:9], gain[12:15, 9:12] = np.eye(3), np.eye(3)
            if self.prescribed_orientation:
                phi[6:12, :] = 0.
                gain[6:12, :] = 0.  # known attitude/gyro bias are frozen in debug A
            noise = self.config["noise_density"]
            density = np.repeat([noise["gyro_rad_s_sqrt_hz"], noise["accel_m_s2_sqrt_hz"],
                                 noise["gyro_bias_random_walk"], noise["accel_bias_random_walk"]], 3)
            qd = density**2 * dt * noise_multiplier**2
            self.P = phi @ self.P @ phi.T + (gain * qd[None, :]) @ gain.T
            self.P = .5 * (self.P + self.P.T)
            self.last_transition = phi
            if not np.isfinite(self.P).all():
                raise FloatingPointError("Nonfinite ESKF covariance")
        return self.snapshot()

    def zupt_update(self, candidate, measurement_valid=True, max_position_correction_m=None):
        """z=0, h(x)=v; optionally guard the inferred position correction.

        If guarded, set the position rows of K to zero BEFORE the Joseph update.
        Velocity, attitude and bias rows still use the accepted zero velocity.
        Position uncertainty is retained rather than shrunk for an unapplied dx.
        """
        if max_position_correction_m is not None and (
                not np.isfinite(max_position_correction_m) or max_position_correction_m < 0):
            raise ValueError("Position correction limit must be finite and nonnegative")
        if not candidate or not measurement_valid or self.P is None:
            return {"accepted": False, "nis": 0., "rejected_nis": False}
        h = np.zeros((3, 15)); h[:, 3:6] = np.eye(3)
        r = np.eye(3) * self.config["zupt_velocity_std_m_s"]**2
        innovation = -self.velocity
        innovation_cov = self.P[3:6, 3:6] + r
        nis = float(innovation @ np.linalg.solve(innovation_cov, innovation))
        if not np.isfinite(nis) or nis > self.config["zupt_nis_limit"]:
            return {"accepted": False, "nis": nis, "rejected_nis": True}
        k = np.linalg.solve(innovation_cov, h @ self.P).T
        proposed_position = (k @ innovation)[:3]
        proposed_norm = float(np.linalg.norm(proposed_position))
        position_rejected = max_position_correction_m is not None and proposed_norm > max_position_correction_m
        if position_rejected:
            k[:3, :] = 0.
        dx = k @ innovation
        identity_minus_kh = np.eye(15) - k @ h
        posterior = identity_minus_kh @ self.P @ identity_minus_kh.T + k @ r @ k.T
        self.position += dx[:3]
        self.velocity += dx[3:6]  # not a hard velocity reset
        self.quaternion = quaternion_multiply(self.quaternion, rotation_increment(dx[6:9]))
        self.quaternion /= np.linalg.norm(self.quaternion)
        self.bg += dx[9:12]
        self.ba += dx[12:15]
        self.attitude.quaternion[0] = self.quaternion
        reset = np.eye(15)
        reset[6:9, 6:9] = right_jacobian(dx[6:9])
        self.P = reset @ posterior @ reset.T
        self.P = .5 * (self.P + self.P.T)
        self.last_injection = dx
        return {"accepted": True, "nis": nis, "rejected_nis": False,
                "position_update_rejected": bool(position_rejected),
                "proposed_position_correction_m": proposed_norm,
                "proposed_position_correction": proposed_position.copy()}

    def snapshot(self):
        return {"position": self.position.copy(), "velocity": self.velocity.copy(),
                "quaternion": self.quaternion.copy(), "bg": self.bg.copy(), "ba": self.ba.copy(),
                "acceleration": self.acceleration.copy(),
                "covariance_diagonal": None if self.P is None else np.diag(self.P).copy()}
