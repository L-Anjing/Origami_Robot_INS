"""Causal, fixed-window SHOE/GLRT-style rest hypotheses; no truth or drives."""
import numpy as np


class StationaryDetector:
    def __init__(self, config, dt=.001, gravity=9.81, nodes=6):
        self.config, self.gravity = config, gravity
        self.size = max(2, int(np.ceil(config["window_s"] / dt - 1e-9)))
        self.forces = np.zeros((self.size, nodes, 3))
        self.rates = np.zeros_like(self.forces)
        self.valid = np.zeros((self.size, nodes), dtype=bool)
        self.quiet_duration = np.zeros(nodes)
        self.dt = dt
        self.index, self.count = 0, 0

    def update(self, force, gyro, valid_for_aiding=None):
        validity = np.ones(len(self.quiet_duration), dtype=bool) if valid_for_aiding is None else np.asarray(valid_for_aiding, dtype=bool)
        instant = (validity & (np.linalg.norm(gyro, axis=1) <= self.config["gyro_limit_rad_s"]) &
                   (np.abs(np.linalg.norm(force, axis=1)-self.gravity) <= self.config["force_tolerance_m_s2"]))
        self.quiet_duration = np.where(instant, self.quiet_duration+self.dt, 0.)
        self.forces[self.index] = force
        self.rates[self.index] = gyro
        self.valid[self.index] = validity
        self.index = (self.index + 1) % self.size
        self.count = min(self.count + 1, self.size)
        if self.count < self.size:
            return np.zeros(len(self.quiet_duration), dtype=bool), np.full(len(self.quiet_duration), np.inf)
        mean = self.forces.mean(axis=0)
        expected = self.gravity * mean / np.maximum(np.linalg.norm(mean, axis=1, keepdims=True), 1e-12)
        statistic = np.mean(np.sum((self.forces - expected)**2, axis=2) / self.config["sigma_accel_m_s2"]**2 +
                            np.sum(self.rates**2, axis=2) / self.config["sigma_gyro_rad_s"]**2, axis=0)
        guard = ((np.max(np.linalg.norm(self.rates, axis=2), axis=0) <= self.config["gyro_limit_rad_s"]) &
                 (np.max(np.abs(np.linalg.norm(self.forces, axis=2) - self.gravity), axis=0) <= self.config["force_tolerance_m_s2"]))
        duration_ok = self.quiet_duration + 1e-10 >= self.config.get("minimum_stationary_s", self.size*self.dt)
        return guard & self.valid.all(axis=0) & duration_ok & (statistic < self.config["glrt_threshold"]), statistic


class DrivePhaseTracker:
    """Three independent switches; no drive exclusivity or anchor assignment."""
    def __init__(self, config):
        self.config = config
        self.previous = None
        self.off_age = np.zeros(3)
        self.last_falling = np.full(3, -np.inf)

    def update(self, controls, time, dt):
        controls = np.asarray(controls)
        rising = np.zeros(3, dtype=bool) if self.previous is None else ((self.previous == 0) & (controls == 1))
        falling = np.zeros(3, dtype=bool) if self.previous is None else ((self.previous == 1) & (controls == 0))
        self.off_age = np.where(controls == 0, self.off_age+dt, 0.)
        self.last_falling[falling] = time
        stable_off = (controls == 0) & (self.off_age + 1e-10 >= self.config["stable_off_min_s"])
        elapsed = time-self.last_falling
        after_end = ((controls == 0) & stable_off &
                     (elapsed >= self.config["post_falling_delay_s"]) &
                     (elapsed <= self.config["post_falling_window_s"]))
        self.previous = controls.copy()
        return {"rising": rising, "falling": falling, "stable_off": stable_off,
                "multi_transition": bool(np.count_nonzero(rising | falling) > 1),
                "phase_candidate": bool(np.any(after_end[self.config["relevant_drives"]]))}
