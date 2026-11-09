"""Timestamped single-IMU navigation, shared by ROS 2 and offline checks.

Matches test/test_real_imu.m: static alignment, causal time-window GLRT,
constant-acceleration position integration and ESKF zero-velocity updates.
Device quaternions are retained for inspection, never used as attitude aids.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np

from .core import INS
from .health import INSHealthMonitor


@dataclass(frozen=True)
class IMUSample:
    timestamp_ns: int
    force: np.ndarray
    gyro: np.ndarray
    device_quaternion: np.ndarray | None = None


def read_recording(path):
    """Strict recorder-format parser; subtract integer ns before float conversion."""
    samples = []
    previous_step, previous_stamp = 0, -1
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                blocks = line.split(";")
                if len(blocks) != 2:
                    raise ValueError("expected one IMU block and one T block")
                tokens = blocks[0].split()
                stamp_match = re.fullmatch(r"T\s+([0-9]{1,19})", blocks[1].strip())
                if len(tokens) != 12 or not stamp_match:
                    raise ValueError("invalid IMU fields or integer nanosecond timestamp")
                step, sensor = int(tokens[0]), int(tokens[1])
                values = np.array([float(v) for v in tokens[2:]])
                stamp = int(stamp_match[1])
                if step <= previous_step or sensor != 0 or stamp <= previous_stamp:
                    raise ValueError("steps/timestamps must increase and sensor ID must be 0")
                if not np.isfinite(values).all() or abs(np.linalg.norm(values[:4])-1) >= .01:
                    raise ValueError("nonfinite measurement or invalid device quaternion")
                if stamp // 10**9 > 2**31-1:
                    raise ValueError("timestamp exceeds ROS builtin_interfaces/Time range")
                samples.append(IMUSample(stamp, values[4:7], values[7:10],
                                         values[:4]/np.linalg.norm(values[:4])))
                previous_step, previous_stamp = step, stamp
            except (ValueError, OverflowError) as error:
                raise ValueError(f"{path}, line {line_number}: {error}") from error
    if len(samples) < 3:
        raise ValueError("At least three IMU samples are required")
    return samples


class TimeWindowDetector:
    """Causal GLRT with durations measured from actual integer timestamps."""
    def __init__(self, config, gravity):
        self.config, self.gravity = config, gravity
        self.window = deque()
        self.quiet_since = None

    def update(self, timestamp_ns, force, gyro, valid, gap_limit_ns):
        c = self.config
        quiet = (valid and np.linalg.norm(gyro) <= c["gyro_limit_rad_s"] and
                 abs(np.linalg.norm(force)-self.gravity) <= c["force_tolerance_m_s2"])
        if not quiet:
            self.quiet_since = None
        elif (self.quiet_since is None or
              (self.window and timestamp_ns-self.window[-1][0] > gap_limit_ns)):
            self.quiet_since = timestamp_ns
        self.window.append((timestamp_ns, force.copy(), gyro.copy(), quiet))
        cutoff = timestamp_ns - round(c["window_s"]*1e9)
        while len(self.window) > 1 and self.window[1][0] <= cutoff:
            self.window.popleft()
        if len(self.window) < 3 or self.window[0][0] > cutoff:
            return False, float("inf")
        forces = np.array([v[1] for v in self.window])
        gyros = np.array([v[2] for v in self.window])
        mean = forces.mean(axis=0)
        expected = self.gravity*mean/max(np.linalg.norm(mean), 1e-12)
        statistic = float(np.mean(np.sum((forces-expected)**2, axis=1)/c["sigma_accel_m_s2"]**2 +
                                  np.sum(gyros**2, axis=1)/c["sigma_gyro_rad_s"]**2))
        stationary = (self.quiet_since is not None and all(v[3] for v in self.window) and
                      timestamp_ns-self.quiet_since >= round(c["minimum_stationary_s"]*1e9) and
                      max(b[0]-a[0] for a, b in zip(self.window, list(self.window)[1:])) <= gap_limit_ns and
                      statistic < c["glrt_threshold"])
        return bool(stationary), statistic


class RealIMUNavigator:
    def __init__(self, config, initial_static_seconds=2., zupt_mode="glrt"):
        if not np.isfinite(initial_static_seconds) or initial_static_seconds <= 0:
            raise ValueError("initial_static_seconds must be positive")
        if zupt_mode not in ("glrt", "none"):
            raise ValueError("zupt_mode must be glrt or none")
        self.config = deepcopy(config)
        self.max_integration_interval_s = float(config.get("real_imu", {}).get("max_integration_interval_s", .25))
        if not np.isfinite(self.max_integration_interval_s) or self.max_integration_interval_s <= 0:
            raise ValueError("max_integration_interval_s must be positive")
        self.initial_static_seconds, self.zupt_mode = initial_static_seconds, zupt_mode
        self.pending = []
        self.raw = self.eskf = None
        self.previous_input_ns = self.previous_ns = None
        self.first_ns = None
        self.intervals = deque(maxlen=100)
        self.last_attempt_ns = None
        self.samples = self.accepted = self.rejected = self.gaps = 0
        self.minimum_eigenvalue = float("inf")
        self.health = INSHealthMonitor(config.get("real_imu", {}).get("health"))

    def push(self, sample):
        """Returns no states during calibration, then buffered states and one per input."""
        if (sample.force.shape != (3,) or sample.gyro.shape != (3,) or
                not np.isfinite([sample.force, sample.gyro]).all()):
            raise ValueError("Expected finite single-IMU acceleration and angular rate")
        if self.previous_input_ns is not None and sample.timestamp_ns <= self.previous_input_ns:
            raise ValueError("IMU timestamps must be strictly increasing")
        if self.previous_input_ns is not None:
            interval_s = (sample.timestamp_ns-self.previous_input_ns)*1e-9
            if interval_s > self.max_integration_interval_s:
                raise ValueError(f"IMU timestamp gap {interval_s:.3f} s exceeds "
                                 f"{self.max_integration_interval_s:g} s; integration suspended. "
                                 "Reset and perform static alignment before resuming")
        self.previous_input_ns = sample.timestamp_ns
        if self.eskf is None:
            self.pending.append(sample)
            elapsed_ns = sample.timestamp_ns-self.pending[0].timestamp_ns
            if elapsed_ns < round(self.initial_static_seconds*1e9):
                return []
            self._initialize()
            pending, self.pending = self.pending, []
            return [self._step(item) for item in pending]
        return [self._step(sample)]

    def _initialize(self):
        c = self.config
        g = c["gravity_m_s2"]
        detector = c["robot"]["stationary"]
        first = self.pending[0].timestamp_ns
        calibration = [s for s in self.pending if
                       s.timestamp_ns-first <= round(self.initial_static_seconds*1e9)]
        if len(calibration) < 10:
            raise ValueError("Initial static interval requires at least 10 samples")
        forces, gyros = np.array([s.force for s in calibration]), np.array([s.gyro for s in calibration])
        mean, bg = forces.mean(axis=0), gyros.mean(axis=0)
        if not (abs(np.linalg.norm(mean)-g) <= detector["force_tolerance_m_s2"] and
                np.max(forces.std(axis=0, ddof=1)) < detector["sigma_accel_m_s2"] and
                np.max(gyros.std(axis=0, ddof=1)) < detector["sigma_gyro_rad_s"] and
                np.linalg.norm(bg) < detector["gyro_limit_rad_s"]):
            raise ValueError("Initial interval is not static; start with a stationary IMU")
        intervals = [b.timestamp_ns-a.timestamp_ns for a, b in zip(calibration, calibration[1:])]
        if detector["window_s"]*1e9 < 2*np.median(intervals):
            raise ValueError("Stationary window must cover at least two sample intervals")
        self.intervals.extend(intervals)
        up = mean/np.linalg.norm(mean)
        roll, pitch = np.arctan2(up[1], up[2]), np.arctan2(-up[0], np.hypot(up[1], up[2]))
        cr, sr, cp, sp = np.cos(roll/2), np.sin(roll/2), np.cos(pitch/2), np.sin(pitch/2)
        q = np.array([cr*cp, sr*cp, cr*sp, -sr*sp])
        ba = mean-g*up
        ekf_config = c["robot"]["eskf"]
        ekf_config["initial_gyro_bias_rad_s"] = bg.tolist()
        ekf_config["initial_accel_bias_m_s2"] = ba.tolist()
        for covariance in (False, True):
            state = INS(np.zeros(3), np.zeros(3), c["robot"]["ahrs"], ekf_config,
                        gravity=g, covariance=covariance, position_integration="constant_acceleration")
            state.quaternion = q.copy()
            if covariance:
                self.eskf = state
            else:
                self.raw = state
        self.detector = TimeWindowDetector(detector, g)
        self.first_ns = first
        self.initialization = {"samples": len(calibration), "gyro_bias": bg, "accel_bias": ba,
                               "quaternion": q}

    def _step(self, sample):
        c, g = self.config["robot"], self.config["gravity_m_s2"]
        force = sample.force-self.initialization["accel_bias"]
        gyro = sample.gyro-self.initialization["gyro_bias"]
        impact = np.linalg.norm(force) > c["quality"]["impact_force_norm_g"]*g
        valid = not (impact or
                     np.max(np.abs(sample.force)) >= c["quality"]["accelerometer_range_g"]*g or
                     np.max(np.abs(sample.gyro)) >= np.radians(c["quality"]["gyroscope_range_deg_s"]))
        gap_limit_ns = 5*float(np.median(self.intervals))
        stationary, statistic = self.detector.update(sample.timestamp_ns, force, gyro, valid, gap_limit_ns)
        update = {"accepted": False, "rejected_nis": False, "nis": float("nan")}
        gap, requested = False, False
        previous_position, dt = None, 0.
        predicted_position = self.eskf.position.copy()
        zupt_prior = None
        if self.previous_ns is not None:
            previous_position = self.eskf.position.copy()
            interval_ns = sample.timestamp_ns-self.previous_ns
            gap = interval_ns > gap_limit_ns
            self.gaps += int(gap)
            self.intervals.append(interval_ns)
            dt = interval_ns*1e-9
            self.raw.propagate(sample.gyro, sample.force, dt, valid)
            self.eskf.propagate(sample.gyro, sample.force, dt, valid,
                                1. if valid else c["quality"]["impact_noise_multiplier"])
            predicted_position = self.eskf.position.copy()
            requested = (self.zupt_mode == "glrt" and stationary and
                         (self.last_attempt_ns is None or sample.timestamp_ns-self.last_attempt_ns >=
                          round(c["eskf"]["zupt_min_interval_s"]*1e9)))
            if requested:
                self.last_attempt_ns = sample.timestamp_ns
                zupt_prior = {
                    "position": self.eskf.position.copy(), "velocity": self.eskf.velocity.copy(),
                    "position_covariance": self.eskf.P[:3, :3].copy(),
                    "position_velocity_covariance": self.eskf.P[:3, 3:6].copy(),
                    "velocity_innovation_covariance": self.eskf.P[3:6, 3:6].copy() +
                    np.eye(3)*c["eskf"]["zupt_velocity_std_m_s"]**2}
                correction_limit = None
                if self.config.get("real_imu", {}).get("protect_position_updates", False):
                    limits = self.health.config
                    step_budget = limits["step_tolerance_m"] + limits["max_speed_m_s"]*dt
                    propagated_step = float(np.linalg.norm(predicted_position-previous_position))
                    correction_limit = min(limits["max_position_correction_m"],
                                           max(0., step_budget-propagated_step))
                update = self.eskf.zupt_update(True, valid, max_position_correction_m=correction_limit)
        health = self.health.observe(sample.timestamp_ns, previous_position, predicted_position,
                                     self.eskf.position, self.eskf.P, dt,
                                     update.get("position_update_rejected", False),
                                     update.get("proposed_position_correction_m", 0.))
        self.previous_ns = sample.timestamp_ns
        self.samples += 1
        self.accepted += int(update["accepted"])
        self.rejected += int(update["rejected_nis"])
        if self.samples % c["eskf"]["covariance_check_every"] == 0:
            self.check_covariance()
        return {"timestamp_ns": sample.timestamp_ns, "raw": self.raw.snapshot(),
                "eskf": self.eskf.snapshot(), "covariance": self.eskf.P.copy(),
                "stationary": stationary, "glrt": statistic, "valid": valid,
                "impact": bool(impact), "gap": gap, "zupt_requested": bool(requested),
                "health": health, "zupt_prior": zupt_prior, **update}

    def check_covariance(self):
        minimum = float(np.linalg.eigvalsh(self.eskf.P)[0])
        self.minimum_eigenvalue = min(self.minimum_eigenvalue, minimum)
        if minimum < -self.config["robot"]["eskf"]["covariance_negative_tolerance"]:
            raise FloatingPointError("ESKF covariance is not positive semidefinite")
        return minimum
