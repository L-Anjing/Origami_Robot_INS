"""Separate physical propagation, estimator corrections and pose uncertainty."""
import math

import numpy as np


class INSHealthMonitor:
    """Report continuity loss without clipping states or discarding observations.

    Limits describe application tolerances, not statistically calibrated fault
    probabilities. A discontinuity stays latched until the estimator is reset.
    """
    DEFAULTS = {"max_speed_m_s": 2., "step_tolerance_m": .02,
                "max_position_correction_m": .10, "max_position_std_m": .50}

    def __init__(self, config=None):
        self.config = dict(self.DEFAULTS)
        self.config.update(config or {})
        for name, value in self.config.items():
            if name not in self.DEFAULTS or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Invalid INS health limit: {name}")
        self.anomaly_count = 0
        self.max_correction_seen_m = 0.
        self.last_anomaly_ns = None
        self.rejected_position_updates = 0
        self.max_proposed_correction_m = 0.

    def observe(self, timestamp_ns, previous_position, predicted_position, corrected_position, covariance, dt,
                position_update_rejected=False, proposed_position_correction_m=0.):
        correction = float(np.linalg.norm(corrected_position-predicted_position))
        step = motion_step = 0.
        limit = self.config["step_tolerance_m"]
        if previous_position is not None:
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError("Health monitor requires a positive sample interval")
            step = float(np.linalg.norm(corrected_position-previous_position))
            motion_step = float(np.linalg.norm(predicted_position-previous_position))
            limit += self.config["max_speed_m_s"]*dt
        sigma = float(np.sqrt(max(0., np.max(np.linalg.eigvalsh(covariance[:3, :3])))))
        large_correction = correction > self.config["max_position_correction_m"]
        excessive_motion = previous_position is not None and motion_step > limit
        discontinuity = previous_position is not None and (step > limit or large_correction)
        anomaly = bool(discontinuity or excessive_motion)
        if anomaly:
            self.anomaly_count += 1
            self.last_anomaly_ns = timestamp_ns
        self.max_correction_seen_m = max(self.max_correction_seen_m, correction)
        self.rejected_position_updates += int(position_update_rejected)
        self.max_proposed_correction_m = max(self.max_proposed_correction_m, proposed_position_correction_m)
        return {"position_step_m": step, "propagated_step_m": motion_step,
                "position_correction_m": correction, "position_step_limit_m": limit,
                "position_std_m": sigma, "position_discontinuity": bool(discontinuity),
                "excessive_propagation": bool(excessive_motion),
                "large_position_correction": bool(large_correction),
                "position_uncertain": sigma > self.config["max_position_std_m"],
                "anomaly": anomaly, "continuity_lost": self.anomaly_count > 0,
                "anomaly_count": self.anomaly_count, "last_anomaly_ns": self.last_anomaly_ns,
                "max_correction_seen_m": self.max_correction_seen_m,
                "position_update_rejected": bool(position_update_rejected),
                "position_update_rejections": self.rejected_position_updates,
                "proposed_position_correction_m": proposed_position_correction_m,
                "max_proposed_correction_m": self.max_proposed_correction_m,
                "estimate_unreliable": self.anomaly_count > 0 or self.rejected_position_updates > 0}
