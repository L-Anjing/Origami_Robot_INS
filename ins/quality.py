"""Measurement flags; simulation samples are NEVER clipped or replaced."""
import numpy as np


def measurement_flags(force, gyro, config, gravity=9.81):
    force, gyro = np.asarray(force), np.asarray(gyro)
    finite = np.isfinite(force).all(axis=-1) & np.isfinite(gyro).all(axis=-1)
    accel_range = np.max(np.abs(force), axis=-1) > config["accelerometer_range_g"] * gravity
    gyro_range = np.max(np.abs(gyro), axis=-1) > np.radians(config["gyroscope_range_deg_s"])
    impact = np.linalg.norm(force, axis=-1) > config["impact_force_norm_g"] * gravity
    return {"finite": finite, "accel_over_range": accel_range,
            "gyro_over_range": gyro_range, "impact": impact,
            "valid_for_aiding": finite & ~accel_range & ~gyro_range & ~impact}
