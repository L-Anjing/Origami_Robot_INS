"""Translation-only weighted least squares with no realized-truth weights."""
import numpy as np


def fuse_reference(position_w, relative_imu_w, acceleration_w, gyroscope, stationary, config):
    """argmin_root sum_i w_i ||p_INS_i - root - r_i(shape)||².

    W-frame attitudes fix the shape orientation. Root attitude is plate0's
    attitude, not an invalid mean of six differently articulated attitudes.
    Weights are heuristic confidence, not calibrated inverse covariance.
    """
    candidates = position_w - relative_imu_w
    score = (1 + (np.linalg.norm(acceleration_w, axis=-1) / config["acceleration_scale_m_s2"])**2 +
             (np.linalg.norm(gyroscope, axis=-1) / config["gyro_scale_rad_s"])**2)
    weights = np.asarray(config["base_weights"]) / score
    weights *= np.where(stationary, config["stationary_weight_multiplier"], 1.)
    weights /= weights.sum(axis=-1, keepdims=True)
    root = np.sum(weights[..., None] * candidates, axis=-2)
    spread = np.sqrt(np.sum(weights * np.sum((candidates - root[..., None, :])**2, axis=-1), axis=-1))
    return root, weights, spread
