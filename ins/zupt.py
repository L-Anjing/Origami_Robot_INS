"""IMU-only stationary candidates and guarded OFFLINE velocity correction."""
import numpy as np


def runs(mask):
    """Half-open [start, stop) runs, including both array boundaries."""
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def detect_stationary(specific_force, gyroscope, time, controls, config, gravity=9.81):
    dt = float(np.median(np.diff(time)))
    count = max(1, int(np.ceil(config["minimum_duration_s"] / dt - 1e-9)))
    force_error = np.abs(np.linalg.norm(specific_force, axis=2) - gravity)
    gyro_norm = np.linalg.norm(gyroscope, axis=2)
    instantaneous = ((gyro_norm <= config["gyro_threshold_rad_s"]) &
                     (force_error <= config["force_norm_tolerance_m_s2"]))
    imu_stationary = np.zeros_like(instantaneous)
    for node in range(6):
        for start, stop in runs(instantaneous[:, node]):
            if stop - start >= count:
                imu_stationary[start:stop, node] = True
    # Any real 1->0 transition is an event. Constant zero at file start isn't one.
    falling = np.diff(controls.astype(int), axis=0) == -1
    event_indices = np.flatnonzero(np.any(falling, axis=1)) + 1
    nearby = np.zeros(len(time), dtype=bool)
    before, after = config["drive_before_s"], config["drive_after_s"]
    for index in event_indices:
        nearby |= (time >= time[index] - before) & (time <= time[index] + after)
    # Timing only selects already-qualified IMU windows; it never proves rest.
    assisted = np.zeros_like(imu_stationary)
    for node in range(6):
        for start, stop in runs(imu_stationary[:, node]):
            if np.any(nearby[start:stop]):
                assisted[start:stop, node] = True
    selected = imu_stationary if config["detector"] == "imu" else assisted
    return {"stationary": selected, "imu_stationary": imu_stationary,
            "drive_stationary": assisted, "drive_nearby": nearby,
            "falling_edges": falling, "events": event_indices,
            "minimum_samples": count}


def correct_velocity(raw_velocity, time, stationary):
    """Correct only moving spans bracketed by qualified stationary runs.

    For each bounded span integrate from v=0 at its left boundary, then subtract
    a linear ramp equal to its residual terminal velocity. This uses FUTURE
    data and is not an online ZUPT-EKF. Unbounded head/tail remain raw, apart
    from the velocity reset after the last observed stationary sample.
    Low angular rate and |f|~g cannot distinguish rest from constant translation;
    these windows remain hypotheses, never externally validated ground truth.
    """
    corrected = raw_velocity.copy()
    segment_counts = []
    for node in range(6):
        windows = runs(stationary[:, node])
        if not windows:
            segment_counts.append(0)
            continue
        for start, stop in windows:
            corrected[start:stop, node] = 0
        for (_, left_stop), (right_start, _) in zip(windows[:-1], windows[1:]):
            left = left_stop - 1
            right = right_start
            local = raw_velocity[left:right + 1, node] - raw_velocity[left, node]
            alpha = (time[left:right + 1] - time[left]) / (time[right] - time[left])
            corrected[left:right + 1, node] = local - alpha[:, None] * local[-1]
        # A final stationary anchor permits forward propagation, but no endpoint ramp.
        last = windows[-1][1] - 1
        corrected[last:, node] = raw_velocity[last:, node] - raw_velocity[last, node]
        segment_counts.append(max(0, len(windows) - 1))
    return corrected, segment_counts
