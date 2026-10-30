"""Independent evaluation; endpoint truth never modifies propagated states."""
import numpy as np


def reference_positions(endpoints, endpoint_indices):
    reference = np.full((len(endpoints), 6, 3), np.nan)
    for node, point in enumerate(endpoint_indices):
        if point is not None:
            reference[:, node] = endpoints[:, point]
    return reference


def evaluate(position, velocity, reference, time, velocity_method):
    """Relative p(t)-p(t_first) comparison, explicitly aligned using truth p_first.

    Position RMSE is sqrt(mean(||error||^2)), not per-axis RMSE. Under the
    documented step-end simulator convention, delta p[k]/dt approximates v[k].
    Central differences are optional for continuous-time/smoothed truth. Neither
    method smooths this dataset or supplies any correction to INS.
    """
    result = []
    for node in range(6):
        if not np.isfinite(reference[:, node]).all():
            result.append(None)
            continue
        truth = reference[:, node]
        error = position[:, node] - (truth - truth[0])
        if velocity_method == "step_end_difference":
            truth_velocity = np.diff(truth, axis=0) / np.diff(time)[:, None]
            # First recorded velocity cannot be differenced without p(t=0).
            velocity_error = velocity[1:, node] - truth_velocity
        elif velocity_method == "central_difference":
            truth_velocity = np.gradient(truth, time, axis=0, edge_order=2)
            velocity_error = velocity[1:-1, node] - truth_velocity[1:-1]
        else:
            raise ValueError(f"Unknown truth velocity method: {velocity_method}")
        result.append({
            "position_rmse_m": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
            "final_position_error_m": float(np.linalg.norm(error[-1])),
            "velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum(velocity_error**2, axis=1)))),
            "velocity_method": velocity_method,
            "evaluation_alignment": "truth first-position translation only; no fit, no state feedback",
        })
    return result


def sampling_consistency(acceleration_w, endpoints, endpoint_indices, time):
    """Read-only check of backward velocity differencing when mapping is known.

    Under semi-implicit p[k]=p[k-1]+v[k]dt, a[k]=(v[k]-v[k-1])/dt.
    This is a diagnostic of that discretization, not an assumed new definition.
    """
    dt = np.diff(time)
    result = []
    for node, point in enumerate(endpoint_indices):
        if point is None:
            result.append(None)
            continue
        velocity_fd = np.diff(endpoints[:, point], axis=0) / dt[:, None]
        accel_fd = np.diff(velocity_fd, axis=0) / dt[1:, None]
        residual = acceleration_w[2:, node] - accel_fd
        result.append({"step_end_acceleration_residual_rmse_m_s2": float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))})
    return result
