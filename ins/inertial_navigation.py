"""Pure inertial propagation: this module never receives endpoint truth."""
import numpy as np


def world_acceleration(rotations_ws, specific_force, gravity=9.81):
    return np.einsum("...ij,...j->...i", rotations_ws, specific_force) + np.array([0., 0., -gravity])


def integrate_acceleration(acceleration, time, initial_velocity, method="trapezoid", initial_epoch="first_sample"):
    """Output starts at t[0]; initial velocity epoch must be explicit.

    For the documented step-end acceleration, a[0]=(v(t[0])-v(0))/t[0].
    Thus an initial velocity assumed at simulation start must first be advanced
    to t[0]. No synthetic acceleration at t=0 is invented or taken from truth.
    """
    if method not in ("trapezoid", "step_end"):
        raise ValueError(f"Unknown integration method: {method}")
    velocity = np.empty_like(acceleration)
    velocity[0] = np.broadcast_to(initial_velocity, acceleration.shape[1:])
    if initial_epoch == "simulation_start":
        velocity[0] += acceleration[0] * time[0]
    elif initial_epoch != "first_sample":
        raise ValueError(f"Unknown initial velocity epoch: {initial_epoch}")
    interval_acceleration = acceleration[1:]
    if method == "trapezoid":
        interval_acceleration = (acceleration[:-1] + acceleration[1:]) / 2
    velocity[1:] = velocity[0] + np.cumsum(interval_acceleration * np.diff(time)[:, None, None], axis=0)
    return velocity


def integrate_velocity(velocity, time, method="trapezoid"):
    position = np.zeros_like(velocity)  # displacement in W, metres
    interval_velocity = velocity[1:] if method == "step_end" else (velocity[:-1] + velocity[1:]) / 2
    position[1:] = np.cumsum(interval_velocity * np.diff(time)[:, None, None], axis=0)
    return position
