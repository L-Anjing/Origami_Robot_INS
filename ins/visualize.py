"""Static PNGs only. Plotting never alters any estimator array."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .orientation import euler_degrees
from .zupt import runs

COLORS = plt.get_cmap("tab10").colors


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def metric_axes(axis):
    """Equal geometric scale for metre axes; never flip the world directions."""
    limits = np.array([axis.get_xlim3d(), axis.get_ylim3d(), axis.get_zlim3d()])
    centers = limits.mean(axis=1)
    half = max(float(np.ptp(limits, axis=1).max()) / 2, .001)
    # Equal limits in a cubic display, rather than a millimetre-thin axes box
    # with overlapping Y/Z ticks for the long, narrow crack trajectory.
    axis.set_xlim3d(centers[0] - half, centers[0] + half)
    axis.set_ylim3d(centers[1] - half, centers[1] + half)
    axis.set_zlim3d(centers[2] - half, centers[2] + half)
    axis.set_box_aspect((1, 1, 1))
    axis.tick_params(labelsize=7, pad=1)
    for coordinate in (axis.xaxis, axis.yaxis, axis.zaxis):
        coordinate.label.set_size(9)
        coordinate.labelpad = 2


def plot_common(data, detection, truth_rotations, estimated_rotations, output):
    output = Path(output)
    fig, axes = plt.subplots(6, 2, figsize=(13, 12), sharex=True)
    for node in range(6):
        axes[node, 0].plot(data.time, np.linalg.norm(data.specific_force[:, node], axis=1), lw=.6)
        axes[node, 0].axhline(9.81, color="k", ls=":", lw=.7)
        axes[node, 1].plot(data.time, np.linalg.norm(data.gyroscope[:, node], axis=1), lw=.6)
        axes[node, 0].set_ylabel(f"S{node}: |f| (m/s²)")
        axes[node, 1].set_ylabel(f"S{node}: |omega| (rad/s)")
    for axis in axes.flat:
        axis.grid(alpha=.25)
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    fig.suptitle("Raw measurements: no clipping, no filtering")
    save(fig, output / "sensor_norms.png")

    fig, axes = plt.subplots(3, 1, figsize=(13, 5), sharex=True)
    for drive, axis in enumerate(axes):
        axis.step(data.time, data.controls[:, drive], where="post")
        axis.set_ylabel(f"u{drive}")
        axis.set_yticks([0, 1])
        axis.grid(alpha=.25)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Three independent drive signals (overlap allowed)")
    save(fig, output / "drive_signals.png")

    fig, axes = plt.subplots(7, 1, figsize=(13, 10), sharex=True)
    for drive in range(3):
        axes[0].step(data.time, data.controls[:, drive] + drive * 1.3, where="post", label=f"u{drive}")
    axes[0].legend(loc="upper right", ncol=3)
    axes[0].set_ylabel("Drives (offset)")
    for node in range(6):
        axis = axes[node + 1]
        axis.fill_between(data.time, 0, 1, where=detection["drive_nearby"], color="gold", alpha=.4, label="Drive-off neighborhood")
        axis.step(data.time, detection["imu_stationary"][:, node].astype(int), where="post", label="IMU candidate")
        axis.step(data.time, detection["drive_stationary"][:, node].astype(int) * .8, where="post", label="Drive-assisted subset")
        axis.set_ylim(-.1, 1.1)
        axis.set_ylabel(f"S{node}")
        axis.grid(alpha=.2)
    axes[1].legend(loc="upper right", ncol=3, fontsize=8)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Stationary hypotheses: drive timing alone never sets zero velocity")
    save(fig, output / "stationary_detection.png")

    truth_angles = euler_degrees(truth_rotations)
    estimated_angles = euler_degrees(estimated_rotations)
    fig, axes = plt.subplots(6, 3, figsize=(15, 13), sharex=True)
    for node in range(6):
        for component, name in enumerate(("Roll", "Pitch", "Yaw")):
            axis = axes[node, component]
            axis.plot(data.time, truth_angles[:, node, component], lw=.8, label="Truth")
            axis.plot(data.time, estimated_angles[:, node, component], lw=.7, ls="--", label="Gyro estimate")
            axis.set_ylabel(f"S{node} {name} (deg)")
            axis.grid(alpha=.25)
    axes[0, 0].legend()
    for axis in axes[-1]:
        axis.set_xlabel("Time (s)")
    fig.suptitle("R_WS Euler xyz; unwrapped for display; no absolute yaw observation")
    save(fig, output / "orientation_rpy.png")

    # Independent endpoint reference; these are not labeled as IMU positions.
    fig = plt.figure(figsize=(10, 7))
    axis = fig.add_subplot(projection="3d")
    for point in range(7):
        xyz = data.endpoints[:, point]
        axis.plot(*xyz.T, lw=.7, label=f"V{point}")
    axis.set(xlabel="X_W (m)", ylabel="Y_W (m)", zlabel="Z_W (m)",
             title="Endpoint world truth only: overall motion toward -X_W")
    axis.legend()
    metric_axes(axis)
    save(fig, output / "endpoints_reference.png")


def plot_run(time, position, velocity, raw_position, reference, run_name, output):
    output = Path(output)
    fig = plt.figure(figsize=(11, 8))
    axis = fig.add_subplot(projection="3d")
    for node in range(6):
        xyz = position[:, node]
        axis.plot(*xyz.T, color=COLORS[node], lw=.8, label=f"S{node}")
        axis.scatter(*xyz[-1], color=COLORS[node], marker="x", s=20)
    axis.set(xlabel="Delta X_W (m)", ylabel="Delta Y_W (m)", zlabel="Delta Z_W (m)",
             title=f"{run_name}: six relative INS trajectories (first sample = 0)")
    axis.legend()
    metric_axes(axis)
    # Each node has its own untranslated displacement origin. No invented geometry.
    save(fig, output / "trajectories_3d.png")

    fig, axes = plt.subplots(6, 3, figsize=(15, 13), sharex=True)
    for node in range(6):
        for component, name in enumerate("XYZ"):
            axis = axes[node, component]
            axis.plot(time, raw_position[:, node, component], color=".55", lw=.8, label="Raw INS")
            axis.plot(time, position[:, node, component], lw=.8, label="Selected INS")
            if np.isfinite(reference[:, node]).all():
                truth_relative = reference[:, node] - reference[0, node]
                axis.plot(time, truth_relative[:, component], "k--", lw=.8, label="Confirmed truth (relative)")
            axis.set_ylabel(f"S{node} Delta {name}_W (m)")
            axis.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("Time (s)")
    fig.suptitle(run_name + ": position components; no rotation or trajectory fit")
    save(fig, output / "position_xyz.png")

    fig, axes = plt.subplots(6, 1, figsize=(12, 11), sharex=True)
    for node, axis in enumerate(axes):
        axis.plot(time, np.linalg.norm(velocity[:, node], axis=1), lw=.7)
        axis.set_ylabel(f"S{node} |v| (m/s)")
        axis.grid(alpha=.25)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(run_name + ": speed")
    save(fig, output / "speed.png")

    known = [node for node in range(6) if np.isfinite(reference[:, node]).all()]
    if known:
        fig = plt.figure(figsize=(14, 9))
        for index, node in enumerate(known):
            axis = fig.add_subplot(2, 3, index + 1, projection="3d")
            axis.plot(*reference[:, node].T, "k", lw=.8, label="Confirmed IMU truth")
            # Truth used ONLY here to translate relative INS to W for display.
            axis.plot(*(position[:, node] + reference[0, node]).T, lw=.8, label="INS + truth p(first)")
            axis.set(title=f"S{node}", xlabel="X_W (m)", ylabel="Y_W (m)", zlabel="Z_W (m)")
            axis.legend(fontsize=7)
            metric_axes(axis)
        fig.suptitle(run_name + ": world-coordinate comparison; first-position translation only")
        save(fig, output / "truth_comparison.png")


def plot_comparison(time, results, reference, output):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), sharex=True)
    for node, axis in enumerate(axes.flat):
        known = np.isfinite(reference[:, node]).all()
        for name, position in results.items():
            value = position[:, node]
            if known:
                value = value - (reference[:, node] - reference[0, node])
            axis.plot(time, np.linalg.norm(value, axis=1), lw=.8, label=name)
        axis.set_title(f"S{node}: " + ("position error" if known else "INS displacement; truth mapping unknown"))
        axis.set(xlabel="Time (s)", ylabel="m")
        axis.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Selected run comparison (overlapping curves can be identical)")
    save(fig, Path(output) / "comparison.png")
