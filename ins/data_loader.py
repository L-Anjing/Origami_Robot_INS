"""Strict parser for the documented eight-block simulation format."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SimulationData:
    step: np.ndarray
    time: np.ndarray
    quaternion: np.ndarray  # N, 6, 4; wxyz; R_WS
    specific_force: np.ndarray  # N, 6, 3; m/s^2 in S
    gyroscope: np.ndarray  # N, 6, 3; rad/s in S
    controls: np.ndarray  # N, 3; independent binary signals
    endpoints: np.ndarray  # N, 7, 3; evaluation only, never passed to INS


def load_data(path, sample_rate_hz=1000.0):
    steps, nodes, controls, endpoints = [], [], [], []
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                blocks = line.strip().split(";")
                if len(blocks) != 8:
                    raise ValueError(f"expected 8 blocks, got {len(blocks)}")
                parsed_nodes = []
                for node in range(6):
                    tokens = blocks[node].split()
                    expected = 12 if node == 0 else 11
                    if len(tokens) != expected:
                        raise ValueError(f"node {node}: expected {expected} tokens")
                    if node == 0:
                        steps.append(int(tokens.pop(0)))
                    if int(tokens.pop(0)) != node:
                        raise ValueError(f"wrong node ID in block {node}")
                    parsed_nodes.append([float(value) for value in tokens])
                control_tokens = blocks[6].split()
                if len(control_tokens) != 3 or any(t not in ("0", "1") for t in control_tokens):
                    raise ValueError("controls must be three independent binary integers")
                point_tokens = blocks[7].split()
                if len(point_tokens) != 22 or point_tokens[0] != "V":
                    raise ValueError("expected V followed by 21 coordinates")
                points = [float(value) for value in point_tokens[1:]]
                if not np.isfinite(parsed_nodes).all() or not np.isfinite(points).all():
                    raise ValueError("NaN/Inf detected; input was not repaired or skipped")
                nodes.append(parsed_nodes)
                controls.append([int(value) for value in control_tokens])
                endpoints.append(points)
            except (ValueError, OverflowError) as error:
                raise ValueError(f"{path}, line {line_number}: {error}") from error
    step = np.asarray(steps, dtype=np.int64)
    if len(step) < 3:
        raise ValueError("At least three samples are required")
    if step[0] != 1 or not np.all(np.diff(step) == 1):
        raise ValueError("Expected consecutive steps beginning at 1; gaps are not interpolated")
    values = np.asarray(nodes)
    norms = np.linalg.norm(values[:, :, :4], axis=2)
    if np.any(np.abs(norms - 1) > 0.01):
        raise ValueError("Quaternion norm differs from 1 by > 0.01; check input")
    return SimulationData(step, step / sample_rate_hz, values[:, :, :4],
                          values[:, :, 4:7], values[:, :, 7:10],
                          np.asarray(controls, dtype=np.int8),
                          np.asarray(endpoints).reshape(-1, 7, 3))


def describe(data, gravity=9.81):
    """Small machine-readable reliability summary; no sample CSVs."""
    force = np.linalg.norm(data.specific_force, axis=2)
    gyro = np.linalg.norm(data.gyroscope, axis=2)
    states, counts = np.unique(data.controls, axis=0, return_counts=True)
    qnorm = np.linalg.norm(data.quaternion, axis=2)
    return {
        "samples": len(data.time), "time_s": [float(data.time[0]), float(data.time[-1])],
        "sample_rate_hz": float(1 / np.median(np.diff(data.time))),
        "quaternion_norm_min_max": [float(qnorm.min()), float(qnorm.max())],
        "nonfinite_count": 0,  # parser rejects these, including endpoint values
        "force_percentiles_m_s2": np.percentile(force, [0, 50, 90, 99, 100], axis=0).T.tolist(),
        "gyro_percentiles_rad_s": np.percentile(gyro, [0, 50, 90, 99, 100], axis=0).T.tolist(),
        "percentile_order": [0, 50, 90, 99, 100],
        "force_norm_over_16g_samples": np.sum(force > 16 * gravity, axis=0).tolist(),
        "any_force_axis_over_16g_samples": np.sum(np.max(np.abs(data.specific_force), axis=2) > 16 * gravity, axis=0).tolist(),
        "control_states_counts": {"".join(map(str, s)): int(n) for s, n in zip(states, counts)},
        "control_switches": np.count_nonzero(np.diff(data.controls, axis=0), axis=0).tolist(),
        "initial_force_norm_m_s2": force[0].tolist(),
    }
