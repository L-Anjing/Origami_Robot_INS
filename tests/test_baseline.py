"""Physical invariants, discrete timing, strict parsing, and ZUPT boundaries."""
import tempfile
from pathlib import Path
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
from scipy.spatial.transform import Rotation

from ins.data_loader import load_data
from ins.inertial_navigation import integrate_acceleration, integrate_velocity, world_acceleration
from ins.orientation import gyro_rotations, truth_rotations
from ins.zupt import correct_velocity, detect_stationary


def sample_line(step):
    blocks = []
    for node in range(6):
        prefix = f"{step} {node}" if node == 0 else str(node)
        blocks.append(prefix + " 1 0 0 0 0 0 9.81 0 0 0")
    return ";".join(blocks + ["1 1 0", "V " + " ".join(["0"] * 21)])


class NavigationTests(unittest.TestCase):
    def test_gravity_rotation_and_wxyz(self):
        rotations = Rotation.from_euler("xyz", [[.4, -.8, 1.2], [0, 0, 0]]).as_matrix()[:, None]
        force = np.einsum("...ji,j->...i", rotations, [0., 0., 9.81])
        assert_allclose(world_acceleration(rotations, force), 0, atol=1e-12)
        assert_allclose(world_acceleration(rotations, np.zeros_like(force)),
                        np.broadcast_to([0, 0, -9.81], force.shape))
        quaternion = np.array([[[np.sqrt(.5), 0, 0, np.sqrt(.5)]]])
        assert_allclose(truth_rotations(quaternion)[0, 0] @ [1, 0, 0], [0, 1, 0], atol=1e-12)

    def test_trapezoid_initial_state_and_constant_acceleration(self):
        time = np.arange(1, 101) * .001
        acceleration = np.broadcast_to([2., -3., .5], (100, 6, 3)).copy()
        v0 = np.array([.1, .2, -.3])
        velocity = integrate_acceleration(acceleration, time, v0)
        position = integrate_velocity(velocity, time)
        elapsed = (time - time[0])[:, None, None]
        assert_allclose(velocity, np.broadcast_to(v0, velocity.shape) + acceleration * elapsed, atol=1e-13)
        assert_allclose(position, v0 * elapsed + acceleration * elapsed**2 / 2, atol=1e-13)
        assert_array_equal(position[0], np.zeros((6, 3)))

    def test_step_end_reproduces_simulator_discrete_updates(self):
        time = np.arange(1, 101) * .001
        acceleration = np.zeros((100, 6, 3))
        acceleration[:, :, 0] = np.sin(time[:, None] * 100)
        initial = np.broadcast_to([.3, 0, 0], (6, 3))
        expected_velocity = np.empty_like(acceleration)
        expected_velocity[0] = initial
        expected_position = np.zeros_like(acceleration)
        for k in range(1, len(time)):
            dt = time[k] - time[k - 1]
            expected_velocity[k] = expected_velocity[k - 1] + acceleration[k] * dt
            expected_position[k] = expected_position[k - 1] + expected_velocity[k] * dt
        velocity = integrate_acceleration(acceleration, time, initial, "step_end")
        assert_allclose(velocity, expected_velocity, atol=1e-14)
        assert_allclose(integrate_velocity(velocity, time, "step_end"), expected_position, atol=1e-14)

    def test_first_step_acceleration_advances_simulation_start_velocity(self):
        time = np.arange(1, 5) * .001
        acceleration = np.zeros((4, 6, 3))
        acceleration[0, :, 2] = 300
        for method in ("step_end", "trapezoid"):
            velocity = integrate_acceleration(acceleration, time, [0, 0, 0], method, "simulation_start")
            assert_allclose(velocity[0, :, 2], .3)
        velocity = integrate_acceleration(acceleration, time, [0, 0, 0], "step_end", "simulation_start")
        assert_allclose(velocity[:, :, 2], .3)
        position = integrate_velocity(velocity, time, "step_end")
        assert_allclose(position[-1, :, 2], .0009)

    def test_body_gyro_includes_first_interval(self):
        time = np.arange(1, 101) * .001
        gyro = np.zeros((100, 6, 3))
        gyro[:, :, 2] = 2
        expected = Rotation.from_rotvec(np.c_[np.zeros((100, 2)), 2 * time]).as_matrix()
        for method in ("trapezoid", "step_end"):
            rotations = gyro_rotations(gyro, time, method)
            for node in range(6):
                assert_allclose(rotations[:, node], expected, atol=1e-13)


class StationaryTests(unittest.TestCase):
    config = {"minimum_duration_s": .03, "gyro_threshold_rad_s": .05,
              "force_norm_tolerance_m_s2": .3, "drive_before_s": .02,
              "drive_after_s": .1, "detector": "imu"}

    def test_drives_alone_cannot_create_stationary_windows(self):
        time = np.arange(1, 101) * .001
        controls = np.ones((100, 3), dtype=int)
        controls[50:, 0] = 0  # Other two drives remain ON.
        force = np.broadcast_to([0., 0., 100.], (100, 6, 3))
        gyro = np.zeros_like(force)
        detection = detect_stationary(force, gyro, time, controls, self.config)
        self.assertEqual(detection["events"].tolist(), [50])
        self.assertFalse(detection["stationary"].any())
        self.assertFalse(detection["drive_stationary"].any())

    def test_minimum_continuous_duration_and_drive_subset(self):
        time = np.arange(1, 101) * .001
        force = np.broadcast_to([0., 0., 9.81], (100, 6, 3)).copy()
        gyro = np.zeros_like(force)
        gyro[20:80, :, 0] = 1  # Both short edge runs must be rejected.
        controls = np.zeros((100, 3), dtype=int)
        detection = detect_stationary(force, gyro, time, controls, self.config)
        self.assertFalse(detection["stationary"].any())
        gyro[:] = 0
        detection = detect_stationary(force, gyro, time, controls, self.config)
        self.assertTrue(detection["stationary"].all())
        self.assertFalse(detection["drive_stationary"].any())

    def test_no_anchor_is_exact_identity(self):
        rng = np.random.default_rng(2)
        velocity = rng.normal(size=(100, 6, 3))
        result, counts = correct_velocity(velocity, np.arange(100) * .001, np.zeros((100, 6), dtype=bool))
        assert_array_equal(result, velocity)
        self.assertEqual(counts, [0] * 6)
        self.assertFalse(np.shares_memory(result, velocity))

    def test_ramp_only_with_two_anchors_and_tail_is_not_forced(self):
        time = np.arange(100) * .001
        velocity = np.broadcast_to(time[:, None, None], (100, 6, 3)).copy()
        stationary = np.zeros((100, 6), dtype=bool)
        stationary[10:20] = True
        stationary[70:80] = True
        result, counts = correct_velocity(velocity, time, stationary)
        assert_array_equal(result[:10], velocity[:10])  # Unbounded head: no fabricated start anchor.
        assert_allclose(result[10:80], 0, atol=1e-15)
        assert_allclose(result[80:], velocity[80:] - velocity[79], atol=1e-15)
        self.assertEqual(counts, [1] * 6)


class ParserTests(unittest.TestCase):
    def load_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.txt"
            path.write_text(text, encoding="utf-8")
            return load_data(path)

    def test_fields_and_overlapping_controls(self):
        data = self.load_text("\n".join(sample_line(k) for k in range(1, 4)))
        assert_allclose(data.time, [.001, .002, .003])
        assert_array_equal(data.controls, [[1, 1, 0]] * 3)
        self.assertEqual(data.specific_force.shape, (3, 6, 3))
        assert_allclose(data.specific_force[:, :, 2], 9.81)

    def test_rejects_malformed_nonfinite_missing_steps(self):
        valid = [sample_line(k) for k in range(1, 4)]
        variants = ["\n".join(valid).replace("9.81", "nan", 1),
                    "\n".join(valid).replace("9.81", "inf", 1),
                    "\n".join(valid).replace("1 1 0;V", "1 2 0;V", 1),
                    "\n".join([sample_line(1), sample_line(3), sample_line(4)]),
                    "\n".join(valid).replace(";1 ", ";2 ", 1),
                    "\n".join(valid).replace(";V ", "; ", 1)]
        for text in variants:
            with self.subTest(text=text[:50]):
                with self.assertRaises(ValueError):
                    self.load_text(text)


if __name__ == "__main__":
    unittest.main()
