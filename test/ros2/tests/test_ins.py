"""Numerical checks for alignment, kinematics and causal rest detection."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from ins.core import INS
from ins.health import INSHealthMonitor
from ins.real_imu import IMUSample, RealIMUNavigator, TimeWindowDetector, read_recording


ROOT = Path(__file__).resolve().parents[3]


def config():
    with (ROOT/"config.json").open() as stream:
        c = json.load(stream)
    c["robot"]["stationary"].update(window_s=.15, minimum_stationary_s=.20, gyro_limit_rad_s=.08)
    return c


class INSTests(unittest.TestCase):
    def test_guarded_zupt_retains_velocity_update_and_position_uncertainty(self):
        from copy import deepcopy
        c = config()
        source = INS(np.zeros(3), [1., 0., 0.], c["robot"]["ahrs"], c["robot"]["eskf"])
        source.P = np.eye(15)
        source.P[0, 3] = source.P[3, 0] = .9
        guarded, normal = deepcopy(source), deepcopy(source)
        result = guarded.zupt_update(True, max_position_correction_m=.02)
        normal.zupt_update(True)
        self.assertTrue(result["accepted"])
        self.assertTrue(result["position_update_rejected"])
        self.assertGreater(result["proposed_position_correction_m"], .8)
        np.testing.assert_array_equal(guarded.position, source.position)
        np.testing.assert_allclose(guarded.velocity, normal.velocity)
        np.testing.assert_allclose(guarded.P[:3, :3], source.P[:3, :3])
        self.assertGreaterEqual(np.linalg.eigvalsh(guarded.P)[0], -1e-12)
        self.assertLess(np.linalg.norm(guarded.velocity), .001)
        source.velocity *= .001
        guarded, normal = deepcopy(source), deepcopy(source)
        result = guarded.zupt_update(True, max_position_correction_m=.02)
        normal.zupt_update(True)
        self.assertFalse(result["position_update_rejected"])
        np.testing.assert_array_equal(guarded.position, normal.position)
        np.testing.assert_array_equal(guarded.P, normal.P)

    def test_navigator_guard_records_rejected_proposal_without_position_jump(self):
        c = config()
        c["real_imu"] = {"protect_position_updates": True}
        nav = RealIMUNavigator(c)
        force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
        for i in range(221):
            nav.push(IMUSample(i*10_000_000, force, gyro))
        nav.eskf.P = np.eye(15)
        nav.eskf.P[0, 3] = nav.eskf.P[3, 0] = .9
        nav.eskf.velocity[:] = [1., 0., 0.]
        result = nav.push(IMUSample(2_220_000_000, force, gyro))[0]
        self.assertTrue(result["accepted"])
        self.assertTrue(result["health"]["position_update_rejected"])
        self.assertFalse(result["health"]["position_discontinuity"])
        self.assertTrue(result["health"]["estimate_unreliable"])
        self.assertAlmostEqual(result["health"]["position_correction_m"], 0.)
        self.assertAlmostEqual(result["eskf"]["position"][0], .02)
        self.assertGreaterEqual(nav.check_covariance(), -1e-12)

    def test_short_gap_integrates_actual_dt_without_clipping(self):
        c = config()
        c["robot"]["ahrs"]["gain_s_inv"] = 0.
        nav = RealIMUNavigator(c, zupt_mode="none")
        force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
        for i in range(221):
            nav.push(IMUSample(i*10_000_000, force, gyro))
        result = nav.push(IMUSample(2_350_000_000, np.array([1., 0., 9.81]), gyro))[0]
        self.assertTrue(result["gap"])
        self.assertAlmostEqual(nav.eskf.position[0], .5*.15**2)
        self.assertAlmostEqual(nav.eskf.velocity[0], .15)

    def test_long_input_gap_does_not_modify_filter_or_calibration(self):
        force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
        nav = RealIMUNavigator(config())
        for i in range(221):
            nav.push(IMUSample(i*10_000_000, force, gyro))
        state, covariance = nav.eskf.snapshot(), nav.eskf.P.copy()
        stamp, samples = nav.previous_input_ns, nav.samples
        with self.assertRaisesRegex(ValueError, "integration suspended"):
            nav.push(IMUSample(200_000_000_000, force, gyro))
        for key, value in state.items():
            np.testing.assert_array_equal(nav.eskf.snapshot()[key], value)
        np.testing.assert_array_equal(nav.eskf.P, covariance)
        self.assertEqual(nav.previous_input_ns, stamp)
        self.assertEqual(nav.samples, samples)
        nav = RealIMUNavigator(config())
        nav.push(IMUSample(0, force, gyro))
        with self.assertRaisesRegex(ValueError, "integration suspended"):
            nav.push(IMUSample(3_000_000_000, force, gyro))
        self.assertEqual(len(nav.pending), 1)
        self.assertIsNone(nav.eskf)

    def test_health_distinguishes_motion_from_position_correction(self):
        monitor = INSHealthMonitor()
        zero = np.zeros(3)
        covariance = np.eye(15)*.01
        health = monitor.observe(10_000_000, zero, np.array([.005, 0., 0.]),
                                 np.array([.005, 0., 0.]), covariance, .01)
        self.assertFalse(health["anomaly"])
        health = monitor.observe(20_000_000, zero, np.array([.005, 0., 0.]),
                                 np.array([2., 0., 0.]), covariance, .01)
        self.assertTrue(health["large_position_correction"])
        self.assertTrue(health["position_discontinuity"])
        self.assertFalse(health["excessive_propagation"])
        self.assertAlmostEqual(health["position_correction_m"], 1.995)
        health = monitor.observe(30_000_000, zero, zero, zero, covariance, .01)
        self.assertFalse(health["anomaly"])
        self.assertTrue(health["continuity_lost"])
        self.assertEqual(health["anomaly_count"], 1)

    def test_health_uses_elapsed_time_and_reports_uncertainty(self):
        zero = np.zeros(3)
        monitor = INSHealthMonitor()
        covariance = np.eye(15)
        health = monitor.observe(1_000_000_000, zero, np.array([1., 0., 0.]),
                                 np.array([1., 0., 0.]), covariance, 1.)
        self.assertFalse(health["anomaly"])
        self.assertTrue(health["position_uncertain"])
        health = monitor.observe(1_010_000_000, zero, np.array([1., 0., 0.]),
                                 np.array([1., 0., 0.]), covariance, .01)
        self.assertTrue(health["excessive_propagation"])

    def test_large_zupt_correction_passes_nis_but_triggers_continuity_fault(self):
        nav = RealIMUNavigator(config())
        force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
        for i in range(221):
            nav.push(IMUSample(i*10_000_000, force, gyro))
        nav.eskf.P = np.eye(15)
        nav.eskf.P[0, 3] = nav.eskf.P[3, 0] = .9
        nav.eskf.velocity[:] = [1., 0., 0.]
        result = nav.push(IMUSample(2_220_000_000, force, gyro))[0]
        self.assertTrue(result["accepted"])
        self.assertLess(result["nis"], nav.config["robot"]["eskf"]["zupt_nis_limit"])
        self.assertTrue(result["health"]["large_position_correction"])
        self.assertTrue(result["health"]["continuity_lost"])
        self.assertGreater(abs(result["eskf"]["position"][0]), .5)
        np.testing.assert_array_equal(result["eskf"]["position"], nav.eskf.position)
        prior = result["zupt_prior"]
        expected_correction = prior["position_velocity_covariance"] @ np.linalg.solve(
            prior["velocity_innovation_covariance"], -prior["velocity"])
        np.testing.assert_allclose(result["eskf"]["position"]-prior["position"], expected_correction)

    def test_constant_acceleration_position_and_legacy_default(self):
        c = config()
        ahrs = deepcopy(c["robot"]["ahrs"])
        ahrs["gain_s_inv"] = 0.
        for method, expected_x in [("constant_acceleration", 1.), ("step_end", 1.01)]:
            ins = INS(np.zeros(3), np.zeros(3), ahrs, c["robot"]["eskf"],
                      backend="gyro", position_integration=method)
            for _ in range(100):
                ins.propagate(np.zeros(3), np.array([2., 0., 9.81]), .01)
            np.testing.assert_allclose(ins.position, [expected_x, 0, 0], atol=1e-12)
            np.testing.assert_allclose(ins.velocity, [2, 0, 0], atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(ins.P)[0], -1e-12)

    def test_known_yaw_rotation(self):
        c = config()
        ins = INS(np.zeros(3), np.zeros(3), c["robot"]["ahrs"], c["robot"]["eskf"],
                  backend="gyro", covariance=False, position_integration="constant_acceleration")
        for _ in range(100):
            ins.propagate(np.array([0., 0., np.pi/2]), np.array([0., 0., 9.81]), .01)
        np.testing.assert_allclose(ins.quaternion, [np.sqrt(.5), 0, 0, np.sqrt(.5)], atol=1e-12)
        np.testing.assert_allclose(ins.position, np.zeros(3), atol=1e-12)

    def test_static_tilt_and_bias_at_both_rates(self):
        origin = 1_793_621_978_123_456_789
        roll = .2
        up = np.array([0., np.sin(roll), np.cos(roll)])
        force, gyro = up*9.86, np.array([.001, -.002, .003])
        for hz in (100, 500):
            nav = RealIMUNavigator(config())
            for i in range(3*hz+1):
                nav.push(IMUSample(origin+i*(10**9//hz), force, gyro))
            np.testing.assert_allclose(nav.eskf.position, 0, atol=1e-10)
            np.testing.assert_allclose(nav.eskf.velocity, 0, atol=1e-10)
            np.testing.assert_allclose(nav.eskf.quaternion, [np.cos(roll/2), np.sin(roll/2), 0, 0], atol=1e-10)
            np.testing.assert_allclose(nav.eskf.bg, gyro, atol=1e-10)
            self.assertGreater(nav.accepted, 50)
            self.assertGreaterEqual(nav.check_covariance(), -1e-12)
            self.assertEqual(nav.samples, 3*hz+1)

    def test_causal_detector_restarts_after_gap(self):
        c = config()
        detector = TimeWindowDetector(c["robot"]["stationary"], 9.81)
        force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
        for i in range(31):
            stationary, _ = detector.update(i*10_000_000, force, gyro, True, 50_000_000)
            if i < 20:
                self.assertFalse(stationary)
        self.assertTrue(stationary)
        stationary, _ = detector.update(800_000_000, force, gyro, True, 50_000_000)
        self.assertFalse(stationary)
        for i in range(1, 21):
            stationary, _ = detector.update(800_000_000+i*10_000_000, force, gyro, True, 50_000_000)
        self.assertTrue(stationary)

    def test_motion_rejected_during_initialization(self):
        nav = RealIMUNavigator(config())
        with self.assertRaisesRegex(ValueError, "not static"):
            for i in range(201):
                nav.push(IMUSample(i*10_000_000, np.array([0., 0., 9.81]), np.array([0., 0., .5])))

    def test_50ms_rest_detection_at_both_rates(self):
        for hz in (100, 500):
            c = config()
            c["robot"]["stationary"].update(window_s=.05, minimum_stationary_s=.05)
            detector = TimeWindowDetector(c["robot"]["stationary"], 9.81)
            step_ns = 10**9//hz
            force, gyro = np.array([0., 0., 9.81]), np.zeros(3)
            for i in range(hz//20+1):
                stationary, _ = detector.update(i*step_ns, force, gyro, True, 5*step_ns)
                self.assertEqual(stationary, i*step_ns >= 50_000_000)
            # A new motion sample immediately cancels an already confirmed rest.
            stationary, _ = detector.update((i+1)*step_ns, force, np.array([.2, 0, 0]), True, 5*step_ns)
            self.assertFalse(stationary)

    def test_no_zupt_mode_and_timestamp_rejection(self):
        nav = RealIMUNavigator(config(), zupt_mode="none")
        for i in range(221):
            nav.push(IMUSample(i*10_000_000, np.array([0., 0., 9.81]), np.zeros(3)))
        self.assertEqual(nav.accepted, 0)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            nav.push(IMUSample(2_200_000_000, np.array([0., 0., 9.81]), np.zeros(3)))

    def test_parser_retains_nanoseconds_and_rejects_duplicate(self):
        origin = 1_793_621_978_123_456_789
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"imu.txt"
            lines = [f"{i+1} 0 1 0 0 0 0 0 9.81 0 0 0;T {origin+i*2_000_001}\n" for i in range(3)]
            path.write_text("# recorder\n"+"".join(lines), encoding="utf-8-sig")
            samples = read_recording(path)
            self.assertEqual(samples[1].timestamp_ns-samples[0].timestamp_ns, 2_000_001)
            path.write_text("".join(lines+[lines[-1]]))
            with self.assertRaisesRegex(ValueError, "must increase"):
                read_recording(path)


if __name__ == "__main__":
    unittest.main()
