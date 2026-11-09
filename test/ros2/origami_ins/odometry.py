"""Online ESKF adapter: Odometry, Path, TF, pose arrows and diagnostics."""
from collections import deque
import json
import math
import signal
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from ins.attitude_estimator import quaternion_matrix, skew
from ins.health import INSHealthMonitor
from ins.real_imu import IMUSample, RealIMUNavigator, TimeWindowDetector
from .replay import stamp_message
from .trajectory import split_trajectory, stamp_ns as stamp_to_ns


def fill_pose(pose, state):
    pose.position.x, pose.position.y, pose.position.z = map(float, state["position"])
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, state["quaternion"])


class InertialOdometry(Node):
    def __init__(self):
        super().__init__("inertial_odometry")
        defaults = {"config_file": "", "imu_topic": "/imu/data", "imu_frame": "imu_link",
                    "odom_frame": "odom", "body_frame": "imu_link", "raw_body_frame": "imu_raw",
                    "initial_static_seconds": 2., "stationary_window_seconds": .05,
                    "minimum_stationary_seconds": .05, "gyro_threshold_rad_s": .08,
                    "force_tolerance_m_s2": .30, "zupt_mode": "glrt", "path_rate_hz": 20.,
                    "visualization_rate_hz": 10., "max_path_points": 20000}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        for name, default in INSHealthMonitor.DEFAULTS.items():
            self.declare_parameter(f"health.{name}", default,
                                   ParameterDescriptor(read_only=True))
        self.declare_parameter("max_integration_interval_seconds", .25, ParameterDescriptor(read_only=True))
        self.declare_parameter("input_timeout_seconds", .50, ParameterDescriptor(read_only=True))
        self.declare_parameter("protect_zupt_position", True, ParameterDescriptor(read_only=True))
        p = lambda key: self.get_parameter(key).value
        with open(p("config_file"), encoding="utf-8") as stream:
            self.config = json.load(stream)
        self.config.setdefault("real_imu", {})["health"] = {
            name: float(p(f"health.{name}")) for name in INSHealthMonitor.DEFAULTS}
        self.config["real_imu"]["max_integration_interval_s"] = float(p("max_integration_interval_seconds"))
        self.config["real_imu"]["protect_position_updates"] = bool(p("protect_zupt_position"))
        self.input_timeout_s = float(p("input_timeout_seconds"))
        if not math.isfinite(self.input_timeout_s) or self.input_timeout_s <= 0:
            raise ValueError("input_timeout_seconds must be positive")
        detector = self.config["robot"]["stationary"]
        for config_name, parameter in [("window_s", "stationary_window_seconds"),
                                       ("minimum_stationary_s", "minimum_stationary_seconds"),
                                       ("gyro_limit_rad_s", "gyro_threshold_rad_s"),
                                       ("force_tolerance_m_s2", "force_tolerance_m_s2")]:
            value = float(p(parameter))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{parameter} must be positive")
            detector[config_name] = value
        self.initial_seconds, self.zupt_mode = p("initial_static_seconds"), p("zupt_mode")
        self.imu_frame, self.odom_frame = p("imu_frame"), p("odom_frame")
        self.body_frame, self.raw_frame = p("body_frame"), p("raw_body_frame")
        frames = [self.odom_frame, self.body_frame, self.raw_frame]
        if any(not frame or frame.startswith("/") for frame in [*frames, self.imu_frame]):
            raise ValueError("Frame IDs must be nonempty without a leading slash")
        if len(set(frames)) != 3 or self.body_frame != self.imu_frame:
            raise ValueError("odom/body/raw frames must differ; body must coincide with the input IMU")
        path_rate, visual_rate = float(p("path_rate_hz")), float(p("visualization_rate_hz"))
        if not all(math.isfinite(v) and v > 0 for v in (path_rate, visual_rate)):
            raise ValueError("Path and visualization rates must be positive")
        self.path_interval_ns = max(1, round(1e9/path_rate))
        self.max_points = int(p("max_path_points"))
        if self.max_points < 2:
            raise ValueError("max_path_points must be at least 2")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, "/Odometry", 100)
        self.raw_odom_pub = self.create_publisher(Odometry, "/ins/raw_odometry", 100)
        self.path_pub = self.create_publisher(Path, "/path", latched)
        self.full_path_pub = self.create_publisher(Path, "/ins/estimate_history", latched)
        self.segments_pub = self.create_publisher(MarkerArray, "/ins/trajectory_segments", latched)
        self.raw_path_pub = self.create_publisher(Path, "/ins/raw_path", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/ins/markers", latched)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.tf = TransformBroadcaster(self)
        self.reset_state()
        self.add_on_set_parameters_callback(self.on_timing_parameters)
        # Best effort input accepts live SensorData publishers and reliable replay.
        qos = QoSProfile(depth=2000, reliability=qos_profile_sensor_data.reliability)
        self.create_subscription(Imu, p("imu_topic"), self.on_imu, qos)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.create_timer(1/visual_rate, self.publish_visuals,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Listening on {p('imu_topic')}; keep IMU static for {self.initial_seconds:g} s. "
            f"TF {self.odom_frame} -> {self.body_frame}; local Z up, initial yaw=0. "
            f"ZUPT window={detector['window_s']:g} s, minimum rest={detector['minimum_stationary_s']:g} s.")

    def on_timing_parameters(self, parameters):
        names = {"stationary_window_seconds": "window_s",
                 "minimum_stationary_seconds": "minimum_stationary_s"}
        updates = {}
        for parameter in parameters:
            if parameter.name in names:
                value = parameter.value
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    return SetParametersResult(successful=False, reason=f"{parameter.name} must be positive and finite")
                updates[names[parameter.name]] = float(value)
        if not updates:
            return SetParametersResult(successful=True)
        config = dict(self.navigator.config["robot"]["stationary"])
        config.update(updates)
        intervals = list(self.navigator.intervals)
        if not intervals and len(self.navigator.pending) >= 2:
            pending = self.navigator.pending
            intervals = [b.timestamp_ns-a.timestamp_ns for a, b in zip(pending, pending[1:])]
        if intervals and config["window_s"]*1e9 < 2*float(np.median(intervals)):
            return SetParametersResult(successful=False, reason="Stationary window must cover at least two sample intervals")
        self.config["robot"]["stationary"].update(updates)
        self.navigator.config["robot"]["stationary"].update(updates)
        if self.navigator.eskf is not None:
            # Restart rest qualification under the new timing; preserve all INS states.
            self.navigator.detector = TimeWindowDetector(config, self.config["gravity_m_s2"])
        self.get_logger().info(f"ZUPT timing updated: window={config['window_s']:g} s, "
                               f"minimum rest={config['minimum_stationary_s']:g} s")
        return SetParametersResult(successful=True)

    def reset_state(self):
        self.navigator = RealIMUNavigator(self.config, self.initial_seconds, self.zupt_mode)
        self.path, self.raw_path = deque(maxlen=self.max_points), deque(maxlen=self.max_points)
        self.zupt_points = deque(maxlen=self.max_points)
        self.last_path_ns = None
        self.previous_result = None
        self.path_breaks = deque()
        self.anomaly_points = deque(maxlen=self.max_points)
        self.latest = None
        self.failed = ""
        self.received = self.dropped = 0
        self.last_input_wall = self.last_input_clock_ns = None

    def input_age_s(self):
        if self.last_input_wall is None:
            return None
        if self.get_parameter("use_sim_time").value:
            return max(0., (self.get_clock().now().nanoseconds-self.last_input_clock_ns)*1e-9)
        return max(0., time.monotonic()-self.last_input_wall)

    def on_reset(self, request, response):
        self.reset_state()
        clear = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[clear]))
        self.publish_visuals()
        response.success = True
        response.message = "Reset; keep IMU static for calibration again."
        return response

    def on_imu(self, msg):
        if self.failed:
            return
        timestamp_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if (timestamp_ns < 0 or msg.header.frame_id != self.imu_frame or
                (self.navigator.previous_input_ns is not None and
                 timestamp_ns <= self.navigator.previous_input_ns)):
            self.dropped += 1
            self.get_logger().warning("Ignoring IMU with unexpected frame or non-increasing timestamp",
                                      throttle_duration_sec=2.)
            return
        sample = IMUSample(timestamp_ns,
                           np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
                           np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
        if not np.isfinite([sample.force, sample.gyro]).all():
            self.dropped += 1
            self.get_logger().warning("Ignoring nonfinite IMU sample", throttle_duration_sec=2.)
            return
        self.received += 1
        self.last_input_wall = time.monotonic()
        self.last_input_clock_ns = self.get_clock().now().nanoseconds
        was_ready = self.navigator.eskf is not None
        try:
            results = self.navigator.push(sample)
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            self.failed = str(error)
            self.get_logger().error(f"INS stopped: {error}. Correct the input and call ~/reset.")
            return
        if not results:
            return
        if not was_ready:
            cal = self.navigator.initialization
            self.get_logger().info(f"Alignment ready: {cal['samples']} samples; gyro bias={cal['gyro_bias']}, "
                                   f"accel bias={cal['accel_bias']}")
        for result in results:
            stamp_ns = result["timestamp_ns"]
            health = result["health"]
            broken = health["position_discontinuity"]
            if broken:
                if self.previous_result is not None:
                    for name, history in (("eskf", self.path), ("raw", self.raw_path)):
                        before = self.previous_result
                        if not history or history[-1].header.stamp != stamp_message(before["timestamp_ns"]):
                            pose = PoseStamped()
                            pose.header.stamp = stamp_message(before["timestamp_ns"])
                            pose.header.frame_id = self.odom_frame
                            fill_pose(pose.pose, before[name])
                            history.append(pose)
                    x, y, z = map(float, self.previous_result["eskf"]["position"])
                    self.anomaly_points.append(Point(x=x, y=y, z=z))
                self.path_breaks.append(stamp_ns)
                x, y, z = map(float, result["eskf"]["position"])
                self.anomaly_points.append(Point(x=x, y=y, z=z))
            if self.last_path_ns is None or stamp_ns-self.last_path_ns >= self.path_interval_ns or broken:
                for name, history in (("eskf", self.path), ("raw", self.raw_path)):
                    pose = PoseStamped()
                    pose.header.stamp, pose.header.frame_id = stamp_message(stamp_ns), self.odom_frame
                    fill_pose(pose.pose, result[name])
                    history.append(pose)
                self.last_path_ns = stamp_ns
            if health["anomaly"]:
                elapsed = (stamp_ns-self.navigator.first_ns)*1e-9
                self.get_logger().error(
                    f"INS continuity anomaly at {elapsed:.3f} s: step={health['position_step_m']:.3f} m, "
                    f"propagation={health['propagated_step_m']:.3f} m, "
                    f"correction={health['position_correction_m']:.3f} m, "
                    f"limit={health['position_step_limit_m']:.3f} m; ZUPT NIS={result['nis']:.3g}. "
                    "Estimate retained; trajectory continuity is lost.", throttle_duration_sec=1.)
            elif health["position_update_rejected"]:
                elapsed = (stamp_ns-self.navigator.first_ns)*1e-9
                self.get_logger().error(
                    f"ZUPT at {elapsed:.3f} s: inferred position correction "
                    f"{health['proposed_position_correction_m']:.3f} m rejected; "
                    "zero-velocity, attitude and bias update retained. Position remains uncertain.",
                    throttle_duration_sec=1.)
            if result["accepted"]:
                x, y, z = map(float, result["eskf"]["position"])
                self.zupt_points.append(Point(x=x, y=y, z=z))
            self.previous_result = result
        if self.path:
            oldest_ns = stamp_to_ns(self.path[0].header.stamp)
            while self.path_breaks and self.path_breaks[0] < oldest_ns:
                self.path_breaks.popleft()
        self.latest = results[-1]
        if self.latest["gap"]:
            self.get_logger().warning("IMU time gap detected; integrating actual dt and restarting rest duration",
                                      throttle_duration_sec=2.)
        self.publish_odometry(self.latest, sample.gyro)

    def publish_odometry(self, result, measured_gyro):
        transforms = []
        for name, publisher, child in (("eskf", self.odom_pub, self.body_frame),
                                        ("raw", self.raw_odom_pub, self.raw_frame)):
            state = result[name]
            rotation = quaternion_matrix(state["quaternion"])
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = stamp_message(result["timestamp_ns"]), self.odom_frame
            msg.child_frame_id = child
            fill_pose(msg.pose.pose, state)
            # Odometry twist is in child_frame_id, while p and v internally are in odom.
            body_velocity = rotation.T @ state["velocity"]
            body_rate = measured_gyro-state["bg"]
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, body_velocity)
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, body_rate)
            if name == "eskf":
                covariance = result["covariance"]
                jac_pose = np.zeros((6, 15))
                jac_pose[:3, :3], jac_pose[3:, 6:9] = np.eye(3), rotation
                jac_twist = np.zeros((6, 15))
                jac_twist[:3, 3:6], jac_twist[:3, 6:9] = rotation.T, skew(body_velocity)
                jac_twist[3:, 9:12] = -np.eye(3)
                pose_cov = jac_pose @ covariance @ jac_pose.T
                twist_cov = jac_twist @ covariance @ jac_twist.T
                dt = float(np.median(self.navigator.intervals))*1e-9
                gyro_density = self.config["robot"]["eskf"]["noise_density"]["gyro_rad_s_sqrt_hz"]
                twist_cov[3:, 3:] += np.eye(3)*gyro_density**2/dt
                msg.pose.covariance = pose_cov.ravel().tolist()
                msg.twist.covariance = twist_cov.ravel().tolist()
            else:
                msg.pose.covariance = (np.eye(6)*1e6).ravel().tolist()
                msg.twist.covariance = (np.eye(6)*1e6).ravel().tolist()
            publisher.publish(msg)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = msg.header, child
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation = msg.pose.pose.orientation
            transforms.append(tf)
        self.tf.sendTransform(transforms)

    def publish_visuals(self):
        timestamp_ns = self.latest["timestamp_ns"] if self.latest else 0
        for publisher, history, state_name in ((self.path_pub, self.path, "eskf"),
                                               (self.raw_path_pub, self.raw_path, "raw")):
            path = Path()
            path.header.frame_id, path.header.stamp = self.odom_frame, stamp_message(timestamp_ns)
            path.poses = list(history)
            # Include the newest sample even when the rate-limited history ends earlier.
            if self.latest and (not history or history[-1].header.stamp != path.header.stamp):
                pose = PoseStamped(header=path.header)
                fill_pose(pose.pose, self.latest[state_name])
                path.poses.append(pose)
            if state_name == "eskf":
                self.full_path_pub.publish(path)
                segments = split_trajectory(path.poses, self.path_breaks)
                previous = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
                for index, segment in enumerate(segments[:-1]):
                    if len(segment) < 2:
                        continue
                    marker = Marker()
                    marker.header = path.header
                    marker.ns, marker.id = "trajectory_segments", index
                    marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
                    marker.pose.orientation.w = 1.
                    marker.scale.x = .006
                    marker.color.g, marker.color.b, marker.color.a = .745, 1., 1.
                    marker.points = [pose.pose.position for pose in segment]
                    previous.markers.append(marker)
                self.segments_pub.publish(previous)
                path = Path(header=path.header, poses=segments[-1] if segments else [])
            publisher.publish(path)
        if self.latest:
            self.marker_pub.publish(self.markers())
        self.publish_diagnostics()

    def markers(self):
        stamp = stamp_message(self.latest["timestamp_ns"])
        # Remove the status label left by earlier versions in an open RViz session.
        markers = [Marker(ns="ins", id=3, action=Marker.DELETE)]
        for marker_id, kind in ((0, Marker.ARROW), (1, Marker.SPHERE),
                                 (2, Marker.SPHERE_LIST), (4, Marker.SPHERE_LIST)):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.odom_frame, stamp
            marker.ns, marker.id, marker.type, marker.action = "ins", marker_id, kind, Marker.ADD
            marker.pose.orientation.w = 1.
            marker.color.a = 1.
            if marker_id == 0:
                fill_pose(marker.pose, self.latest["eskf"])
                marker.scale.x, marker.scale.y, marker.scale.z = .12, .025, .025
                marker.color.r, marker.color.g, marker.color.b = .1, .65, 1.
                if self.latest["health"]["estimate_unreliable"]:
                    marker.color.r, marker.color.g, marker.color.b = 1., .1, .1
            elif marker_id == 1:
                marker.scale.x = marker.scale.y = marker.scale.z = .025
                marker.color.g = 1.
            elif marker_id == 2:
                marker.scale.x = marker.scale.y = marker.scale.z = .012
                marker.color.r, marker.color.g = .1, 1.
                marker.points = list(self.zupt_points)
            elif marker_id == 4:
                marker.scale.x = marker.scale.y = marker.scale.z = .03
                marker.color.r, marker.color.g, marker.color.b = 1., .1, .1
                marker.points = list(self.anomaly_points)
            markers.append(marker)
        return MarkerArray(markers=markers)

    def publish_diagnostics(self):
        status = DiagnosticStatus(name="origami_ins/estimator", hardware_id=self.imu_frame)
        status.level = DiagnosticStatus.OK
        if self.failed:
            status.level, status.message = DiagnosticStatus.ERROR, self.failed
        elif self.input_age_s() is not None and self.input_age_s() > self.input_timeout_s:
            status.level = DiagnosticStatus.STALE
            status.message = f"IMU input stopped: last valid receipt {self.input_age_s():.2f} s ago"
        elif not self.latest:
            status.level = DiagnosticStatus.WARN
            status.message = f"Static calibration: {len(self.navigator.pending)} samples; keep IMU still"
        else:
            status.message = "Running: local Z up, yaw unreferenced"
            health = self.latest["health"]
            if health["continuity_lost"]:
                status.level = DiagnosticStatus.ERROR
                status.message = "INS continuity lost: large position step/correction; estimate retained for inspection"
            elif health["position_update_rejections"]:
                status.level = DiagnosticStatus.ERROR
                status.message = "Large inferred position correction rejected; position estimate remains uncertain"
            elif health["position_uncertain"]:
                status.level = DiagnosticStatus.WARN
                status.message = "Position uncertainty exceeds the configured application limit"
            elif not self.latest["valid"]:
                status.level, status.message = DiagnosticStatus.WARN, "Impact/range flag: aiding disabled"
        values = {"received": self.received, "processed": self.navigator.samples,
                  "dropped": self.dropped, "zupt_accepted": self.navigator.accepted,
                  "zupt_rejected_nis": self.navigator.rejected, "time_gaps": self.navigator.gaps,
                  "stationary_window_seconds": self.navigator.config["robot"]["stationary"]["window_s"],
                  "minimum_stationary_seconds": self.navigator.config["robot"]["stationary"]["minimum_stationary_s"]}
        values["input_age_seconds"] = self.input_age_s()
        if self.latest:
            values.update({"stationary": self.latest["stationary"], "glrt": self.latest["glrt"],
                           "zupt_nis": self.latest["nis"], "speed_m_s": np.linalg.norm(self.latest["eskf"]["velocity"]),
                           "gyro_bias_rad_s": self.latest["eskf"]["bg"].tolist(),
                           "accel_bias_m_s2": self.latest["eskf"]["ba"].tolist()})
            values.update(self.latest["health"])
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = stamp_message(self.latest["timestamp_ns"]) if self.latest else self.get_clock().now().to_msg()
        self.diagnostic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InertialOdometry()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
