"""Replay recorder TXT files without losing integer nanoseconds or IMU frames."""
import math
import signal
import time

import rclpy
from rclpy.clock import Clock as ROSClock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool

from ins.real_imu import read_recording


def stamp_message(timestamp_ns):
    from builtin_interfaces.msg import Time
    sec, nanosec = divmod(int(timestamp_ns), 10**9)
    return Time(sec=sec, nanosec=nanosec)


class IMUReplay(Node):
    def __init__(self):
        super().__init__("imu_replay")
        defaults = {"data_file": "", "imu_topic": "/imu/data", "imu_frame": "imu_link",
                    "playback_rate": 1., "start_delay_s": 2., "publish_clock": True}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = lambda key: self.get_parameter(key).value
        self.rate = float(p("playback_rate"))
        delay = float(p("start_delay_s"))
        if not math.isfinite(self.rate) or self.rate <= 0 or not math.isfinite(delay) or delay < 0:
            raise ValueError("playback_rate must be positive and start_delay_s nonnegative")
        self.samples = read_recording(p("data_file"))
        self.frame = p("imu_frame")
        # Reliable replay is also compatible with best-effort SensorData subscribers.
        self.publisher = self.create_publisher(Imu, p("imu_topic"), QoSProfile(depth=2000))
        self.clock_publisher = None
        if p("publish_clock"):
            self.clock_publisher = self.create_publisher(Clock, "/clock", QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.index = 0
        self.started = False
        self.paused = False
        self.finished = False
        self.due_wall = time.monotonic()+delay
        self.origin_ns = self.samples[0].timestamp_ns
        self.create_service(SetBool, "~/pause", self.pause)
        # Wall clock timer is essential: /clock has no publisher until replay starts.
        self.timer = self.create_timer(.001, self.tick, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
        duration = (self.samples[-1].timestamp_ns-self.origin_ns)*1e-9
        self.get_logger().info(f"Loaded {len(self.samples)} samples, {duration:.3f} s; rate={self.rate:g}")

    def pause(self, request, response):
        now = time.monotonic()
        if request.data and not self.paused:
            self.pause_wall = now
            self.paused = True
        elif not request.data and self.paused:
            self.due_wall += now-self.pause_wall
            self.paused = False
        response.success = True
        response.message = "paused" if self.paused else "playing"
        return response

    def tick(self):
        if self.paused:
            return
        if self.finished:
            # Keep the final simulation time available to late RViz subscribers.
            if self.clock_publisher:
                self.clock_publisher.publish(Clock(clock=stamp_message(self.samples[-1].timestamp_ns)))
            return
        now = time.monotonic()
        if now < self.due_wall:
            return
        if not self.started:
            if self.publisher.get_subscription_count() == 0:
                return
            self.due_wall = now
            self.started = True
        target_ns = self.origin_ns + int((now-self.due_wall)*self.rate*1e9)
        sent = 0
        while self.index < len(self.samples) and self.samples[self.index].timestamp_ns <= target_ns:
            sample = self.samples[self.index]
            stamp = stamp_message(sample.timestamp_ns)
            if self.clock_publisher:
                self.clock_publisher.publish(Clock(clock=stamp))
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = stamp, self.frame
            q = sample.device_quaternion
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, q)
            msg.orientation_covariance[0] = -1.  # device orientation is uncalibrated, unused by the estimator
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, sample.force)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, sample.gyro)
            self.publisher.publish(msg)
            self.index += 1
            sent += 1
            if sent >= 200:
                break  # bound each timer callback, without dropping overdue samples
        if self.index == len(self.samples):
            self.finished = True
            self.timer.timer_period_ns = 100_000_000
            self.get_logger().info(f"Replay complete: {self.index} samples. Final time retained; Ctrl+C to exit.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IMUReplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
