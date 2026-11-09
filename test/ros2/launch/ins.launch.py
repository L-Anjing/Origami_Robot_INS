"""Replay or live IMU odometry with optional RViz2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("origami_ins"))
    arg = LaunchConfiguration
    boolean = lambda name: ParameterValue(arg(name), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("replay", default_value="true"),
        DeclareLaunchArgument("data_file", default_value=str(share/"data"/"imu_100hz.txt")),
        DeclareLaunchArgument("params_file", default_value=str(share/"config"/"ins.yaml")),
        DeclareLaunchArgument("config_file", default_value=str(share/"config"/"config.json")),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("start_delay_s", default_value="2.0"),
        DeclareLaunchArgument("zupt_mode", default_value="glrt"),
        DeclareLaunchArgument("use_sim_time", default_value=arg("replay")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="origami_ins", executable="inertial_odometry", name="inertial_odometry",
             output="screen", parameters=[arg("params_file"), {
                 "config_file": arg("config_file"), "imu_topic": arg("imu_topic"),
                 "imu_frame": arg("imu_frame"), "body_frame": arg("imu_frame"),
                 "zupt_mode": arg("zupt_mode"), "use_sim_time": boolean("use_sim_time")}]),
        Node(package="origami_ins", executable="imu_replay", name="imu_replay", output="screen",
             condition=IfCondition(arg("replay")), parameters=[{
                 "data_file": arg("data_file"), "imu_topic": arg("imu_topic"), "imu_frame": arg("imu_frame"),
                 "playback_rate": ParameterValue(arg("playback_rate"), value_type=float),
                 "start_delay_s": ParameterValue(arg("start_delay_s"), value_type=float),
                 "publish_clock": boolean("use_sim_time")}]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             condition=IfCondition(arg("use_rviz")),
             arguments=["-d", str(share/"rviz"/"ins.rviz")],
             parameters=[{"use_sim_time": boolean("use_sim_time")}]),
    ])
