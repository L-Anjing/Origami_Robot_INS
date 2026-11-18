# ROS 2 接口

包 `origami_ins`，原生可执行文件 `inertial_odometry` 、`imu_replay` 与 `imu_record`，节点同名。统一根目录ament构建，无rclpy或ament_cmake_python依赖。

| 接口 | 类型 | 约定 |
|---|---|---|
| `/imu/data`（可配置） | sensor_msgs/Imu | 比力m/s²、角速度rad/s、严格递增时间戳，默认frame=imu_link |
| `/Odometry` | nav_msgs/Odometry | ESKF；位置在odom、twist在child frame |
| `/ins/raw_odometry` | nav_msgs/Odometry | 同传播的Raw INS，默认child=imu_raw |
| `/path` | nav_msgs/Path | 当前连续轨迹段 |
| `/ins/estimate_history` | nav_msgs/Path | 有界完整轨迹历史 |
| `/ins/raw_path` | nav_msgs/Path | Raw INS轨迹 |
| `/ins/trajectory_segments` | visualization_msgs/MarkerArray | 历史连续段 |
| `/ins/markers` | visualization_msgs/MarkerArray | 朝向箭头、原点、ZUPT点、异常点 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 初始化、输入超时、异常、偏置、NIS和健康状态 |
| `/tf` | TF | odom → imu_link / imu_raw |
| `/clock` | rosgraph_msgs/Clock | 回放可选发布 |
| `/inertial_odometry/reset` | std_srvs/Trigger | 清状态并重新静止初始化 |
| `/imu_replay/pause` | std_srvs/SetBool | true暂停，false继续 |

ROS帧必须非空且无前导斜杠；body_frame必须与imu_frame一致，未假定未测量的外参。IMU输入使用SensorDataQoS，回放使用可靠QoS。路径和Marker使用transient-local；保留输入频率计算、路径采样频率、显示频率的独立配置。

`config/ins.yaml` 包含原参数名。`config_file` 指向算法配置（默认安装的 `config/real_imu.json`）。`stationary_window_seconds`、`minimum_stationary_seconds` 支持在线修改并重置检测窗口，不重置导航状态；其余业务参数只读，修改后需重启。`use_sim_time` 由ROS自身管理。

```bash
ros2 param set /inertial_odometry stationary_window_seconds 0.06
ros2 service call /inertial_odometry/reset std_srvs/srv/Trigger '{}'
ros2 service call /imu_replay/pause std_srvs/srv/SetBool '{data: true}'
```

帧/时间戳/有限性错误丢弃输入；过大时间间隔停止积分直到reset。位置修正保护、轨迹断点和健康告警保留，不将异常强制平滑或隐藏。静止初始化只估计初值，不能完整辨识单姿态加计偏置。

本机接口测试（需允许localhost DDS socket）：

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173
build/native_ros/origami_ins/test_ros_interfaces build/native_ros/origami_ins \
  config/real_imu.json data/imu_100hz.txt
```

该测试为单独可执行文件，不自动加入默认CTest，避免无网络权限的构建环境误报。覆盖Odometry、历史Path、动态窗口、reset、回放IMU、时钟和pause服务。

## TXT 录制

`imu_record` 使用 SensorDataQoS 订阅 `imu_topic`（默认 `/imu/data`），将单 IMU 数据写入必填参数 `output_file` 指定的新文件。输出兼容离线 `real` 与 `imu_replay`，保存消息时间戳、设备四元数、比力与角速度。文件已存在时拒绝启动；Ctrl+C 停止。姿态不可用时以单位四元数占位，非有限、非单位四元数及非递增时间戳样本丢弃。用法见根目录 README。
