# ROS 2 INS 轨迹与朝向验证

在 RViz2 中直接观察 INS 估计的三维轨迹和当前朝向：TXT 回放与实时 IMU 共用 ESKF + GLRT/ZUPT 核心，通过轨迹、姿态坐标轴、方向箭头及零速更新位置呈现惯性状态的演化。

ROS 2 Humble 入口与 `test/test_real_imu.m` 共用初始静止对准、六轴姿态估计和滤波参数约定；按真实时间戳积分，默认前 2 秒静止初始化、初始位置/速度/航向为零。100 Hz 和 500 Hz 数据共用按秒定义的静止窗口。

参考 [FAST_LIO](https://github.com/zxzzzxz-id/fast_lio) 的 launch、参数文件、Odometry/TF 和 RViz2 组织方式，围绕 INS 轨迹与朝向提供完整的回放和观察入口。

## 编译与运行

在本仓库根目录执行，使用系统 Python（需要 NumPy 和 ROS 2 Humble 的 `ament_cmake_python`、`rclpy`、`tf2_ros`、RViz2）：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths test/ros2 --packages-select origami_ins \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 默认回放 imu_100hz.txt，自动启动 RViz2
ros2 launch origami_ins ins.launch.py

# 回放 500 Hz，实际积分 dt 不受播放倍速影响
ros2 launch origami_ins ins.launch.py \
  data_file:="$PWD/data/imu_500hz.txt" playback_rate:=1.0
```

录制格式：`step 0 qw qx qy qz ax ay az wx wy wz;T <整数纳秒时间戳>`。加速度是含重力的比力，单位 m/s²；角速度 rad/s。设备四元数作为录制信息保留，INS 姿态由加速度计和陀螺仪独立估计。时间戳先用整数解析，相减后再转秒，保留纳秒差值精度。

回放默认延迟 2 秒，并等待输入订阅建立后开始；发布 `/clock`，估计节点与 RViz2 使用仿真时间。结束后保留最终时刻和轨迹，可以继续旋转查看。再次回放请重新启动 launch。加 `use_rviz:=false` 可无界面运行，`zupt_mode:=none` 可关闭零速更新。

ROS 2 默认采用 **0.05 秒检测窗口 + 0.05 秒连续静止时间**，以便响应短暂停顿；实际触发时刻还取决于采样时刻、GLRT 统计量及角速度/比力条件。开始时的 2 秒静止初始化用于对准与偏置估计。

两个检测时间参数支持运行中更新：

```bash
ros2 param set /inertial_odometry stationary_window_seconds 0.05
ros2 param set /inertial_odometry minimum_stationary_seconds 0.05
```

更新后重新累计静止时间，保留当前位置、速度、姿态与协方差。检测窗口至少覆盖两个实际采样间隔。降低时间门限可以提前触发；位置闭合效果需结合运动中的姿态反馈和实际停顿共同评价。

## 实时 IMU 或 rosbag

先启动估计节点，再开始提供 IMU，保证最初 2 秒静止：

```bash
# 实时 /imu/data；默认使用消息中的真实时间戳
ros2 launch origami_ins ins.launch.py replay:=false

# 自定义话题及传感器坐标系
ros2 launch origami_ins ins.launch.py replay:=false \
  imu_topic:=/sensor/imu imu_frame:=sensor_imu

# rosbag：单独的终端播放，避免与 TXT 回放同时发布 /clock
ros2 launch origami_ins ins.launch.py replay:=false use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

输入为 `sensor_msgs/msg/Imu`；`header.frame_id` 必须匹配 `imu_frame`，时间戳严格递增。输入订阅采用 SensorData 的 best-effort 策略，兼容常见硬件驱动及可靠发布的 TXT 回放。原始测量按时间顺序完整处理；时间空缺按真实 dt 积分，并重新累计静止时间。初始化阶段缓存数据，对准成功后按顺序处理。初始化检测到运动会停止估计，在 `/diagnostics` 报错；修正输入后重启或调用 reset 服务。

实时输入超过 `input_timeout_seconds`（默认 0.50 秒）没有新消息时，诊断标为 STALE。时间戳间隔超过 `max_integration_interval_seconds`（默认 0.25 秒）时，在修改滤波状态前暂停积分并报错；恢复数据流后调用 reset，再静止对准。低于该上限的短时间空缺按实际 dt 处理。长时间断流不会被当作持有当前加速度持续运动，也不会自动重置坐标原点。

默认 TF 是 `odom -> imu_link`，Raw 对照为 `odom -> imu_raw`。位姿定义在 IMU 原点，机体系与传感器系重合。自定义 `imu_frame` 时，在 RViz 的 Current IMU attitude 中把 Reference Frame 改为对应坐标系。

## RViz2 观察方式

- 青色线：分段显示 ESKF 连续轨迹，位置跳变处断开；橙色 Raw INS 在 Displays 中勾选开启。
- 红/绿/蓝坐标轴：当前 IMU 的 X/Y/Z 朝向；青色箭头沿当前 IMU 的 +X。
- 绿色球：起点；小绿点：成功执行 ZUPT 的位置。
- 红色点：实际输出中发生连续性异常的前后位置。被保护机制拦下的位置修正保留在诊断中，不画成运动位置；诊断异常后方向箭头变红，直到 reset。
- 地面网格每格 0.1 m，场景中的轨迹、坐标轴和标记采用纯图形显示。
- 鼠标拖动旋转，滚轮缩放；轨迹超出初始视角时调整 Views 的 Distance/Focal Point。

局部系 +Z 向上，原点为初始 IMU 位置，初始 yaw=0。航向由陀螺仪相对初始方向传播，因此采用相对航向口径，并随积分时间累积误差。实机实验以相对轨迹、姿态变化和静止段速度为观察量；绝对位置精度以同步位置参考为评价基准。

## ROS 接口

位置健康检查分别记录积分位移、ZUPT 位置修正和最终帧间位移。默认应用门限为：帧间位移不超过 `0.02 m + 2.0 m/s × dt`，单次位置修正不超过 `0.10 m`，位置协方差最大主方向的标准差不超过 `0.50 m`。这些门限在 `config/ins.yaml` 的 `health` 中配置，表示应用连续性与不确定度要求。

NIS 检验约束零速度创新；位置健康检查负责报告连续性丢失。大位移修正可能反映此前累计的状态误差，不能据此断定当前 IMU 帧或零速观测错误。

ROS 2 默认启用 `protect_zupt_position: true`。当 ZUPT 推导的位置修正超过单次修正上限或本帧剩余位移预算时，该次更新的位置增益设为零，速度、姿态与偏置按接受的零速度观测更新。协方差使用修改后的增益进行 Joseph 更新，保留未应用位置修正对应的不确定度。诊断记录被拦下的修正幅度、次数及 NIS，并保持位置估计不可靠的提示。

该保护用于阻止异常大位置修正直接注入实时轨迹，不提供返回起点的位置观测。连续性与位置闭合效果分别评价。配置 `protect_zupt_position: false` 可复现完整位置修正；共享 INS 核心与 MATLAB 原版的默认更新方式保留。

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 输入，可用 `imu_topic` 改名 |
| `/Odometry` | `nav_msgs/msg/Odometry` | ESKF 位姿、机体系速度与协方差 |
| `/ins/raw_odometry` | `nav_msgs/msg/Odometry` | Raw 对照，协方差使用 1e6 对角占位值 |
| `/path` | `nav_msgs/msg/Path` | ESKF 当前连续轨迹段，保留最新消息 |
| `/ins/trajectory_segments` | `visualization_msgs/msg/MarkerArray` | 跳变之前的连续轨迹段，保留最新消息 |
| `/ins/estimate_history` | `nav_msgs/msg/Path` | 所有保留的原始估计点，包含位置修正，受 max_path_points 限制 |
| `/ins/raw_path` | `nav_msgs/msg/Path` | Raw 轨迹，保留最新消息 |
| `/ins/markers` | `visualization_msgs/msg/MarkerArray` | 方向、起点、ZUPT 与红色异常位置 |
| `/tf` | `tf2_msgs/msg/TFMessage` | ESKF 与 Raw 的实时姿态 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 初始化、处理帧数、静止判定、NIS、偏置和异常 |
| `/clock` | `rosgraph_msgs/msg/Clock` | TXT 回放时间 |

```bash
# 查看位置与四元数（ROS 字段顺序 xyzw，内部使用 wxyz）
ros2 topic echo /Odometry --field pose.pose

# 暂停 / 继续 TXT 回放
ros2 service call /imu_replay/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /imu_replay/pause std_srvs/srv/SetBool '{data: false}'

# 清空估计和轨迹，从接下来的静止数据重新对准
ros2 service call /inertial_odometry/reset std_srvs/srv/Trigger '{}'
```

`config/ins.yaml` 配置时间窗口、静止门限、发布频率与轨迹长度；`config_file` 指向根目录 `config.json` 的安装副本，提供 AHRS、ESKF 噪声与量程参数。可通过 `params_file:=/absolute/path/custom.yaml` 提供自定义参数。滤波逐帧处理 IMU，轨迹按 20 Hz 取样、10 Hz 发布，默认最多保留 20000 点，将状态估计频率与显示频率独立配置。初始航向及噪声参数的含义与 MATLAB 相同。

## 数值检查

在仓库根目录运行：

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s test/ros2/tests -v
```

覆盖恒加速度位移、已知角速度朝向、100/500 Hz 静止倾斜对准及偏置、掉帧后的静止检测、初始化运动拒绝、关闭 ZUPT，以及纳秒时间戳精度与重复帧拒绝。
