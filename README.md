# 折纸机器人多 IMU 形态重建与惯性里程计

本项目面向六板折纸机器人，利用安装在各板上的六个 IMU，同时研究机器人“如何变形”和“如何移动”。项目实现了形态重建、单 IMU 惯性导航、多 IMU 融合与铰链约束滤波，并提供仿真评估、实机 IMU 数据录制和回放、MATLAB 动画及 ROS 2 / RViz2 接口。

算法核心使用 C++17、Eigen、nlohmann/json 和 TinyXML2；离线计算不依赖 ROS 2，实机接口使用 ROS 2，绘图和动画使用 MATLAB。

## 方法与功能

- **六 IMU 形态重建**：结合各板姿态、相邻板铰链关系和 XML 机器人几何模型，估计相对折叠角，通过正向运动学恢复六板构型。
- **单 IMU 惯性导航**：实现 Raw INS 和 15 维误差状态卡尔曼滤波（ESKF），估计位置、速度、姿态及陀螺仪和加速度计零偏。支持 IMU 静止检测和驱动相位辅助零速更新（ZUPT）。
- **多 IMU 融合**：对比单 IMU 积分、多 IMU 均值融合，以及带铰链位置、铰链轴约束的联合 ESKF。联合滤波保留六节点间的交叉协方差，通过几何约束校正导航状态及零偏。
- **标定与评估**：提供静止标定、零噪声一致性诊断，以及白噪声、固定零偏和随机游走条件下的多随机种子对照实验。
- **实机数据链路**：订阅 `sensor_msgs/Imu` 录制 TXT，使用同一输入格式进行离线导航或 ROS 2 回放，通过 Odometry、Path、TF 和诊断消息接入 RViz2。

六轴 IMU 的航向依赖陀螺仪传播，纯惯性定位仍会漂移。当前多 IMU 对照主要基于仿真；噪声参数是可调整的实验假设，实机精度需要结合硬件标定和同步位置参考评估。

## 快速开始：离线构建

以下命令均在仓库根目录执行。Ubuntu 上安装构建依赖：

```bash
sudo apt install build-essential cmake libeigen3-dev nlohmann-json3-dev libtinyxml2-dev
cmake -S . -B build/standalone -DBUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/standalone -j2
ctest --test-dir build/standalone --output-on-failure
```

### 运行六 IMU 联合滤波

```bash
build/standalone/origami_ins multi-imu-run config/multi_imu_eskf.json \
  data/data_L_qawp_20261030.txt outputs/joint robot/zbot_crack_L2.xml
```

输出导航误差 CSV、指标与汇总 JSON 等文件到 `outputs/joint/`。用 MATLAB 绘制轨迹：

```matlab
addpath('matlab');
plot_multi_imu_eskf('outputs/joint');
```

将子命令改为 `multi-imu-suite` 可运行配置中的多种子、噪声和约束对照实验，耗时取决于配置的实验规模。已有图表与指标见[实验结果](results/README.md)，[静止标定与零噪声诊断](results/diagnostics/README.md)单独说明标定及输入一致性。

### 运行形态与参考 IMU 导航动画

```bash
build/standalone/origami_ins simulation config/simulation.json \
  data/data_L_qawp_20261030.txt outputs/robot robot/zbot_crack_L2.xml all
build/standalone/origami_ins export-mat outputs/robot/estimated
```

```matlab
addpath('matlab');
animate_robot('outputs/robot/estimated/robot_odometry.mat');
```

`simulation` 支持 `estimated`、`ground_truth` 和 `all`；真值姿态模式用于验证。形态及融合对照入口为 `shape-only` 和 `shape-fusion`，参数顺序同样是 `CONFIG DATA OUTPUT MODEL`，使用 `config/shape_experiment.json`。

## 实机使用：录制、处理与回放

### 构建 ROS 2 接口

以 Ubuntu 22.04 / ROS 2 Humble 为例，先安装 ROS 2、colcon 及包依赖，再在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select origami_ins \
  --build-base build/native_ros --install-base install/native --cmake-args -DBUILD_ROS2=ON
source install/native/setup.bash
```

### 录制 IMU 数据

先启动设备驱动，使其发布 `sensor_msgs/Imu`，再在已 source 上述环境的终端运行：

```bash
ros2 run origami_ins imu_record --ros-args \
  -p imu_topic:=/imu/data -p output_file:="$PWD/data/imu_new.txt"
```

按 Ctrl+C 结束。输出文件的父目录必须存在，程序拒绝覆盖已有文件。起始至少保持静止 2 秒，供后续导航初始化使用。录制保存消息中的真实纳秒时间戳，不按标称频率重采样；输入加速度必须为包含重力的比力（m/s²），角速度为 rad/s。

TXT 与已有 `imu_100hz.txt` / `imu_500hz.txt` 格式一致：

```text
# step 0 qw qx qy qz ax ay az wx wy wz;T timestamp_ns
1 0 1 0 0 0 0 0 9.81 0 0 0;T 1793623178406190059
```

四元数按 `wxyz` 保存设备原值；消息声明姿态不可用时写入单位四元数占位。单 IMU 离线导航和回放使用比力、角速度及时间戳。非有限数值、负时间戳和非递增时间戳会被丢弃。录制使用 SensorDataQoS，适合传感器话题；消息是否丢失还取决于驱动及传输情况。

已有数据头中的 `record_imu.py` 是历史录制工具名称，当前仓库和可查 Git 历史中未找到该脚本；这里提供的 `imu_record` 是兼容 TXT 格式的 C++ 录制入口。

### 离线处理实测记录

```bash
build/standalone/origami_ins real config/real_imu.json \
  data/imu_new.txt outputs/real 2 glrt
```

输出 `frames.jsonl` 和 `summary.json`。参数 `2` 为初始化静止时长（秒），`glrt` 可替换为 `none` 以关闭零速更新。

### ROS 2 回放与实时导航

```bash
ros2 launch origami_ins ins.launch.xml data_file:="$PWD/data/imu_new.txt"
# 接入设备实时话题
ros2 launch origami_ins ins.launch.xml replay:=false imu_topic:=/imu/data
```

默认启动 RViz2；无图形环境可加 `use_rviz:=false`。不指定数据文件时回放仓库自带的 `imu_100hz.txt`。话题、服务、参数和坐标帧约定见 [ROS 2 接口](docs/ROS2接口.md)。当前 ROS 2 导航入口为单 IMU，六 IMU 联合滤波通过离线入口运行。

## 数据、参数与坐标约定

- `data/data_L_qawp_20261030.txt`：六 IMU 仿真输入；`data/read_data_L_qawp.txt` 为相关数据说明。
- `data/imu_100hz.txt`、`data/imu_500hz.txt`：单 IMU 实测记录。补充生成数据放在 `data/additional/`，方法见[补充数据说明](data/data_add_README.md)。
- `config/`：单 IMU 导航、形态实验、多 IMU 联合滤波及 ROS 2 参数；`robot/`：机器人 XML 几何模型。
- `include/`、`src/`：算法和程序入口；`tests/`：回归与数值测试；`matlab/`：动画与绘图；`docs/`：算法推导与接口说明；`outputs/`：运行生成的输出。

世界系 +Z 向上，使用 SI 单位，四元数为 `wxyz`；重力补偿为 `a_W = R_WS f_S + [0, 0, -9.81]`。S0–S5 对应 V0–V5，实体板长 50 mm。关节角 0 rad 表示完全展开（板间内角 180°）。仿真真值用于评估或明确标注的验证模式；比较结果时须同时核对初始化先验、噪声、ZUPT 和真值辅助配置。

详细方法见[紧耦合多 IMU 惯性里程计](docs/紧耦合多IMU惯性里程计.md)、[形态约束与多 IMU 融合实验](docs/形态约束与多IMU融合实验.md)和[滤波与零偏说明](docs/KF说明与零偏方案.md)。
