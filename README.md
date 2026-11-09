# 折纸机器人多 IMU 形态重建与惯性里程计

折纸机器人的运动同时包含节段形变与整体位移，需要在同一时间轴上理解机器人“如何变形、如何移动”。本项目将六 IMU 姿态估计、几何运动学和惯性里程计连接起来：由各板姿态恢复三段六节构型，由参考 IMU 的 ESKF 估计整体运动，并同步呈现三维形态、轨迹与零速更新。

核心优势是**形态与位移协同呈现、静止事件约束惯性状态、实测数据统一接入**。MATLAB 提供六板动画和定量评估，ROS 2 + RViz2 提供实机轨迹与朝向验证；100 Hz 与 500 Hz 记录共用真实时间戳驱动的处理流程。

## 技术方案

- **六 IMU 形态重建**：六轴 AHRS 估计各板姿态，结合 MuJoCo XML 的板体尺寸和连接偏移，通过正向运动学恢复六板构型，保留节段间相对转动。
- **参考 IMU 惯性里程计**：15 维误差状态 ESKF 联合估计位置、速度、姿态及陀螺仪/加速度计偏置，通过 Joseph 协方差更新和误差状态注入融合零速约束；Raw INS 提供同传播模型的对照。
- **驱动相位辅助 ZUPT**：因果尾随 SHOE/GLRT 检测结合三路驱动切换事件，提供 `none`、`glrt`、`phase` 三种对照策略；通过有效性与创新检验后执行零速更新。
- **高冲击质量管理**：标记冲击及潜在超量程输入，控制加速度姿态反馈与静止窗口有效性，保留原始惯性输入。
- **MATLAB 联合可视化**：真实尺寸六板动画、Raw/ESKF/真值轨迹、三维与 XYZ 位置误差、累计 RMSE 和实际 ZUPT 更新标记，支持暂停、时间滑块及播放速度调整。
- **ROS 2 实机验证**：TXT 按时间戳回放与实时 IMU 订阅共用 INS 核心，通过 Odometry、Path 和 TF 在 RViz2 中呈现轨迹、姿态坐标轴、方向箭头及 ZUPT 位置。

形态估计与惯性里程计采用模块化计算，由参考 IMU 位姿统一呈现。INS 核心基于 NumPy，可复用于离线分析和 ROS 2 消息处理。

## 实机轨迹与朝向：ROS 2 + RViz2

Ubuntu 22.04 / ROS 2 Humble，在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths test/ros2 --packages-select origami_ins \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch origami_ins ins.launch.py
```

默认回放 `data/imu_100hz.txt`。RViz2 以青色轨迹显示 ESKF 位置，以 XYZ 坐标轴和 +X 箭头显示当前朝向，支持开启 Raw INS 对照。切换 500 Hz 数据或接入实时 IMU：

```bash
ros2 launch origami_ins ins.launch.py data_file:="$PWD/data/imu_500hz.txt"
ros2 launch origami_ins ins.launch.py replay:=false imu_topic:=/imu/data
```

输入起始 2 秒保持静止，用于重力对准及初始偏置估计。完整接口、坐标定义与参数见 [ROS 2 使用说明](test/ros2/README.md)；[MATLAB 实机测试](test/README.md) 提供同一数据的三维轨迹和状态曲线。

## 六 IMU 形态与轨迹：Python + MATLAB

在项目根目录执行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py
```

默认计算 `ground_truth` 和 `estimated` 两种姿态模式，参考节点为 S0。输入为：

```text
data/data_L_qawp_20261030.txt
config.json
robot/zbot_crack_L2.xml
```

输出位于 `outputs/robot/`，包含 `robot_odometry.mat`、`summary.json` 和静态图。几何模块直接读取 MuJoCo XML 中的板体尺寸与连接关系。

在 MATLAB 中进入项目根目录：

```matlab
addpath('matlab');
animate_robot
```

可指定显示模式和播放参数：

```matlab
animate_robot([], 'Mode', 'estimated', 'ReferenceIMU', 0, ...
    'Method', 'eskf', 'ZUPTMode', 'phase', 'Stride', 25, 'PlaybackSpeed', 1)
```

`Method` 支持 `raw`、`eskf`；`ZUPTMode` 支持 `none`、`glrt`、`phase`。误差图与机器人动画同步，界面显示当前误差、截至播放时刻的累计 RMSE、数据集末帧误差及实际 ZUPT 更新次数。

## 选择参考节点与配置

```powershell
# 使用 S2，计算 IMU 估计姿态模式
.\.venv\Scripts\python.exe main.py --reference-imu 2 --mode estimated

# 预计算 S0、S2、S4，供 MATLAB 菜单切换
.\.venv\Scripts\python.exe main.py --reference-imu 0 --precompute-references 2 4

# 导出 MAT 和汇总，跳过静态图
.\.venv\Scripts\python.exe main.py --no-plots

```

在 `config.json` 中集中配置参考节点、初值、ESKF 噪声、GLRT/驱动相位阈值及传感器量程，支持将硬件标定结果直接用于对应实验。MATLAB 菜单列出已预计算的参考节点。

## 实验设计

| 实验 | 论证职责 | 观察量 |
| --- | --- | --- |
| 六 IMU 形态重建与参考位姿组合 | 展示节段形变与整体运动的协同表达 | 六板构型、参考轨迹、时间同步动画 |
| Raw INS 与 ESKF 对照 | 分析零速约束对惯性状态的作用 | 轨迹形态、静止段速度、ZUPT 更新响应 |
| `none` / `glrt` / `phase` 策略对照 | 分析 IMU 静止检测与驱动相位信息的约束作用 | 更新时刻、更新次数、仿真位置误差 |
| 100 Hz / 500 Hz 实测数据 | 验证真实时间戳驱动的统一处理流程 | 采样间隔、处理帧数、轨迹与朝向 |

仿真实验以端点真值计算三维及 XYZ 位置误差和累计 RMSE。实机实验围绕轨迹、朝向及静止事件响应展开；位置精度实验采用同步位置参考作为评价基准。

INS 数值检查覆盖恒加速度积分、已知角速度朝向、两种采样率下的静止对准与偏置估计：

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s test/ros2/tests -v
```

## 坐标与评估约定

世界系 +Z 向上，本次运动方向为 −X；单位采用 SI，四元数顺序为 `wxyz`，重力补偿为 `a_W = R_WS @ f_S + [0, 0, -9.81]`。实体板长 50 mm，模型连接链节距为 50.1 mm。

`estimated` 使用陀螺仪和加速度计；`ground_truth` 使用仿真姿态验证几何与积分链路。S0–S5 对应 V0–V5，端点真值用于独立评估和显示，导航位置误差按世界坐标直接计算。ZUPT 以实际完成的卡尔曼更新计数为准；六轴姿态的航向由陀螺仪传播。

方法参考：[Gait-Tracking](https://github.com/xioTechnologies/Gait-Tracking) · [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)
