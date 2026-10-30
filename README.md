# Multi-IMU Shape Reconstruction & Inertial Odometry

面向三段六节折纸机器人的 Python–MATLAB 验证工程：六 IMU 重建机器人形态，单参考 IMU 提供在线惯性里程计，在同一动画中同步展示机器人运动与轨迹误差。

## 技术方案

- **六 IMU 形态重建**：六轴 AHRS 估计各板姿态，结合 MuJoCo XML 的板体尺寸和连接偏移，通过正向运动学恢复六板构型，保留节段间相对转动。
- **单参考 IMU 导航**：Raw INS 与 15 维误差状态 ESKF，包含位置、速度、姿态及陀螺仪/加速度计偏置；采用 Joseph 协方差更新与误差状态注入。
- **驱动相位辅助 ZUPT**：因果尾随 SHOE/GLRT 检测结合三路驱动切换事件，提供 `none`、`glrt`、`phase` 三种对照策略；通过有效性与创新检验后执行零速更新。
- **高冲击质量管理**：标记冲击及潜在超量程输入，控制加速度姿态反馈与静止窗口有效性，保留原始惯性输入。
- **MATLAB 联合可视化**：真实尺寸六板动画、Raw/ESKF/真值轨迹、三维与 XYZ 位置误差、累计 RMSE 和实际 ZUPT 更新标记，支持暂停、时间滑块及播放速度调整。

形态与导航独立计算，通过参考 IMU 的估计位置组合显示。INS 核心依赖 NumPy，与数据读取、绘图和 MAT 导出分离。

## 快速开始

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

输出位于 `outputs/robot/`，包含 `robot_odometry.mat`、`summary.json` 和静态图。XML 几何读取无需安装 MuJoCo Python 包。

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

# 运行测试
.\.venv\Scripts\python.exe -m unittest discover -s tests
```

在 `config.json` 中配置参考节点、初值、ESKF 噪声、GLRT/驱动相位阈值及传感器量程。噪声与阈值为可调实验参数，实际硬件部署前需标定。MATLAB 菜单列出已预计算的参考节点。

## 坐标与评估约定

世界系 +Z 向上，本次运动方向为 −X；单位采用 SI，四元数顺序为 `wxyz`，重力补偿为 `a_W = R_WS @ f_S + [0, 0, -9.81]`。实体板长 50 mm，模型连接链节距为 50.1 mm。

`estimated` 使用陀螺仪和加速度计；`ground_truth` 使用仿真姿态验证几何与积分链路。S0–S5 对应 V0–V5，端点真值用于独立评估和显示，导航位置误差按世界坐标直接计算。ZUPT 以实际完成的卡尔曼更新计数为准；六轴姿态的航向由陀螺仪传播。

方法参考：[Gait-Tracking](https://github.com/xioTechnologies/Gait-Tracking) · [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)
