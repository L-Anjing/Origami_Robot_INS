# 折纸机器人多 IMU 惯性里程计

C++17 / Eigen 实现六 IMU 形态重建、单参考 IMU 的 15 维误差状态 ESKF、GLRT/驱动相位辅助 ZUPT，提供离线计算和 ROS 2 Humble 接口。工程源代码和测试不依赖 Python；ROS 2 工具链自身仍使用其系统依赖。

目录和排版参照 [FAST-LIO 指定提交](https://github.com/zxzzzxz-id/fast_lio/tree/395ff6e600aaafe4727e85a4c81cf10f556f359a)：根目录统一 CMake/ament，公共头文件、处理模块、配置、启动和测试分别组织。保留本工程 ESKF 模型，不引入 LiDAR、PCL、ikd-Tree 或 IKFoM。

```text
Origami_Robot_INS/
├── CMakeLists.txt / package.xml
├── include/
│   ├── common_lib.h          # Eigen类型、IMU记录类型
│   ├── so3_math.h            # SO(3)指数映射、右雅可比
│   ├── eskf.hpp              # AHRS、15维误差状态滤波
│   ├── robot_model.h         # XML几何
│   ├── mat_export.h          # MAT v5输出
│   └── ros2_utils.h          # ROS消息转换
├── src/
│   ├── IMU_Processing.hpp / .cpp  # 实测对准、流式导航、健康检查
│   ├── preprocess.h / .cpp       # 输入解析、GLRT、质量检查
│   ├── robot_odometry.h / .cpp   # 六板重建、参考节点、三策略对照
│   ├── inertialOdometry.cpp      # ROS2导航接口
│   ├── imu_replay.cpp            # ROS2回放与时钟、暂停
│   └── offline_ins.cpp          # 离线入口
├── config/                   # simulation.json、real_imu.json、ins.yaml
├── launch/ins.launch.xml
├── rviz_cfg/ins.rviz
├── tests/                    # C++测试、固定回归数据
├── docs/                     # KF说明、接口说明、验证记录
├── matlab/animate_robot.m
├── robot/                    # 原MJCF模型
└── data/                     # 原始数据
```

## 独立构建

依赖 CMake ≥3.16、C++17 编译器、Eigen3、nlohmann-json、tinyxml2。

```bash
cmake -S . -B build/standalone -DBUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/standalone -j2
ctest --test-dir build/standalone --output-on-failure
```

默认构建 ROS 2；独立计算显式设置 `BUILD_ROS2=OFF`。项目通过 `.clang-format` 统一四空格、Allman 大括号和头文件风格。

## 仿真与 MATLAB

```bash
build/standalone/origami_ins simulation config/simulation.json \
  data/data_L_qawp_20261030.txt outputs/robot robot/zbot_crack_L2.xml all
build/standalone/origami_ins export-mat outputs/robot/estimated
build/standalone/origami_ins export-mat outputs/robot/ground_truth
```

每种姿态模式独立输出 `frames.jsonl`、`summary.json`，按需生成 `robot_odometry.mat`（schema 2）。`estimated` 是正常六轴估计；`ground_truth` 只用于显式调试。端点真值不参与位置校正。参考节点由 `reference_imu_index` 和 `precompute_reference_indices` 配置。

```matlab
addpath('matlab');
animate_robot('outputs/robot/estimated/robot_odometry.mat');
```

## 实测 IMU

```bash
build/standalone/origami_ins real config/real_imu.json \
  data/imu_100hz.txt outputs/real_100hz 2 glrt
build/standalone/origami_ins real config/real_imu.json \
  data/imu_500hz.txt outputs/real_500hz 2 glrt
```

开头2秒保持静止以对准重力和估计偏置初值。真实时间戳使用整数纳秒；设备四元数不作为姿态辅助。实测配置使用50ms GLRT窗口及50ms连续静止时长；非法数据和过大间隔报错，不插值或裁剪原始输入。

## ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log/native build --base-paths . --packages-select origami_ins \
  --build-base build/native_ros --install-base install/native \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/native/setup.bash
ros2 launch origami_ins ins.launch.xml
# 接入实时IMU：
ros2 launch origami_ins ins.launch.xml replay:=false imu_topic:=/imu/data
```

包名 `origami_ins`，可执行文件 `inertial_odometry`、`imu_replay`，原话题名和服务名保留。启动文件改为XML；参数位于 `config/ins.yaml`。详细契约见 [ROS2接口](docs/ROS2接口.md)。

## 数值与边界

名义状态 `(p,v,q,bg,ba)`；误差状态 `[δp,δv,δθ_local,δbg,δba]`。世界Z向上、SI单位、四元数wxyz。仿真使用步末位置积分，实测使用常加速度积分；Joseph更新、右乘注入和协方差重置保留。完整推导见 [KF与零偏说明](docs/KF说明与零偏方案.md)。

C++回归测试读取固定检查点，覆盖两份实测完整记录、两种仿真完整记录及三种ZUPT策略，并检查MAT输出。检查点来自先前已与Python逐帧比对通过的结果；原Python源码、Python测试和依赖清单已删除。历史误差记录保存在 [验证记录](docs/validation/README.md)。

原 `main.py --task baseline` 的离线速度回填实验和Python静态绘图退出当前工程入口；当前支持上述原生计算链及MATLAB可视化。数据、模型、已有实验结果和MATLAB代码保留。删除工程源码不清理用户的 `.venv` 或旧构建/安装缓存。

六轴IMU+ZUPT不能保证绝对航向、位置和所有偏置均可观测；噪声和门限需硬件标定。MAT格式校验和ROS本机链路测试不等同于MATLAB交互验收或实机精度验证。
