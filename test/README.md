# 实机 IMU 三维轨迹测试

以真实时间戳统一处理 100 Hz 和 500 Hz 实机数据，将六轴姿态估计、15 状态 ESKF 与静止事件零速更新结合，在同一流程中呈现轨迹、朝向、速度及偏置估计。MATLAB 提供并列三维轨迹与状态曲线，[ROS 2 + RViz2](ros2/README.md) 提供 TXT 回放和实时 IMU 接入。

| 实测数据 | 帧数 | 时长 |
| --- | ---: | ---: |
| `data/imu_100hz.txt` | 4367 | 43.66 s |
| `data/imu_500hz.txt` | 12166 | 24.33 s |

在 MATLAB 工程根目录运行，通过同一个程序选择数据：

```matlab
addpath('test');
result100 = test_real_imu('data/imu_100hz.txt');
result500 = test_real_imu('data/imu_500hz.txt');
```

直接运行 `result = test_real_imu;` 默认读取 100 Hz 数据。运行环境为 MATLAB R2018b 及以上，使用基础功能。

- **三维轨迹**：并列展示 Raw INS 和 ESKF 轨迹，颜色编码时间，标记起点、终点及成功的零速更新，支持鼠标旋转。
- **状态曲线**：展示 XYZ 位移、速度、比力与角速度模长，并标记静止候选和零速更新时刻。

初始化采用开头 2 秒静止数据，完成重力对准、陀螺仪偏置估计及沿重力方向的加速度偏差校正；初始位置、速度和航向角设为零。静止检测窗口按秒定义，两种采样率共用 0.15 秒窗口和 0.20 秒最短连续静止时间，滤波参数读取 `config.json`。

`result.eskf.position_W_m` 为 N×3 位移，`result.eskf.quaternion_wxyz` 为 N×4 姿态四元数；偏置估计保存在 `gyro_bias_rad_s` 和 `accel_bias_m_s2` 字段。结果采用局部坐标系，+Z 向上，位移以首帧为原点。

实验以 Raw INS 和 ESKF 的轨迹形态、静止段速度及零速更新响应，分析静止约束对惯性状态的作用。录制动作作为静止区间的核对依据；位置精度实验以同步位置参考为评价基准。
