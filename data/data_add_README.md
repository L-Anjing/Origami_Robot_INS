# 补充数据

`additional/` 独立保存生成数据，不修改原始动态记录。

- `static_ideal.csv`：六IMU展开静止5秒，1000 Hz；时间−4.999至0秒，角速度0、比力[0,0,9.81] m/s²。已知单位姿态、零初速，根位置[0.2,-0.11,0.02] m，相邻偏移0.0501 m。
- `static_fixed_bias.csv`、`static_full.csv`：种子42的带噪静止示例。前者含白噪声与固定零偏，后者再加随机游走；参数沿用 `config/multi_imu_eskf.json`。陀螺单位rad/s，比力单位m/s²。
- `*_bias_truth.csv`：独立评估用的零偏真值，标定器不读取。
- `consistent_motion.txt`：复用现有FK生成器的组合运动，使用与step-end积分配对的区间等效惯性量，仅作零噪声一致性对照。

0.5/1/2/5秒标定分别取同一5秒记录的末尾窗口。噪声生成器连续接入动态阶段，不重新播种；各时长共用完全相同的动态测量与零偏轨迹。动态时间仍从0.001秒开始，静止阶段不计入导航积分。因多消耗了5秒噪声，动态随机序列不同于早期无静止前缀实验。

仅保存种子42的原始补充数据；42–71的统计在 `results/diagnostics/`。正常标定逐IMU取陀螺均值，并按已知姿态减去重力比力估计加计零偏。未知姿态不能这样辨识全部加计零偏。

生成与验证：`build/standalone/calibration_study config/multi_imu_eskf.json data/data_L_qawp_20261030.txt robot/zbot_crack_L2.xml results/diagnostics data/additional`。
