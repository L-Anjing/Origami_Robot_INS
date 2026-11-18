> 旧实验输出已清理；当前精简结果见[结果说明](../results/README.md)。

# 运动学约束下的紧耦合多 IMU 惯性里程计

折纸机器人以铰链把独立惯性轨迹连接成同一个运动系统。联合滤波将共享铰链的位置与旋转轴一致性转化为导航校正，使形态约束直接作用于位置、速度、姿态和零偏。铰链位置与轴约束采用90维误差状态；显式关节状态联合滤波进一步显式跟踪五个折叠角及共享陀螺噪声，形成95维误差状态。

## 1. 架构依据与实现范围

[DogLegs论文](https://arxiv.org/abs/2503.04580)及[官方代码](https://github.com/YibinWu/DogLegs)展示了将多个惯性单元置于同一估计器、通过机器人运动学联系各惯性状态的架构。参考阅读的模块为imufusion.cpp/.h与inspropagation.cpp/.h，具体blob记录于outputs/joint/manifest.json。以下公式为本项目右乘局部姿态误差约定的独立实现。

DogLegs具有独立关节编码器与腿式接触信息。本项目关节角来自同一批陀螺，因此角度传播与节点姿态传播必须共享噪声；不能把积分角当独立编码器反复更新。当前不引入腿式接触假设，不复制其误差符号或尺度因子状态。联合滤波直接观测铰链，不把姿态辅助关节角估计积分角作为外部测量。

## 2. 坐标、形态与状态

Rᵢ把节点局部坐标变换到世界系，pᵢ是IMU原点世界位置，g=[0,0,−9.81]ᵀ。右乘误差约定为 Rᵢ,true = R̂ᵢ Exp([δθᵢ]×)。单节点名义状态：

$$x_i=(p_i,v_i,q_i,b_{g,i},b_{a,i}),\qquad
\delta x_i=[\delta p_i^T,\delta v_i^T,\delta\theta_i^T,\delta b_{g,i}^T,\delta b_{a,i}^T]^T.$$

q为单位四元数，存储4个标量但误差姿态3维。六节点误差堆叠为δX∈R⁹⁰，保存完整90×90协方差。显式关节状态联合滤波追加五个δϑ，使δX∈R⁹⁵；关节角ϑ与姿态误差δθ使用不同符号。

初始先验在config/multi_imu_eskf.json：p₀=[0.2,−0.11,0.02] m，q₀=[1,0,0,0]，ϑ=[0,0,0,0,0]，所有初始线速度、角速度、关节速度为0。FK给出pᵢ=[0.2+0.0501i,−0.11,0.02] m。五轴为+Z、−Z、+Z、−Z、+Z；折叠角0表示直线展开，通常板间内角为180°。数据第一帧是t=0.001 s，先执行传播，不把首帧真值覆写到状态。XML根位置与这段日志的初始世界位置不同，采用配置中注明的日志先验。

## 3. 离散传播与Φ、Q

记Δt=0.001 s，ω=ωₘ−b̂g，f=fₘ−b̂a，C=Exp([ωΔt]×)。当前step_end更新：

$$R^+=RC,\quad a^+=R^+f+g,\quad
v^+=v+a^+\Delta t,\quad p^+=p+v\Delta t+a^+\Delta t^2.$$

偏置名义值在传播中保持，随机游走进入Q。constant_acceleration旧入口采用1/2位置系数，本实验采用step_end系数1。以下A/B/S使用本次更新后的R：

$$A=C^T,\quad B=-J_r(\omega\Delta t)\Delta t,\quad S=-R^+[f]_\times.$$

关闭AHRS反馈时，单节点精确对应代码的离散线性化：

$$\Phi_i=\begin{bmatrix}
I&I\Delta t&SA\Delta t^2&SB\Delta t^2&-R^+\Delta t^2\\
0&I&SA\Delta t&SB\Delta t&-R^+\Delta t\\
0&0&A&B&0\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix}.$$

采用已离散的噪声增量η=[ηg,ηa,ηwg,ηwa]。令N=−Jr(ωΔt)，噪声映射为：

$$G_i=\begin{bmatrix}
SN\Delta t^2&-R^+\Delta t&0&0\\
SN\Delta t&-R^+&0&0\\
N&0&0&0\\
0&0&I&0\\
0&0&0&I
\end{bmatrix},\quad
D_i=\operatorname{diag}(\sigma_g^2\Delta t I,\sigma_a^2\Delta t I,\sigma_{wg}^2\Delta t I,\sigma_{wa}^2\Delta t I).$$

Qᵢ=GᵢDᵢGᵢᵀ。密度来自robot.eskf.noise_density；quality判定为冲击的样本按配置放大过程噪声，保留原始未裁剪输入。这里密度与实验注入的“每采样白噪声标准差”是不同配置项，未宣称二者完成硬件标定。

联合滤波的Φ=diag(Φ₀,…,Φ₅)，独立噪声时Q=diag(Q₀,…,Q₅)，但P不再块对角：

$$P^- = \Phi P^+\Phi^T+Q.$$

代码通过稀疏Φ乘完整P，跨节点相关项持续传播。ρ相关偏置实验使不同节点初始偏置及游走的同名局部轴具有ρσ²交叉项，白噪声保持独立。该实验定义是局部轴相关偏置，不等于任意姿态下同一个世界系干扰向量。

## 4. 共享初始协方差

六节点共享同一个初始根位姿与速度误差，而非给六条初始轨迹分配互不相关的位姿先验。令rᵢ为根系FK位移：

$$\delta p_i=\delta p_0-R_0[r_i]_\times\delta\theta_0,\qquad
\delta v_i=\delta v_0,\qquad
\delta\theta_i=R_i^TR_0\delta\theta_0.$$

由9维根位姿/速度误差和36维独立偏置构造B∈R⁹⁰ˣ⁴⁵，P₀=BΣBᵀ；需要时再加入相关偏置块。显式关节状态联合滤波已知精确零关节先验，初始关节误差方差为0。P₀半正定符合共享先验，不添加虚构独立噪声。速度映射式适用于本轮全部零初始角速度与关节速度；若未来使用非零初始转动，应补充杆臂速度对姿态/角速度先验的协方差传播，不能只修改均值配置。

## 5. 铰链位置约束：共享铰链点

父节点i、子节点j。MJCF给出父子静态平移t、静态旋转Rrest与子系铰链锚点a，令lᵢ=t+Rrest a，lⱼ=a。观测为：

$$0=h_p(X)=p_i+R_i l_i-p_j-R_j l_j.$$

按单节点[p,v,θ,bg,ba]排列，非零H块为：

$$H_{p_i}=I,\quad H_{p_j}=-I,\quad
H_{\theta_i}=-R_i[l_i]_\times,\quad H_{\theta_j}=R_j[l_j]_\times.$$

观测残差ν=−h。这里没有将五个轴固定为同一姿态，也不要求板间速度相同。位置约束的雅可比对零偏列为0，但H通过传播后的P交叉项影响零偏。

## 6. 铰链位置与轴约束：二维关节轴

uᵢ、uⱼ为相同有向铰链轴在两节点局部系的表达。a=Rᵢuᵢ、b=Rⱼuⱼ；取a切平面的正交基E∈R³ˣ²：

$$0=h_a(X)=E^T(a\times b).$$

每次线性化固定E，非零块为：

$$H_{\theta_i}=E^T[b]_\times R_i[u_i]_\times,\qquad
H_{\theta_j}=-E^T[a]_\times R_j[u_j]_\times.$$

使用2维观测，保留沿铰链轴的折叠自由度。实现另行拒绝反平行轴，避免叉积对180°错向也为0的歧义。有限差分测试冻结E，并覆盖非零残差。

## 7. 批更新、全状态注入与重置

分别以单个3维位置/2维轴残差计算NIS，按阈值筛选，接受的观测在同一个先验上堆叠成一批更新。R为配置几何容差的块对角矩阵：它是软运动学模型容差，不是另一套独立传感器。几何观测自身不读取本帧陀螺，联合滤波不额外加入由同一陀螺计算的“独立角度测量”。

$$S=HP^-H^T+R,\quad K=P^-H^TS^{-1},\quad\delta\hat X=K\nu.$$

S使用LDLT，不显式求逆。Joseph以代数等价的分解形式计算：L=P−KHP，Ppost=L−LHᵀKᵀ+KRKᵀ。六节点全部注入：

$$p\leftarrow p+\delta p,\quad v\leftarrow v+\delta v,\quad
q\leftarrow q\otimes\operatorname{Exp}(\delta\theta),\quad
b_g\leftarrow b_g+\delta b_g,\quad b_a\leftarrow b_a+\delta b_a.$$

其中Exp用于四元数时表示同一SO(3)旋转增量。完整重置矩阵的每个姿态块为Jr(δθ)，其余为I；P←Jreset Ppost Jresetᵀ，包括跨节点与节点—关节块，最后对称化。

**零偏估计发生在这里。** 传播通过Φθ,bg、Φv,ba、Φp,ba建立相关性；几何残差经K的偏置行修正bg、ba，再在后续采样中先减去修正后的偏置。与旧“只有ZUPT触发才校正偏置”的独立ESKF相比，联合滤波新增运动学反馈通道。

## 8. 显式关节状态联合滤波：95维关节扩展与共享过程噪声

每个关节以相邻节点同一批带噪陀螺传播：

$$\vartheta_j^+=\vartheta_j+[u_c^T(\omega_{m,c}-b_{g,c})-u_p^T(\omega_{m,p}-b_{g,p})]\Delta t.$$

新增Φ关节行：Φϑj,ϑj=1，Φϑj,bgp=+uₚᵀΔt，Φϑj,bgc=−u꜀ᵀΔt。噪声映射对父陀螺列为+uₚᵀ、子列−u꜀ᵀ，使用节点传播已有ηg和D；因此Qϑϑ包含相邻关节共享陀螺项，Qnode,ϑ通常非零。实现从72列六IMU噪声构造扩展Q，而不是给关节添加独立角度噪声。

显式关节状态联合滤波采用FK位置与相对姿态约束，**不再叠加联合滤波的同一组铰链几何**。记rᵢ(ϑ)、Fᵢ(ϑ)为根系FK：

$$h_{r,i}=R_0^T(p_i-p_0)-r_i(\vartheta),\qquad
h_{R,i}=\operatorname{Log}(F_i^T R_0^T R_i).$$

记dᵢ=R₀ᵀ(pᵢ−p₀)，根系第j轴为aⱼ、锚点oⱼ，j为i祖先时：

$$H_{p_0}=-R_0^T,\ H_{p_i}=R_0^T,\ H_{\theta_0}=[d_i]_\times,\
H_{\vartheta_j}=-a_j\times(r_i-o_j).$$

旋转残差e=Log(FᵢᵀR₀ᵀRᵢ)：

$$H_{\theta_i}=J_r^{-1}(e),\quad H_{\theta_0}=-J_l^{-1}(e)F_i^T,\quad
H_{\vartheta_j}=-J_l^{-1}(e)F_i^Ta_j.$$

非祖先角的列为0。SO(3)对数使用四元数最短分支，逆雅可比含小角度展开。关节增量加性注入，其余按第7节重置。T20检查非零旋转残差的观测有限差分、共享Q符号与相邻关节相关项以及Φ偏置符号；显式关节状态联合滤波不把五个积分角伪装为外部编码器。

## 9. 可观测性、噪声与接触范围

共享几何对整体平移、共同世界速度和整体绕重力轴的旋转不提供绝对信息；规范方向HδX=0由测试验证。初始位置/航向先验定义世界系，后续仍需外部源才能约束绝对漂移。零偏可辨识性还依赖运动激励；平面折叠与短序列不代表任意三轴偏置都可独立准确恢复。

本轮不用共同速度伪约束。若将陀螺和杆臂构造的速度残差当作独立测量，会遗漏与传播噪声的相关性；相应开关目前显式报错。接触更新也关闭，驱动开关不是接触或世界零速度观测。零速辅助惯性导航为独立对照，只在IMU GLRT及门限通过时尝试ZUPT；它可能在低动态平移时误判。

NIS、创新、协方差特征值与跨块范数全部保存。当前过程噪声为原ESKF声明参数，注入扰动为另一组受控仿真假设，不能由小NIS宣称一致性。全局规范模式未做可观测子空间投影，因此不提供全状态NEES结论。

## 10. 结果、指标与数据一致性

（旧输出已清理）包含seed42与全部30种子条件。四种噪声、相关偏置、噪声强度、真实拓扑IMU子集、更新频率及五种新运动场景合计426组。ATE无对齐，RPE为1 s世界平移增量，保留各自定义；表格另有姿态/航向/XYZ/绝对末端误差。

原始无噪声单IMU积分的ATE约3.84e−9 m，但六节点多IMU均值融合约0.0331 m；这是输入比力、陀螺积分与记录节点离散轨迹的不同一致性条件造成的，不以真值修补估计。旧审计见outputs/shape/summary.json的input_consistency_audit。新增五种场景由同一FK产生区间等效惯性量，独立测试理想传播与全部节点真值一致性。

铰链位置与轴约束在原始full的30种子平均ATE为0.451 m，相对多IMU均值融合的0.475 m降低5.02%；局部轴相关偏置ρ=0.5时降低26.82%；新combined_motion场景降低63.62%。每个数字绑定其条件，全部方法和无增益条件见完整报告。显式关节状态联合滤波主要展示数学一致的关节状态扩展，本轮导航结果采用铰链位置与轴约束。

## 11. 文件与运行

| 文件 | 职责 |
|---|---|
| include/src/kinematic_constraints.h/.cpp | 铰链位置、二维轴、FK与SO(3)观测及雅可比 |
| include/src/multi_imu_eskf.h/.cpp | 90/95维传播、共享协方差、筛选批更新、注入重置 |
| include/src/multi_imu_experiment.h/.cpp | 同输入基线、426组并行实验、指标与诊断输出 |
| include/src/synthetic_motion.h/.cpp | 五类离散一致新场景 |
| include/eskf.hpp, src/eskf.cpp | 暴露原传播Φ、G、D、Q，保持旧路径计算 |
| include/src/imu_noise.h/.cpp | 独立/相关偏置及噪声生成 |
| config/multi_imu_eskf.json | 初值、软几何噪声、种子和消融方案 |
| src/offline_ins.cpp, CMakeLists.txt | 新CLI与构建入口 |
| tests/test_multi_imu_eskf.cpp | T1–T20数学与状态传播测试 |
| tests/test_synthetic_motion.cpp, test_joint_experiment.cpp | 全节点理想传播、零速辅助惯性导航合法静止更新回归 |
| matlab/plot_multi_imu_eskf.m | 从CSV绘制带关键指标的轨迹 |

上表include/src简写表示对应include目录中的.h与src目录中的.cpp，并非额外嵌套目录。代码沿用include/src/config/matlab结构，核心计算不依赖ROS；ROS2适配入口保持原接口。本轮未将联合估计CLI自动替换为实时六IMU ROS节点。

```bash
cmake -S . -B build/standalone -DBUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/standalone -j2
ctest --test-dir build/standalone --output-on-failure
build/standalone/origami_ins multi-imu-run config/multi_imu_eskf.json \
  data/data_L_qawp_20261030.txt results/joint_one robot/zbot_crack_L2.xml
build/standalone/origami_ins multi-imu-suite config/multi_imu_eskf.json \
  data/data_L_qawp_20261030.txt results/joint_suite robot/zbot_crack_L2.xml
```

MATLAB可执行`addpath('matlab'); plot_multi_imu_eskf('outputs/joint/plot_runs/full');`。本轮已交付图由环境制图工具生成，MATLAB脚本未在当前环境运行。

27/27 CTest通过，原导航回归和所有旧夹具保留。新增测试验证初始化/首帧、位置/轴有限差分、轴自由度与符号、全状态注入、跨节点协方差、重置与PSD、禁用约束退化、真值隔离、随机种子、规范方向、显式关节状态联合滤波共享过程噪声和FK观测雅可比。性能记录为100/200/1000 Hz独立进程CPU均值，不据均值保证硬实时。

## 12. 低维状态设计的后续研究边界

可将状态组织为根p/v/R（9维误差）、五关节角和角速度（10维）、六IMU偏置（36维），共55维；若根角速度也作为状态则58维。各IMU位姿由FK确定，比90/95维表示更紧凑。其难点转为对根角加速度、关节角加速度、杆臂项以及同一陀螺在状态传播/多IMU测量中的相关性建模。当前实现以90/95维作为可检验基线，此段为设计分析，不宣称已经运行第二套低维滤波器。

验证记录：（旧输出已清理）、（旧输出已清理）、（旧输出已清理）、（旧输出已清理）、（旧输出已清理）。
