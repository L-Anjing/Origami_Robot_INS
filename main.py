"""Six-AHRS shape + single-reference ESKF-ZUPT; legacy INS via --task baseline."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from ins.data_loader import describe, load_data
from ins.evaluate import evaluate, reference_positions, sampling_consistency
from ins.inertial_navigation import integrate_acceleration, integrate_velocity, world_acceleration
from ins.orientation import attitude_error_degrees, gyro_rotations, truth_rotations
from ins.visualize import plot_common, plot_comparison, plot_run
from ins.zupt import correct_velocity, detect_stationary, runs


ROOT = Path(__file__).resolve().parent


def arguments():
    parser = argparse.ArgumentParser(description="六AHRS形状与单参考IMU在线ESKF-ZUPT；右手W系、Z向上，SI单位。")
    default_data = ROOT / "data" / "data_L_qawp_20261030.txt"
    if not default_data.exists():
        default_data = ROOT / "data_L_qawp_20261030.txt"
    parser.add_argument("--data", type=Path, default=default_data)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "robot")
    parser.add_argument("--task", choices=["robot", "baseline"], default="robot", help="完整机器人原型或保留的独立INS基线")
    parser.add_argument("--model", type=Path, default=ROOT / "robot" / "zbot_crack_L2.xml")
    parser.add_argument("--attitude", choices=["six_axis", "gyro"], default="six_axis")
    parser.add_argument("--reference-imu", type=int, choices=range(6), help="唯一参考节点，覆盖reference_imu_index")
    parser.add_argument("--precompute-references", type=int, nargs="+", choices=range(6), help="额外单节点实验，供MATLAB真实切换；不进行位置融合")
    parser.add_argument("--no-plots", action="store_true", help="只导出MAT/汇总，不生成静态PNG")
    parser.add_argument("--mode", choices=["ground_truth", "estimated", "all"], default="all")
    parser.add_argument("--zupt", choices=["none", "offline", "both"], default="both", help="仅旧baseline任务；robot导出none/glrt/phase在线对照")
    parser.add_argument("--integration", choices=["trapezoid", "step_end"], help="覆盖配置中的积分方法")
    parser.add_argument("--detector", choices=["imu", "drive"], help="仅旧baseline任务；robot使用因果GLRT/相位GLRT/无ZUPT三个对照")
    parser.add_argument("--initial-velocity", nargs=3, type=float, metavar=("VX", "VY", "VZ"),
                        help="参考节点初速，m/s；旧baseline为六节点共用初速")
    parser.add_argument("--initial-epoch", choices=["simulation_start", "first_sample"],
                        help="初速所属时刻：t=0 或首个记录时刻 t=0.001 s")
    return parser.parse_args()


def validate_config(config):
    if config["sample_rate_hz"] != 1000 or config["gravity_m_s2"] != 9.81:
        raise ValueError("当前说明固定为1000 Hz、g=9.81；修改前需证明数据定义改变")
    if config["integration"] not in ("trapezoid", "step_end"):
        raise ValueError("Unknown integration method")
    if config["initial_velocity_epoch"] not in ("simulation_start", "first_sample"):
        raise ValueError("Unknown initial velocity epoch")
    velocity = np.asarray(config["initial_velocity_m_s"], dtype=float)
    if velocity.shape not in ((3,), (6, 3)) or not np.isfinite(velocity).all():
        raise ValueError("initial_velocity_m_s must be a finite vector of shape (3,) or (6,3)")
    z = config["zupt"]
    for key in ("gyro_threshold_rad_s", "force_norm_tolerance_m_s2", "minimum_duration_s"):
        if not np.isfinite(z[key]) or z[key] <= 0:
            raise ValueError(f"zupt.{key} must be positive and finite")
    for key in ("drive_before_s", "drive_after_s"):
        if not np.isfinite(z[key]) or z[key] < 0:
            raise ValueError(f"zupt.{key} must be nonnegative and finite")
    if z["detector"] not in ("imu", "drive"):
        raise ValueError("detector must be imu or drive")
    mapping = config["truth"]["endpoint_indices"]
    if len(mapping) != 6 or any(i is not None and (type(i) is not int or i < 0 or i > 6) for i in mapping):
        raise ValueError("truth.endpoint_indices must have six entries, each null or an integer 0..6")
    if config["truth"]["velocity_method"] not in ("step_end_difference", "central_difference"):
        raise ValueError("Unknown evaluation velocity method")


def print_report(data_info, config, detection, attitude_error, run_reports, consistency):
    print(f"数据: {data_info['samples']} 帧，t={data_info['time_s'][0]:.3f}–{data_info['time_s'][1]:.3f} s，1000 Hz；NaN/Inf=0。")
    low, high = data_info["quaternion_norm_min_max"]
    print(f"四元数范数: [{low:.10f}, {high:.10f}]；原始数据完整保留，未限幅/滤波。")
    print("测量分布（比力 m/s²、角速度 rad/s；min / P50 / P90 / P99 / max）:")
    for node in range(6):
        force = "/".join(f"{x:.2f}" for x in data_info["force_percentiles_m_s2"][node])
        gyro = "/".join(f"{x:.3f}" for x in data_info["gyro_percentiles_rad_s"][node])
        print(f"  S{node}: |f|={force}; |w|={gyro}; 超16g轴量程帧数={data_info['any_force_axis_over_16g_samples'][node]}")
    print(f"控制状态/帧数: {data_info['control_states_counts']}；切换次数={data_info['control_switches']}。")
    print("坐标/重力: R_WS 将 S→W；a_W=R_WS f_S+[0,0,-9.81]，单位 m/s²；合成静止/倾斜检查通过。")
    print("姿态: ground_truth=仿真姿态调试；estimated=陀螺积分，使用已知 R(0)=I，未进行六轴融合；Yaw无绝对观测。")
    print("陀螺姿态相对真值 RMSE/末帧角误差(deg): " + "; ".join(
        f"S{node}={np.sqrt(np.mean(attitude_error[:, node]**2)):.3f}/{attitude_error[-1, node]:.3f}" for node in range(6)))
    velocity = config["initial_velocity_m_s"]
    epoch = "t=0 (仿真开始)" if config["initial_velocity_epoch"] == "simulation_start" else "t=0.001 s (首个记录时刻)"
    print(f"积分: {config['integration']}；{epoch} 的初速假设={velocity} m/s，未经静止确认。")
    if config["initial_velocity_epoch"] == "simulation_start":
        print("首段积分: v(t=0.001)=v(t=0)+a_W[0]×0.001；位置输出相对首个记录时刻。")
    print("真值映射: " + config["truth"]["mapping_source"])
    print("评估仅用真值首帧位置作平移对齐；无轨迹拟合、无真值反馈。速度真值由端点差分得到，不作为测量输入。")
    print("误差（各项为六节点顺序 S0…S5；位置RMSE/末帧误差 m，速度RMSE m/s）:")
    for name, report in run_reports.items():
        metrics = report["metrics"]
        for label, key in (("p_RMSE", "position_rmse_m"), ("p_final", "final_position_error_m"), ("v_RMSE", "velocity_rmse_m_s")):
            values = " ".join("N/A" if entry is None else f"{entry[key]:.4f}" for entry in metrics)
            print(f"  {name:22s} {label:8s}: {values}")
    windows = [runs(detection["stationary"][:, node]) for node in range(6)]
    print(f"静止候选: detector={config['zupt']['detector']}；|w|≤{config['zupt']['gyro_threshold_rad_s']} rad/s，"
          f"||f|-g|≤{config['zupt']['force_norm_tolerance_m_s2']} m/s²，连续≥{config['zupt']['minimum_duration_s']*1000:g} ms。")
    print("  各节点窗口数=" + str([len(w) for w in windows]) +
          "；首帧起始静止窗口=" + str(detection["stationary"][0].tolist()) +
          f"；驱动断电事件={len(detection['events'])} 个时刻。")
    if not np.any(detection["stationary"]):
        print("  当前没有可信静止窗口；offline 不施加任何校正，因此与原始INS一致。驱动断电不等于零速。")
    else:
        print("  以上仅为IMU静止候选，匀速平移仍无法由这些条件排除；只校正有起止静止锚点的运动段。")
    known_consistency = [entry["step_end_acceleration_residual_rmse_m_s2"] for entry in consistency if entry is not None]
    if known_consistency:
        print("步末差分一致性（仅独立诊断）a残差RMSE m/s²: " + " ".join(f"{x:.4g}" for x in known_consistency))
        if any(x > .01 for x in known_consistency):
            print("  节点1–5存在超出坐标舍入量级的差分残差；尚不能归因于噪声或采样率，未修改数据。")
    print("主要问题: 初速假设未经确认；冲击峰值大、积分对步末采样/时间对齐敏感；无可靠ZUPT锚点，姿态误差可泄漏重力。")
    print("下一步: 导出 body 世界速度与t=0完整状态核验离散时序；采集真实停顿；核验传感器量程/噪声后再试六轴AHRS。")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    args = arguments()
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if args.integration:
        config["integration"] = args.integration
    if args.detector:
        config["zupt"]["detector"] = args.detector
    if args.initial_velocity is not None:
        config["initial_velocity_m_s"] = args.initial_velocity
    if args.initial_epoch:
        config["initial_velocity_epoch"] = args.initial_epoch
    validate_config(config)
    if args.task == "robot":
        from ins.robot_pipeline import run_robot
        run_robot(args, config)
        return
    data = load_data(args.data, config["sample_rate_hz"])
    data_info = describe(data, config["gravity_m_s2"])
    # Both orientation pipelines can run WITHOUT passing any endpoint positions.
    rotations_gt = truth_rotations(data.quaternion)
    rotations_est = gyro_rotations(data.gyroscope, data.time, config["integration"])
    attitude_error = attitude_error_degrees(rotations_est, rotations_gt)
    detection = detect_stationary(data.specific_force, data.gyroscope, data.time,
                                  data.controls, config["zupt"], config["gravity_m_s2"])
    # Gravity sign invariant in both aligned and arbitrarily tilted static poses.
    tilted = rotations_gt[:1]
    static_force = np.einsum("...ji,j->...i", tilted, np.array([0., 0., config["gravity_m_s2"]]))
    if not np.allclose(world_acceleration(tilted, static_force, config["gravity_m_s2"]), 0, atol=1e-10):
        raise RuntimeError("Gravity compensation invariant failed")
    reference = reference_positions(data.endpoints, config["truth"]["endpoint_indices"])
    consistency = sampling_consistency(world_acceleration(rotations_gt, data.specific_force),
                                       data.endpoints, config["truth"]["endpoint_indices"], data.time)
    args.output.mkdir(parents=True, exist_ok=True)
    plot_common(data, detection, rotations_gt, rotations_est, args.output)
    modes = ["ground_truth", "estimated"] if args.mode == "all" else [args.mode]
    variants = ["none", "offline"] if args.zupt == "both" else [args.zupt]
    run_reports, positions = {}, {}
    for mode in modes:
        rotation = rotations_gt if mode == "ground_truth" else rotations_est
        acceleration = world_acceleration(rotation, data.specific_force, config["gravity_m_s2"])
        raw_velocity = integrate_acceleration(acceleration, data.time, config["initial_velocity_m_s"],
                                              config["integration"], config["initial_velocity_epoch"])
        raw_position = integrate_velocity(raw_velocity, data.time, config["integration"])
        for variant in variants:
            name = f"{mode}_{variant}"
            output = args.output / name
            output.mkdir(exist_ok=True)
            velocity, counts = (correct_velocity(raw_velocity, data.time, detection["stationary"])
                                if variant == "offline" else (raw_velocity.copy(), [0] * 6))
            position = integrate_velocity(velocity, data.time, config["integration"])
            if not all(np.isfinite(array).all() for array in (acceleration, velocity, position)):
                raise RuntimeError(f"{name}: nonfinite navigation output")
            metrics = evaluate(position, velocity, reference, data.time, config["truth"]["velocity_method"])
            run_reports[name] = {"orientation_source": "simulation ground truth (debug only)" if mode == "ground_truth" else "gyro only; known initial frame",
                                 "metrics": metrics, "corrected_bounded_motion_segments": counts,
                                 "final_displacement_W_m": position[-1].tolist()}
            positions[name] = position
            np.savez_compressed(output / "trajectory.npz", time_s=data.time,
                                position_relative_W_m=position, velocity_W_m_s=velocity,
                                acceleration_W_m_s2=acceleration, stationary=detection["stationary"],
                                orientation_source=np.array(mode), integration=np.array(config["integration"]),
                                initial_velocity_epoch=np.array(config["initial_velocity_epoch"]),
                                initial_velocity_assumed_m_s=np.asarray(config["initial_velocity_m_s"]))
            plot_run(data.time, position, velocity, raw_position, reference, name, output)
    plot_comparison(data.time, positions, reference, args.output)
    summary = {"config": config, "data": data_info,
               "stationary_window_counts": [len(runs(detection["stationary"][:, node])) for node in range(6)],
               "initial_stationary": detection["stationary"][0].tolist(),
               "attitude_rmse_deg": np.sqrt(np.mean(attitude_error**2, axis=0)).tolist(),
               "sampling_consistency": consistency, "runs": run_reports}
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print_report(data_info, config, detection, attitude_error, run_reports, consistency)
    print(f"静态图表/六节点结果: {args.output.resolve()}")


if __name__ == "__main__":
    main()
