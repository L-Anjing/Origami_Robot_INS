"""Two independent chains: six-AHRS relative shape and single-reference ESKF."""
import json
import numpy as np

from .attitude_estimator import SixAxisAHRS, quaternion_matrix
from .core import INS
from .data_loader import describe, load_data
from .export_mat import export_mat
from .evaluate import sampling_consistency
from .kinematics import forward_chain
from .model_loader import load_robot_model
from .orientation import attitude_error_degrees
from .quality import measurement_flags
from .stationary import DrivePhaseTracker, StationaryDetector

ZUPT_MODES = ("none", "glrt", "phase")


def validate_robot_config(config):
    r = config["robot"]
    if config["integration"] != "step_end" or config["initial_velocity_epoch"] != "simulation_start":
        raise ValueError("Online ESKF uses documented step-end samples and t=0 initialization; legacy experiments: --task baseline")
    index = config["reference_imu_index"]
    indices = [index] + config["precompute_reference_indices"]
    if any(type(i) is not int or not 0 <= i < 6 for i in indices):
        raise ValueError("Reference IMU indices must be integers 0..5")
    if np.asarray(r["initial_root_W_m"]).shape != (3,) or not np.isfinite(r["initial_root_W_m"]).all():
        raise ValueError("Expected finite documented t=0 root position")
    if r["zupt_modes"] != list(ZUPT_MODES) or r["default_zupt_mode"] not in ZUPT_MODES:
        raise ValueError("Export requires none/glrt/phase comparison modes")
    positive_groups = {
        "ahrs": ("gain_s_inv","force_tolerance_m_s2","innovation_limit_deg","minimum_reliable_s"),
        "stationary": ("window_s","minimum_stationary_s","sigma_accel_m_s2","sigma_gyro_rad_s","glrt_threshold","gyro_limit_rad_s","force_tolerance_m_s2"),
        "quality": ("accelerometer_range_g","gyroscope_range_deg_s","impact_force_norm_g","impact_noise_multiplier"),
        "eskf": ("zupt_velocity_std_m_s","zupt_nis_limit","zupt_min_interval_s","covariance_negative_tolerance")}
    for group, keys in positive_groups.items():
        for key in keys:
            if not np.isfinite(r[group][key]) or r[group][key] <= 0:
                raise ValueError(f"robot.{group}.{key} must be positive and finite")
    for group in ("noise_density","initial_std"):
        for value in r["eskf"][group].values():
            if not np.isfinite(value) or value <= 0:
                raise ValueError("ESKF noise/prior scales must be positive, adjustable assumptions")
    for key in ("initial_gyro_bias_rad_s","initial_accel_bias_m_s2"):
        if np.asarray(r["eskf"][key]).shape != (3,) or not np.isfinite(r["eskf"][key]).all():
            raise ValueError("ESKF initial bias must be a finite 3-vector")
    p = r["phase"]
    if (not np.isfinite([p["post_falling_delay_s"],p["post_falling_window_s"],p["stable_off_min_s"]]).all() or
            not p["relevant_drives"] or any(type(i) is not int or i not in range(3) for i in p["relevant_drives"]) or
            not 0 <= p["post_falling_delay_s"] < p["post_falling_window_s"] or p["stable_off_min_s"] < 0):
        raise ValueError("Invalid independent-drive phase windows")
    if r["ahrs"]["innovation_limit_deg"] > 90:
        raise ValueError("Gravity feedback innovation limit must be at most 90 degrees")
    tolerance = config["truth"]["acceleration_consistency_warning_m_s2"]
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Truth acceleration consistency warning tolerance must be positive")
    if type(r["eskf"]["covariance_check_every"]) is not int or r["eskf"]["covariance_check_every"] < 1:
        raise ValueError("covariance_check_every must be a positive integer")


def phase_history(time, controls, config):
    tracker = DrivePhaseTracker(config)
    states = [tracker.update(controls[k], t, t if k == 0 else t-time[k-1]) for k,t in enumerate(time)]
    return {key: np.asarray([s[key] for s in states]) for key in states[0]}


def reference_navigation(time, force, gyro, flags, phase, initial_position, initial_velocity,
                         config, quaternion_debug=None, backend="six_axis"):
    """One physical reference node, compared under three candidate policies."""
    r = config["robot"]
    kwargs = dict(position=initial_position, velocity=initial_velocity, ahrs_config=r["ahrs"],
                  filter_config=r["eskf"], gravity=config["gravity_m_s2"], backend=backend,
                  prescribed_orientation=quaternion_debug is not None)
    raw = INS(**kwargs, covariance=False)
    filters = [INS(**kwargs) for _ in ZUPT_MODES]
    detector = StationaryDetector(r["stationary"], 1/config["sample_rate_hz"], config["gravity_m_s2"], nodes=1)
    n = len(time)
    out = {key: np.empty((n,3)) for key in ("raw_position","raw_velocity")}
    out["raw_quaternion"] = np.empty((n,4))
    for key in ("position","velocity","bg","ba"):
        out[key] = np.empty((n,3,3))  # time, policy, coordinate
    out["quaternion"] = np.empty((n,3,4))
    out["covariance_diagonal"] = np.empty((n,3,15))
    for key in ("zupt_updated","zupt_requested","nis_rejected"):
        out[key] = np.zeros((n,3), dtype=bool)
    out["nis"] = np.zeros((n,3))
    out["glrt_candidate"] = np.zeros(n, dtype=bool)
    out["phase_candidate"] = np.zeros(n, dtype=bool)
    out["glrt_statistic"] = np.zeros(n)
    out["glrt_valid"] = np.zeros(n, dtype=bool)
    last_attempt = np.full(3,-np.inf)
    minimum_eigenvalue = np.full(3,np.inf)
    symmetry_error = np.zeros(3)
    for k,t in enumerate(time):
        dt = t if k==0 else t-time[k-1]
        valid = bool(flags["valid_for_aiding"][k])
        debug = None if quaternion_debug is None else quaternion_debug[k]
        raw_state = raw.propagate(gyro[k],force[k],dt,valid,quaternion_debug=debug)
        out["raw_position"][k], out["raw_velocity"][k], out["raw_quaternion"][k] = raw_state["position"],raw_state["velocity"],raw_state["quaternion"]
        quiet,statistic = detector.update(force[k,None],gyro[k,None],[valid])
        out["glrt_candidate"][k] = quiet[0]
        out["phase_candidate"][k] = bool(quiet[0] and phase["phase_candidate"][k])
        out["glrt_valid"][k] = np.isfinite(statistic[0])
        out["glrt_statistic"][k] = statistic[0] if np.isfinite(statistic[0]) else 0.
        candidates = (False, bool(quiet[0]), bool(out["phase_candidate"][k]))
        for policy,filter_ in enumerate(filters):
            multiplier = 1. if valid else r["quality"]["impact_noise_multiplier"]
            filter_.propagate(gyro[k],force[k],dt,valid,multiplier,debug)
            requested = candidates[policy] and t-last_attempt[policy]+1e-10 >= r["eskf"]["zupt_min_interval_s"]
            if requested:
                last_attempt[policy] = t
            update = filter_.zupt_update(requested,valid)
            out["zupt_requested"][k,policy] = requested
            out["zupt_updated"][k,policy] = update["accepted"]
            out["nis_rejected"][k,policy] = update["rejected_nis"]
            out["nis"][k,policy] = update["nis"]
            state = filter_.snapshot()
            for key in ("position","velocity","quaternion","bg","ba"):
                out[key][k,policy] = state[key]
            out["covariance_diagonal"][k,policy] = state["covariance_diagonal"]
            if k % r["eskf"]["covariance_check_every"] == 0 or k == n-1:
                minimum_eigenvalue[policy] = min(minimum_eigenvalue[policy],float(np.linalg.eigvalsh(filter_.P)[0]))
                symmetry_error[policy] = max(symmetry_error[policy],float(np.max(np.abs(filter_.P-filter_.P.T))))
                if minimum_eigenvalue[policy] < -r["eskf"]["covariance_negative_tolerance"]:
                    raise FloatingPointError("ESKF covariance lost positive semidefiniteness")
    out["quality"] = flags
    out["covariance_min_eigenvalue"] = minimum_eigenvalue
    out["covariance_max_symmetry_error"] = symmetry_error
    return out


def estimate_mode(time, force, gyro, controls, model, config, quaternion_debug=None, backend="six_axis"):
    """No endpoint-truth input; geometry is used only for shape and t=0 placement."""
    r = config["robot"]
    flags = measurement_flags(force,gyro,r["quality"],config["gravity_m_s2"])
    phase = phase_history(time,controls,r["phase"])
    ahrs = SixAxisAHRS(r["ahrs"],config["gravity_m_s2"],"step_end",backend)
    quaternion, feedback = [], []
    for k,t in enumerate(time):
        if quaternion_debug is None:
            q = ahrs.update(gyro[k],force[k],t if k==0 else t-time[k-1],flags["valid_for_aiding"][k])
            feedback.append(ahrs.feedback_used.copy())
        else:
            q = quaternion_debug[k] / np.linalg.norm(quaternion_debug[k],axis=1,keepdims=True)
            feedback.append(np.zeros(6,dtype=bool))
        quaternion.append(q.copy())
    quaternion = np.asarray(quaternion)
    rotation = quaternion_matrix(quaternion)
    shape = forward_chain(rotation,model["marker_link_offsets"])
    initial_shape = forward_chain(np.tile(np.eye(3),(6,1,1)),model["marker_link_offsets"])
    indices = sorted(set([config["reference_imu_index"]] + config["precompute_reference_indices"]))
    references = {}
    for index in indices:
        initial = np.asarray(r["initial_root_W_m"]) + initial_shape[index]
        velocity = np.asarray(config["initial_velocity_m_s"])
        if velocity.ndim == 2: velocity = velocity[index]
        references[f"S{index}"] = reference_navigation(
            time,force[:,index],gyro[:,index],{key:value[:,index] for key,value in flags.items()},
            phase,initial,velocity,config,
            None if quaternion_debug is None else quaternion_debug[:,index],backend)
    z = rotation[:,:,:,2]
    misalignment = np.degrees(np.arccos(np.clip(np.abs(np.sum(z[:,:-1]*z[:,1:],axis=2)),0,1)))
    return {"quaternion":quaternion, "rotation":rotation, "shape":shape,
            "nominal_shape":forward_chain(rotation,model["nominal_link_offsets"]),
            "feedback":np.asarray(feedback), "joint_axis_misalignment_deg":misalignment,
            "references":references, "phase":phase,
            "orientation_source":"simulation attitude, debug only" if quaternion_debug is not None else f"{backend} six-IMU AHRS; no truth correction"}


def position_metrics(estimate,truth):
    norm = np.linalg.norm(estimate-truth,axis=-1)
    return {"rmse_m":np.sqrt(np.mean(norm**2,axis=0)).tolist(), "final_error_m":norm[-1].tolist()}


def evaluate_mode(result,data,model,config):
    rotation_gt = quaternion_matrix(data.quaternion)
    acceleration_gt = np.einsum("...ij,...j->...i",rotation_gt,data.specific_force) + [0,0,-9.81]
    consistency = sampling_consistency(acceleration_gt,data.endpoints,config["truth"]["endpoint_indices"],data.time)
    angle = attitude_error_degrees(result["rotation"],rotation_gt)
    relative_truth = data.endpoints-data.endpoints[:,:1]
    reconstructed = np.einsum("ntij,tkj->ntki",result["rotation"],model["plate_vertices"])+result["shape"][:,:6,None,:]
    truth_vertices = np.einsum("ntij,tkj->ntki",rotation_gt,model["plate_vertices"])+relative_truth[:,:6,None,:]
    plate_rmse = np.sqrt(np.mean(np.sum((reconstructed-truth_vertices)**2,axis=3),axis=(0,2)))
    report = {"attitude_rmse_deg":np.sqrt(np.mean(angle**2,axis=0)).tolist(),
              "shape_plate_relative_rmse_m":plate_rmse.tolist(),
              "shape_marker_relative":position_metrics(result["shape"],relative_truth),
              "references":{}}
    for key,ref in result["references"].items():
        index = int(key[1:])
        endpoint = config["truth"]["endpoint_indices"][index]
        metrics = None if endpoint is None else {
            "raw":position_metrics(ref["raw_position"],data.endpoints[:,endpoint]),
            "eskf":position_metrics(ref["position"],data.endpoints[:,None,endpoint])}
        report["references"][key] = {"position_errors":metrics,
            "truth_endpoint":endpoint, "glrt_candidate_frames":int(ref["glrt_candidate"].sum()),
            "phase_glrt_candidate_frames":int(ref["phase_candidate"].sum()),
            "zupt_update_frames":ref["zupt_updated"].sum(axis=0).tolist(),
            "nis_rejected_frames":ref["nis_rejected"].sum(axis=0).tolist(),
            "accel_saturation_risk_samples":int(ref["quality"]["accel_over_range"].sum()),
            "gyro_saturation_risk_samples":int(ref["quality"]["gyro_over_range"].sum()),
            "impact_samples":int(ref["quality"]["impact"].sum()),
            "truth_acceleration_consistency":consistency[index],
            "force_percentiles_m_s2":np.percentile(np.linalg.norm(data.specific_force[:,index],axis=1),[0,50,90,99,100]).tolist(),
            "gyro_percentiles_rad_s":np.percentile(np.linalg.norm(data.gyroscope[:,index],axis=1),[0,50,90,99,100]).tolist(),
            "covariance_min_eigenvalue":ref["covariance_min_eigenvalue"].tolist(),
            "covariance_max_symmetry_error":ref["covariance_max_symmetry_error"].tolist()}
    return report


def run_robot(args,config):
    if args.reference_imu is not None: config["reference_imu_index"] = args.reference_imu
    if args.precompute_references is not None: config["precompute_reference_indices"] = args.precompute_references
    validate_robot_config(config)
    data = load_data(args.data,config["sample_rate_hz"])
    model = load_robot_model(args.model,config["robot"]["geometry"])
    results,reports = {},{}
    for name in (("ground_truth","estimated") if args.mode=="all" else (args.mode,)):
        results[name] = estimate_mode(data.time,data.specific_force,data.gyroscope,data.controls,model,config,
                                      data.quaternion if name=="ground_truth" else None,args.attitude)
        reports[name] = evaluate_mode(results[name],data,model,config)
    args.output.mkdir(parents=True,exist_ok=True)
    path = export_mat(args.output/"robot_odometry.mat",data.time,data.controls,data.endpoints,data.quaternion,model,results,config)
    summary = {"config":config,"data":describe(data),"modes":reports,"mat_file":str(path),"schema_version":2}
    (args.output/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    if not args.no_plots:
        from .robot_visualize import plot_robot_results
        plot_robot_results(data,model,results,args.output,config)
    fmt = lambda values: " ".join(f"{v:.5g}" for v in values)
    print(f"数据: {len(data.time)}帧，{data.time[0]:.3f}–{data.time[-1]:.3f}s；1000Hz；W: +Z向上，g=[0,0,-9.81]。")
    for name,report in reports.items():
        phase = results[name]["phase"]
        print(f"{name}: 六IMU姿态RMSE(deg): {fmt(report['attitude_rmse_deg'])}")
        print(f"  六板相对形状RMSE(m): {fmt(report['shape_plate_relative_rmse_m'])}")
        for key,ref in report["references"].items():
            error = ref["position_errors"]
            if error is None:
                print(f"  {key}: IMU真值对应未确认，位置误差N/A")
            else:
                print(f"  {key} Raw位置RMSE/末帧误差(m): {error['raw']['rmse_m']:.5g}/{error['raw']['final_error_m']:.5g}")
                print(f"  {key} ESKF位置RMSE/末帧误差(m) [none,glrt,phase]: {fmt(error['eskf']['rmse_m'])} / {fmt(error['eskf']['final_error_m'])}")
            print(f"  {key} GLRT/Phase候选帧={ref['glrt_candidate_frames']}/{ref['phase_glrt_candidate_frames']}；实际ZUPT帧={ref['zupt_update_frames']}；饱和风险 accel/gyro={ref['accel_saturation_risk_samples']}/{ref['gyro_saturation_risk_samples']}。")
            print(f"  {key} |f|与|w| min/P50/P90/P99/max: {fmt(ref['force_percentiles_m_s2'])} m/s²；{fmt(ref['gyro_percentiles_rad_s'])} rad/s。")
            diagnostic = ref["truth_acceleration_consistency"]
            if diagnostic is not None and diagnostic["step_end_acceleration_residual_rmse_m_s2"] > config["truth"]["acceleration_consistency_warning_m_s2"]:
                print(f"  {key}: 比力与已确认端点二阶差分残差超出舍入量级，需核验原点/速度定义或模型版本；未拟合真值修正。")
        print(f"  驱动结束事件(逐路)={phase['falling'].sum(axis=0).tolist()}；多路同时切换时刻={int(phase['multi_transition'].sum())}。")
    print("噪声、量程、初速和检测参数均为可调假设，未标定；原始输入未裁剪。ZUPT=0时仅传播，不宣称抑制漂移。")
    print(f"MAT: {path}；schema 2维度/时间往返检查通过；MATLAB按此前要求未启动。")
