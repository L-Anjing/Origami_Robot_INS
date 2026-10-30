"""MAT v5 export, schema 2: one shared shape plus compact reference experiments."""
from pathlib import Path
import numpy as np
from scipy.io import loadmat, savemat


def export_mat(path,time,controls,truth_points,truth_quaternion,model,results,config):
    """Keep tested coordinate-first MATLAB axes and round-trip validation.

    Shape and odometry have separate fields. No global plate/end-point arrays
    are duplicated for trajectory methods. MATLAB places relative shape using
    the selected precomputed reference INS; truth is never the placement input.
    """
    modes = {}
    for name,result in results.items():
        references = {}
        for key,ref in result["references"].items():
            references[key] = {
                "raw_position_W":ref["raw_position"].T, "raw_velocity_W":ref["raw_velocity"].T,
                "raw_q_WS_wxyz":ref["raw_quaternion"].T,
                "eskf_position_W":ref["position"].transpose(2,0,1),
                "eskf_velocity_W":ref["velocity"].transpose(2,0,1),
                "eskf_q_WS_wxyz":ref["quaternion"].transpose(2,0,1),
                "gyro_bias_rad_s":ref["bg"].transpose(2,0,1),
                "accel_bias_m_s2":ref["ba"].transpose(2,0,1),
                "covariance_diagonal":ref["covariance_diagonal"].transpose(2,0,1),
                "glrt_candidate":ref["glrt_candidate"].reshape(1,-1).astype(np.uint8),
                "phase_glrt_candidate":ref["phase_candidate"].reshape(1,-1).astype(np.uint8),
                "glrt_statistic":ref["glrt_statistic"].reshape(1,-1),
                "glrt_valid":ref["glrt_valid"].reshape(1,-1).astype(np.uint8),
                "zupt_updated":ref["zupt_updated"].T.astype(np.uint8),
                "zupt_requested":ref["zupt_requested"].T.astype(np.uint8),
                "nis_rejected":ref["nis_rejected"].T.astype(np.uint8),
                "nis":ref["nis"].T,
                "quality":{k:v.reshape(1,-1).astype(np.uint8) for k,v in ref["quality"].items()}}
        modes[name] = {
            "q_WS_wxyz":result["quaternion"].transpose(2,1,0),
            "R_WS":result["rotation"].transpose(2,3,1,0),
            "relative_markers_W":result["shape"].transpose(2,1,0),
            "relative_nominal_W":result["nominal_shape"].transpose(2,1,0),
            "joint_axis_misalignment_deg":result["joint_axis_misalignment_deg"].T,
            "accel_feedback_used":result["feedback"].T.astype(np.uint8),
            "orientation_source":result["orientation_source"], "references":references}
    phase = next(iter(results.values()))["phase"]
    phase_payload = {key:(value.T if value.ndim==2 else value.reshape(1,-1)).astype(np.uint8)
                     for key,value in phase.items()}
    indices = sorted(int(key[1:]) for key in next(iter(results.values()))["references"])
    payload = {
        "schema_version":np.array([[2]],dtype=np.int32),
        "time_s":time.reshape(1,-1), "controls":controls.T.astype(np.uint8),
        "gravity_W_m_s2":np.array([[0.],[0.],[-9.81]]),
        "truth_endpoints_W":truth_points.transpose(2,1,0),
        "truth_endpoint_index_zero_based":np.array([-1 if i is None else i for i in config["truth"]["endpoint_indices"]],dtype=np.int16).reshape(1,6),
        "reference_indices_zero_based":np.array(indices,dtype=np.int16).reshape(1,-1),
        "default_reference_index":np.array([[config["reference_imu_index"]]],dtype=np.int16),
        "policy_names":np.array(["none","glrt","phase"],dtype=object).reshape(1,3),
        "default_policy_one_based":np.array([[config["robot"]["zupt_modes"].index(config["robot"]["default_zupt_mode"])+1]]),
        "plate_vertices_S_m":model["plate_vertices"].transpose(2,1,0),
        "plate_faces_one_based":np.array([[1,2,3,4],[5,6,7,8],[1,2,6,5],[2,3,7,6],[3,4,8,7],[4,1,5,8]]),
        "joint_offsets_S_m":model["joint_offsets"].T,
        "marker_link_offsets_S_m":model["marker_link_offsets"].T,
        "physical_tip_offset_S_m":model["physical_tip_offset"].reshape(3,1),
        "imu_body_offset_S_m":model["imu_body_offset_S"].T,
        "crack_centers_W_m":np.stack([c["center"] for c in model["cracks"]]).T,
        "crack_half_sizes_m":np.stack([c["half_size"] for c in model["cracks"]]).T,
        "initial_root_W_m":np.asarray(config["robot"]["initial_root_W_m"]).reshape(3,1),
        "initialization_source":"documented t=0 aligned geometry/position; assumed velocity/bias; no truth trajectory correction",
        "geometry_note":"50mm solids; 50.1mm connection/log markers; shape only, never navigation constraints",
        "covariance_state_order":"dp,dv,dtheta_local,dbg,dba",
        "phase":phase_payload, "modes":modes}
    path = Path(path)
    savemat(path,payload,do_compression=True,long_field_names=True,oned_as="row")
    loaded = loadmat(path,struct_as_record=False,squeeze_me=False)
    n = len(time)
    if loaded["time_s"].shape != (1,n) or not np.array_equal(loaded["time_s"].ravel(),time):
        raise RuntimeError("MAT time round-trip mismatch")
    for name,result in results.items():
        mode = getattr(loaded["modes"][0,0],name)[0,0]
        for field,shape in {"R_WS":(3,3,6,n),"q_WS_wxyz":(4,6,n),"relative_markers_W":(3,7,n)}.items():
            value = getattr(mode,field)
            if value.shape!=shape or not np.isfinite(value).all():
                raise RuntimeError(f"MAT shape/dtype mismatch: {name}.{field}")
        if not np.array_equal(mode.R_WS,result["rotation"].transpose(2,3,1,0)):
            raise RuntimeError("MAT R_WS ordering mismatch")
        for key,ref in result["references"].items():
            value = getattr(mode.references[0,0],key)[0,0]
            for field,shape in {"raw_position_W":(3,n),"eskf_position_W":(3,n,3),
                                "eskf_q_WS_wxyz":(4,n,3),"covariance_diagonal":(15,n,3),
                                "zupt_updated":(3,n)}.items():
                array = getattr(value,field)
                if array.shape!=shape or not np.isfinite(array).all():
                    raise RuntimeError(f"MAT invalid reference field {name}.{key}.{field}")
            if not np.array_equal(value.eskf_position_W,ref["position"].transpose(2,0,1)):
                raise RuntimeError("MAT reference/policy axis mismatch")
    return path.resolve()
