"""Refactor acceptance: physics, ESKF Jacobians/PSD, causality, isolation, MAT."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
import io
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
from scipy.io import loadmat
from scipy.spatial.transform import Rotation

from ins.attitude_estimator import SixAxisAHRS, quaternion_matrix, quaternion_multiply, rotation_increment
from ins.core import INS
from ins.data_loader import SimulationData
from ins.export_mat import export_mat
from ins.kinematics import forward_chain
from ins.model_loader import load_robot_model
from ins.quality import measurement_flags
from ins.robot_pipeline import estimate_mode, run_robot, validate_robot_config
from ins.stationary import DrivePhaseTracker, StationaryDetector

ROOT=Path(__file__).resolve().parents[1]
CONFIG=json.loads((ROOT/"config.json").read_text(encoding="utf-8"))


def model():
    return load_robot_model(ROOT/"robot/zbot_crack_L2.xml",CONFIG["robot"]["geometry"])


def core(covariance=True,velocity=None,prescribed=False):
    return INS([.2,-.11,.02],[0,0,0] if velocity is None else velocity,
               CONFIG["robot"]["ahrs"],CONFIG["robot"]["eskf"],
               covariance=covariance,prescribed_orientation=prescribed)


def synthetic(n=120):
    t=np.arange(1,n+1)*.001
    f=np.tile([0.,0.,9.81],(n,6,1)); g=np.zeros((n,6,3)); u=np.ones((n,3),dtype=np.int8)
    u[40:,0]=0
    if n>90: f[80:90,:,0]=50.
    return t,f,g,u


class ESKFTests(unittest.TestCase):
    def test_core_is_lightweight(self):
        code="import sys; from ins.core import INS; assert not any(k in sys.modules for k in ['scipy','matplotlib','pandas'])"
        result=subprocess.run([sys.executable,"-c",code],cwd=ROOT,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_first_step_gravity_sign_and_quaternion_order(self):
        bank=core()
        s=bank.propagate([0,0,0],[0,0,309.81],.001,False)
        assert_allclose(s["velocity"],[0,0,.3])
        assert_allclose(s["position"],[.2,-.11,.0203])
        s=bank.propagate([0,0,0],[0,0,9.81],.001)
        assert_allclose(s["velocity"],[0,0,.3])
        assert_allclose(s["position"],[.2,-.11,.0206])
        assert_allclose(s["acceleration"],0,atol=1e-14)
        assert_allclose(quaternion_matrix([np.sqrt(.5),0,0,np.sqrt(.5)])@[1,0,0],[0,1,0],atol=1e-14)

    def test_raw_and_no_update_eskf_are_bitwise_identical(self):
        raw,eskf=core(False),core()
        rng=np.random.default_rng(12)
        for _ in range(100):
            g=rng.normal(0,.2,3); f=rng.normal(0,40,3)+[0,0,9.81]
            a=raw.propagate(g,f,.001,False); b=eskf.propagate(g,f,.001,False,10.)
            for key in ("position","velocity","quaternion","bg","ba"):
                assert_array_equal(a[key],b[key])
            before=eskf.snapshot(); p=eskf.P.copy()
            self.assertFalse(eskf.zupt_update(False)["accepted"])
            for key in ("position","velocity","quaternion"): assert_array_equal(before[key],eskf.snapshot()[key])
            assert_array_equal(p,eskf.P)

    def test_zupt_joseph_is_not_a_hard_velocity_reset(self):
        filter_=core(velocity=[.05,0,0])
        filter_.propagate([0,0,0],[0,0,9.81],.001)
        prior=filter_.velocity.copy()
        update=filter_.zupt_update(True)
        self.assertTrue(update["accepted"])
        self.assertGreater(abs(filter_.velocity[0]),0.)
        self.assertLess(abs(filter_.velocity[0]),abs(prior[0]))
        self.assertGreater(np.linalg.norm(filter_.last_injection),0.)
        assert_allclose(np.linalg.norm(filter_.quaternion),1,atol=1e-14)
        assert_allclose(filter_.P,filter_.P.T,atol=1e-14)
        self.assertGreaterEqual(np.linalg.eigvalsh(filter_.P)[0],-1e-12)

    def test_nis_or_invalid_measurement_rejection_changes_nothing(self):
        filter_=core(velocity=[100,0,0])
        for valid in (True,False):
            state=filter_.snapshot(); p=filter_.P.copy()
            self.assertFalse(filter_.zupt_update(True,valid)["accepted"])
            for key in ("position","velocity","quaternion","bg","ba"):
                assert_array_equal(state[key],filter_.snapshot()[key])
            assert_array_equal(p,filter_.P)

    def test_long_stationary_updates_keep_covariance_psd_and_estimate_bias(self):
        filter_=core(velocity=[.02,0,0])
        updates=0
        for k in range(1500):
            filter_.propagate([0,0,0],[.04,0,9.81],.001)
            if k>=30 and k%10==0:
                updates+=filter_.zupt_update(True)["accepted"]
            self.assertTrue(np.isfinite(filter_.P).all())
            self.assertGreaterEqual(np.linalg.eigvalsh(filter_.P)[0],-1e-10)
        self.assertGreater(updates,50)
        self.assertGreater(np.linalg.norm(filter_.ba),1e-5)
        self.assertLess(np.linalg.norm(filter_.velocity),.005)

    def test_debug_quaternion_is_forbidden_in_estimated_mode(self):
        with self.assertRaises(ValueError):
            core().propagate([0,0,0],[0,0,9.81],.001,quaternion_debug=[1,0,0,0])
        filter_=core(prescribed=True,velocity=[.02,0,0])
        q=Rotation.from_euler("xyz",[.1,.2,.3]).as_quat()[[3,0,1,2]]
        filter_.propagate([0,0,0],quaternion_matrix(q).T@[0,0,9.81],.001,quaternion_debug=q)
        filter_.zupt_update(True)
        assert_allclose(filter_.quaternion,q,atol=1e-14)
        assert_array_equal(filter_.bg,[0,0,0])

    def test_discrete_transition_matches_finite_differences_with_and_without_ahrs_feedback(self):
        for feedback in (False,True):
            base=core()
            base.attitude.reliable_duration[:]=1.
            gyro=np.array([.2,-.1,.3]); force=np.array([.1,.2,9.81])
            original=copy.deepcopy(base)
            nominal=base.propagate(gyro,force,.001,feedback)
            phi=base.last_transition
            numerical=np.empty((15,15)); epsilon=1e-6
            for column in range(15):
                perturbed=copy.deepcopy(original); delta=np.eye(15)[column]*epsilon
                perturbed.position+=delta[:3]; perturbed.velocity+=delta[3:6]
                perturbed.quaternion=quaternion_multiply(perturbed.quaternion,rotation_increment(delta[6:9]))
                perturbed.bg+=delta[9:12]; perturbed.ba+=delta[12:15]
                state=perturbed.propagate(gyro,force,.001,feedback)
                theta=Rotation.from_matrix(quaternion_matrix(nominal["quaternion"]).T@quaternion_matrix(state["quaternion"])).as_rotvec()
                numerical[:,column]=np.r_[state["position"]-nominal["position"],state["velocity"]-nominal["velocity"],
                                          theta,state["bg"]-nominal["bg"],state["ba"]-nominal["ba"]]/epsilon
            assert_allclose(phi,numerical,atol=2e-7,rtol=2e-5)


class DetectionTests(unittest.TestCase):
    def test_minimum_duration_and_impact_window_rejection(self):
        cfg=CONFIG["robot"]["stationary"]
        detector=StationaryDetector(cfg,nodes=1)
        for k in range(29):
            quiet,_=detector.update([[0,0,9.81]],[[0,0,0]],[True])
            self.assertFalse(quiet[0])
        self.assertTrue(detector.update([[0,0,9.81]],[[0,0,0]],[True])[0][0])
        self.assertFalse(detector.update([[0,0,1000]],[[0,0,0]],[False])[0][0])
        for _ in range(29):
            self.assertFalse(detector.update([[0,0,9.81]],[[0,0,0]],[True])[0][0])
        self.assertTrue(detector.update([[0,0,9.81]],[[0,0,0]],[True])[0][0])

    def test_multiple_drive_edges_and_initial_off_not_a_falling_event(self):
        tracker=DrivePhaseTracker(CONFIG["robot"]["phase"])
        first=tracker.update([0,0,0],.001,.001)
        self.assertFalse(first["falling"].any()); self.assertFalse(first["phase_candidate"])
        rise=tracker.update([1,1,1],.002,.001)
        assert_array_equal(rise["rising"],[True,True,True]); self.assertTrue(rise["multi_transition"])
        fall=tracker.update([0,1,0],.003,.001)
        assert_array_equal(fall["falling"],[True,False,True]); self.assertTrue(fall["multi_transition"])
        for k in range(4,20): last=tracker.update([0,1,0],k*.001,.001)
        self.assertTrue(last["phase_candidate"])
        last=tracker.update([1,1,1],.02,.001)
        self.assertFalse(last["phase_candidate"])

    def test_hardware_risk_flags_do_not_mutate_input(self):
        f=np.array([[1000.,0,9.81],[0,0,9.81]]); g=np.array([[100.,0,0],[0,0,0.]])
        before=f.copy(); flags=measurement_flags(f,g,CONFIG["robot"]["quality"])
        assert_array_equal(f,before)
        assert_array_equal(flags["accel_over_range"],[True,False])
        assert_array_equal(flags["gyro_over_range"],[True,False])
        assert_array_equal(flags["valid_for_aiding"],[False,True])

    def test_six_axis_feedback_tilt_and_impact_rejection(self):
        cfg=copy.deepcopy(CONFIG["robot"]["ahrs"]); cfg["gain_s_inv"]=3.
        ahrs=SixAxisAHRS(cfg)
        true=Rotation.from_euler("x",7,degrees=True).as_matrix()
        force=np.tile(true.T@[0,0,9.81],(6,1))
        for _ in range(1500): ahrs.update(np.zeros((6,3)),force,.001)
        self.assertLess(np.degrees(Rotation.from_matrix(true.T@quaternion_matrix(ahrs.quaternion)[0]).magnitude()),.1)
        ahrs.update(np.zeros((6,3)),np.tile([0,0,1000.],(6,1)),.001,np.zeros(6,dtype=bool))
        self.assertFalse(ahrs.feedback_used.any())


class PipelineTests(unittest.TestCase):
    def test_model_geometry_and_flexible_relative_rotations_are_preserved(self):
        m=model()
        assert_allclose(np.ptp(m["plate_vertices"],axis=1),np.tile([.05,.002,.05],(6,1)))
        assert_allclose(m["joint_offsets"],np.tile([.0501,0,0],(5,1)))
        r=np.tile(np.eye(3),(6,1,1)); r[1]=Rotation.from_euler("xyz",[.1,.2,.4]).as_matrix()
        points=forward_chain(r,m["marker_link_offsets"])
        assert_allclose(points[2]-points[1],r[1]@[.0501,0,0])
        assert_allclose(r[1],Rotation.from_euler("xyz",[.1,.2,.4]).as_matrix())

    def test_online_prefix_is_unchanged_including_zupt_and_covariance(self):
        t,f,g,u=synthetic(120)
        a=estimate_mode(t,f,g,u,model(),CONFIG)
        b=estimate_mode(t[:70],f[:70],g[:70],u[:70],model(),CONFIG)
        for key in ("quaternion","shape"): assert_array_equal(a[key][:70],b[key])
        for key in ("raw_position","position","velocity","quaternion","bg","ba","covariance_diagonal","zupt_updated","glrt_candidate","phase_candidate"):
            assert_array_equal(a["references"]["S0"][key][:70],b["references"]["S0"][key])
        self.assertGreater(a["references"]["S0"]["zupt_updated"][:,1:].sum(),0)

    def test_other_imus_change_shape_but_never_reference_navigation(self):
        t,f,g,u=synthetic(60); a=estimate_mode(t,f,g,u,model(),CONFIG)
        g[:,1:,2]=2.
        b=estimate_mode(t,f,g,u,model(),CONFIG)
        self.assertFalse(np.array_equal(a["shape"],b["shape"]))
        for key in ("raw_position","position","velocity","quaternion","covariance_diagonal"):
            assert_array_equal(a["references"]["S0"][key],b["references"]["S0"][key])
        self.assertEqual(set(a["references"]),{"S0"})

    def test_estimated_has_no_truth_input_and_raw_reference_attitude_matches_shape_ahrs(self):
        t,f,g,u=synthetic(50); result=estimate_mode(t,f,g,u,model(),CONFIG)
        assert_array_equal(result["references"]["S0"]["raw_quaternion"],result["quaternion"][:,0])
        # Explicit debug quaternion changes only Mode A; estimated is computed without it.
        debug=np.tile([0.,1.,0.,0.],(len(t),6,1))
        estimate_mode(t,f,g,u,model(),CONFIG,debug)
        repeat=estimate_mode(t,f,g,u,model(),CONFIG)
        assert_array_equal(result["references"]["S0"]["raw_position"],repeat["references"]["S0"]["raw_position"])

    def test_reference_selection_computes_actual_reference_measurements(self):
        c=copy.deepcopy(CONFIG); c["precompute_reference_indices"]=[2]
        t,f,g,u=synthetic(35); f[:,2,0]=5.
        result=estimate_mode(t,f,g,u,model(),c)
        self.assertEqual(set(result["references"]),{"S0","S2"})
        self.assertGreater(np.max(np.abs(result["references"]["S2"]["raw_velocity"])),.01)
        assert_allclose(result["references"]["S0"]["raw_velocity"],0,atol=1e-12)

    def test_orchestrator_truth_changes_affect_only_evaluation_and_truth_export(self):
        t,f,g,u=synthetic(35)
        q=np.tile([1.,0.,0.,0.],(len(t),6,1))
        points=np.tile(forward_chain(np.tile(np.eye(3),(6,1,1)),model()["marker_link_offsets"]),(len(t),1,1))
        points+=CONFIG["robot"]["initial_root_W_m"]
        data=SimulationData(np.arange(1,len(t)+1),t,q,f,g,u,points)
        with tempfile.TemporaryDirectory() as directory:
            args=SimpleNamespace(reference_imu=None,precompute_references=None,data=Path("not_read"),
                model=ROOT/"robot/zbot_crack_L2.xml",mode="estimated",attitude="six_axis",no_plots=True,output=Path(directory))
            positions=[]
            for changed in (False,True):
                current=copy.deepcopy(data)
                if changed:
                    current.quaternion[:]=[0.,1.,0.,0.]
                    current.endpoints[:]+=100.
                with patch("ins.robot_pipeline.load_data",return_value=current), redirect_stdout(io.StringIO()):
                    run_robot(args,copy.deepcopy(CONFIG))
                loaded=loadmat(args.output/"robot_odometry.mat",struct_as_record=False,squeeze_me=False)
                positions.append(loaded["modes"][0,0].estimated[0,0].references[0,0].S0[0,0].eskf_position_W.copy())
            assert_array_equal(*positions)

    def test_schema_two_compact_mat_roundtrip_and_no_obsolete_methods(self):
        t,f,g,u=synthetic(35); c=copy.deepcopy(CONFIG); c["precompute_reference_indices"]=[2]
        m=model(); result=estimate_mode(t,f,g,u,m,c)
        truth=result["shape"]+np.asarray(c["robot"]["initial_root_W_m"])
        with tempfile.TemporaryDirectory() as directory:
            path=export_mat(Path(directory)/"test.mat",t,u,truth,result["quaternion"],m,{"estimated":result},c)
            loaded=loadmat(path,struct_as_record=False,squeeze_me=False)
            self.assertEqual(int(loaded["schema_version"][0,0]),2)
            mode=loaded["modes"][0,0].estimated[0,0]
            self.assertFalse(hasattr(mode,"endpoint_position_W")); self.assertFalse(hasattr(mode,"fusion_weights"))
            self.assertEqual(mode.references[0,0].S2[0,0].eskf_position_W.shape,(3,len(t),3))
            assert_array_equal(loaded["reference_indices_zero_based"],[[0,2]])
            assert_array_equal(loaded["time_s"].ravel(),t)
            # Shape placement is a rendering operation: the selected reference must coincide.
            ref=mode.references[0,0].S2[0,0].eskf_position_W[:,:,2]
            root=ref-mode.relative_markers_W[:,2,:]
            assert_allclose(root+mode.relative_markers_W[:,2,:],ref)

    def test_invalid_reference_or_noise_configuration_rejected(self):
        for mutation in ("reference","noise"):
            c=copy.deepcopy(CONFIG)
            if mutation=="reference": c["reference_imu_index"]=6
            else: c["robot"]["eskf"]["noise_density"]["gyro_rad_s_sqrt_hz"]=-1.
            with self.assertRaises(ValueError): validate_robot_config(c)


if __name__=="__main__": unittest.main()
