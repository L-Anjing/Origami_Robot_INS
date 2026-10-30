"""Static diagnostics for independent shape and single-reference odometry."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from .visualize import save


def plot_robot_results(data,model,results,output,config):
    index=config["reference_imu_index"]
    key=f"S{index}"
    policy=config["robot"]["zupt_modes"].index(config["robot"]["default_zupt_mode"])
    chosen=results.get("estimated",next(iter(results.values())))
    ref=chosen["references"][key]
    fig,axes=plt.subplots(2,1,figsize=(12,6),sharex=True)
    axes[0].plot(data.time,np.linalg.norm(data.specific_force[:,index],axis=1),lw=.6)
    axes[1].plot(data.time,np.linalg.norm(data.gyroscope[:,index],axis=1),lw=.6)
    axes[0].set_ylabel("|specific force| (m/s²)")
    axes[1].set(xlabel="Time (s)",ylabel="|angular rate| (rad/s)")
    fig.suptitle(f"{key}: unmodified simulation input; hardware range risk is flagged, never clipped")
    save(fig,output/"measurements.png")

    fig,axes=plt.subplots(len(results),3,figsize=(14,4*len(results)),squeeze=False)
    truth_index=config["truth"]["endpoint_indices"][index]
    for row,(name,result) in enumerate(results.items()):
        nav=result["references"][key]
        for component,label in enumerate("XYZ"):
            ax=axes[row,component]
            ax.plot(data.time,nav["raw_position"][:,component],lw=.8,label="Raw INS")
            ax.plot(data.time,nav["position"][:,policy,component],lw=.8,ls="--",label="Online ESKF + selected ZUPT")
            if truth_index is not None:
                ax.plot(data.time,data.endpoints[:,truth_index,component],"k:",lw=.8,label=f"Verified truth V{truth_index}")
            ax.set(xlabel="Time (s)",ylabel=f"{label}_W (m)",title=f"{name} / {key}")
            ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.suptitle("Reference IMU translation only; no multi-IMU position constraints")
    save(fig,output/"reference_trajectories.png")

    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True)
    for drive in range(3):
        axes[0].step(data.time,data.controls[:,drive]+1.3*drive,where="post",label=f"u{drive}")
    axes[0].legend(ncol=3)
    axes[0].set_ylabel("Drives (offset)")
    axes[1].step(data.time,ref["glrt_candidate"].astype(int),where="post",label="GLRT candidate")
    axes[1].step(data.time,ref["phase_candidate"].astype(int)*.8,where="post",label="Phase + GLRT candidate")
    axes[1].plot(data.time,ref["zupt_updated"][:,policy].astype(int)*.6,label="Actual Kalman update")
    axes[1].set_ylabel("Stationary/update")
    axes[1].legend(ncol=3,fontsize=8)
    for field,label in (("impact","Strong impact"),("accel_over_range","Accel range risk"),("gyro_over_range","Gyro range risk")):
        axes[2].step(data.time,ref["quality"][field].astype(int),where="post",lw=.6,label=label)
    axes[2].set(xlabel="Time (s)",ylabel="Quality flags")
    axes[2].legend(ncol=3,fontsize=8)
    fig.suptitle(f"{key}: causal detection; drive endings alone cannot trigger ZUPT")
    save(fig,output/"stationary_and_phase.png")

    fig,axes=plt.subplots(1,3,figsize=(14,4))
    for ax,k in zip(axes,[0,len(data.time)//2,len(data.time)-1]):
        truth=data.endpoints[k]-data.endpoints[k,0]
        ax.plot(truth[:,0],truth[:,1],"ko-",ms=3,label="Relative truth")
        for name,result in results.items():
            shape=result["shape"][k]
            ax.plot(shape[:,0],shape[:,1],".--",label=name)
        ax.set(title=f"t={data.time[k]:.3f} s",xlabel="Relative X_W (m)",ylabel="Relative Y_W (m)")
        ax.set_aspect("equal",adjustable="datalim")
        ax.grid(alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle("Six independent plate attitudes: relative shape only, no INS feedback")
    save(fig,output/"shape_snapshots.png")
