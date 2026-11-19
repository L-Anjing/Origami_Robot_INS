#include "multi_imu_eskf.h"
#include "simulation_data.h"
#include <fstream>
#include <iostream>
#include <chrono>
using namespace origami;
using Clock = std::chrono::steady_clock;
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const std::string root=argv[1];
    Json base; std::ifstream(root+"/config/multi_imu_eskf.json") >> base;
    auto model=load_robot_model(root+"/robot/zbot_crack_L2.xml",base["robot"]["geometry"]);
    const double dt=1./base["sample_rate_hz"].get<double>();
    auto data=read_simulation(root+"/data/data_L_qawp_20261030.txt",1./dt);
    std::ofstream out(argv[2]);
    out << "seed,imus,compact,dimension,samples,duration_s,prop_us,update_us,amortized_us,p95_step_us,p99_step_us,position_rmse_m,attitude_rmse_deg,final_error_m,layout_position_delta_m,layout_cov_max_delta\n";
    for(int seed=42;seed<52;++seed) {
        auto c=base; c["noise"]["seed"]=seed;
        ImuNoise noise(c["noise"],"full");
        std::vector<ImuMeasurements> input;
        for(auto& frame:data) input.push_back(noise.apply(frame.imu,dt).measured);
        for(int n=1;n<=6;++n) {
            c["joint_filter"]["imu_count"]=n;
            std::vector<V3D> reference; std::vector<Eigen::MatrixXd> cov;
            for(int compact=0;compact<2;++compact) {
                c["joint_filter"]["compact_prefix_state"]=bool(compact);
                // Warm up the same propagation/update path outside the measured run.
                { MultiImuEskf warm(c,model); for(int k=0;k<100;++k) {warm.propagate(input[k],dt); if((k+1)%10==0) warm.update(input[k].time);} }
                MultiImuEskf f(c,model);
                double p2=0,q2=0,dp=0,dc=0; std::vector<double> times;
                for(size_t k=0;k<data.size();++k) {
                    auto tic=Clock::now();
                    f.propagate(input[k],dt);
                    if((k+1)%10==0) f.update(input[k].time);
                    times.push_back(std::chrono::duration<double,std::micro>(Clock::now()-tic).count());
                    p2+=(f.nodes[0].p-data[k].truth.markers[0]).squaredNorm();
                    double angle=f.nodes[0].attitude.q.angularDistance(data[k].truth.orientation[0]);q2+=angle*angle;
                    // Compare complete selected state and covariance at update epochs.
                    if((k+1)%10==0) {
                        if(!compact) {for(int i=0;i<n;++i) reference.push_back(f.nodes[i].p); cov.push_back(f.P.topLeftCorner(15*n,15*n));}
                        else {size_t epoch=(k+1)/10-1; for(int i=0;i<n;++i) dp=std::max(dp,(f.nodes[i].p-reference[epoch*n+i]).norm()); dc=std::max(dc,(f.P-cov[epoch]).cwiseAbs().maxCoeff());}
                    }
                }
                f.diagnostics();
                std::sort(times.begin(),times.end());
                double prop=1e6*f.runtime.propagation_s/data.size();
                double update=1e6*f.runtime.total_update_s/f.runtime.update_count;
                out<<seed<<','<<n<<','<<compact<<','<<f.P.rows()<<','<<data.size()<<','<<data.size()*dt<<','<<prop<<','<<update<<','<<prop+update*f.runtime.update_count/data.size()<<','<<times[size_t(.95*(times.size()-1))]<<','<<times[size_t(.99*(times.size()-1))]<<','<<sqrt(p2/data.size())<<','<<sqrt(q2/data.size())*180/pi<<','<<(f.nodes[0].p-data.back().truth.markers[0]).norm()<<','<<dp<<','<<dc<<'\n';
                out.flush();
            }
        }
        std::cerr << "Completed seed " << seed << '\n';
    }
}
