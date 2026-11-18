#include "multi_imu_eskf.h"
#include "multi_imu_fusion.h"
#include "static_calibration.h"
#include <fstream>
#include <iostream>
using namespace origami;
namespace
{
void require(bool ok, const std::string& message)
{
    if (!ok)
        throw std::runtime_error(message);
}
std::vector<SimulationFrame> dynamic_frames()
{
    std::vector<SimulationFrame> frames(20);
    for (size_t k = 0; k < frames.size(); ++k)
    {
        frames[k].imu.time = (k + 1) * .001;
        frames[k].imu.force.fill(V3D(0, 0, 9.81));
    }
    return frames;
}
} // namespace
int main(int argc, char** argv)
{
    try
    {
        if (argc != 3)
            return 2;
        Json c;
        std::ifstream(std::string(argv[1]) + "/config/multi_imu_eskf.json") >> c;
        int test = std::stoi(argv[2]);
        auto model = load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                      c["robot"]["geometry"]);
        auto initial = forward_shape(model, Quaterniond::Identity(), {});
        auto frames = dynamic_frames();
        if (test == 1 || test == 2 || test == 8)
        {
            c["noise"]["gyro_white_std_deg_s"] = 0.;
            c["noise"]["accel_white_std_m_s2"] = 0.;
            auto sequence =
                make_calibration_sequence(c, test == 1 ? "none" : "fixed_bias", frames, 500);
            auto cal = estimate_stationary_bias(sequence.stationary, initial, c, .001);
            require(cal.available, "Ideal/static calibration rejected");
            for (int i = 0; i < 6; ++i)
            {
                require((cal.gyro[i] - sequence.boundary_truth.gyro[i]).norm() < 1e-14,
                        "Independent gyro bias mean");
                require((cal.accel[i] - sequence.boundary_truth.accel[i]).norm() < 1e-12,
                        "Independent accel bias mean");
            }
            if (test == 2)
                require((cal.gyro[0] - cal.gyro[1]).norm() > 1e-6,
                        "Six IMU biases collapsed into a shared mean");
            if (test == 8)
            {
                MultiImuEskf filter(c, model);
                apply_initial_bias(filter.nodes, cal);
                for (int i = 0; i < 6; ++i)
                    require((filter.nodes[i].bg - cal.gyro[i]).norm() == 0,
                            "Bias initialization omitted");
                filter.propagate(sequence.dynamic[0].measured, .001);
                for (const auto& node : filter.nodes)
                    require(node.v.norm() < 1e-14 &&
                                node.attitude.q.angularDistance(Quaterniond::Identity()) < 1e-14,
                            "Calibrated bias not used by propagation");
            }
        }
        if (test == 3)
        {
            std::array<double, 2> mse{};
            for (int seed = 100; seed < 132; ++seed)
            {
                c["noise"]["seed"] = seed;
                auto s = make_calibration_sequence(c, "white", frames, 5000);
                for (int index = 0; index < 2; ++index)
                {
                    size_t n = index ? 5000 : 500;
                    std::vector<ImuMeasurements> w(s.stationary.end() - n, s.stationary.end());
                    auto cal = estimate_stationary_bias(w, initial, c, .001);
                    require(cal.available, "White-noise window rejected");
                    for (const auto& bias : cal.gyro)
                        mse[index] += bias.squaredNorm();
                }
            }
            double sigma = c["noise"]["gyro_white_std_deg_s"].get<double>() * pi / 180;
            for (int i = 0; i < 2; ++i)
            {
                double observed = mse[i] / (32 * 6 * 3),
                       expected = sigma * sigma / (i ? 5000 : 500);
                require(observed / expected > .7 && observed / expected < 1.3,
                        "White-noise mean variance inconsistent with sigma/sqrt(N)");
            }
            require(mse[1] < mse[0] / 5, "Longer white-noise window failed to reduce mean error");
        }
        if (test == 4)
        {
            auto s = make_calibration_sequence(c, "full", frames, 5000);
            auto cal = estimate_stationary_bias(s.stationary, initial, c, .001);
            require(cal.available, "Random-walk static window rejected");
            V3D mean = V3D::Zero();
            for (const auto& sample : s.stationary)
                mean += sample.gyro[0] / s.stationary.size();
            require((mean - cal.gyro[0]).norm() < 1e-14,
                    "Calibration used a hidden bias truth instead of measurements");
            require((s.stationary_truth.front().gyro[0] - s.boundary_truth.gyro[0]).norm() > 1e-6,
                    "Random walk missing from static prefix");
            require(mean_bias_error_variance(.001, .0001, 5000, .001) >
                        mean_bias_error_variance(.001, 0, 5000, .001),
                    "Random-walk error omitted from theory");
        }
        if (test == 5)
        {
            auto s = make_calibration_sequence(c, "none", frames, 500);
            s.stationary[250].gyro[2].x() = .2;
            require(!estimate_stationary_bias(s.stationary, initial, c, .001).available,
                    "Moving window accepted");
        }
        if (test == 6)
        {
            auto shape = forward_shape(model, increment(V3D(.3, -.2, .1)), {});
            std::vector<ImuMeasurements> samples(500);
            for (size_t k = 0; k < samples.size(); ++k)
            {
                samples[k].time = (k + 1) * .001;
                for (int i = 0; i < 6; ++i)
                    samples[k].force[i] =
                        shape.rotation[i].conjugate() * V3D(0, 0, 9.81) + V3D(.01, -.02, .015);
            }
            auto cal = estimate_stationary_bias(samples, shape, c, .001);
            require(cal.available, "Known tilted attitude rejected");
            for (const auto& ba : cal.accel)
                require((ba - V3D(.01, -.02, .015)).norm() < 1e-12,
                        "Known attitude accelerometer bias calculation");
        }
        if (test == 7 || test == 10)
        {
            auto s = make_calibration_sequence(c, "full", frames, 5000);
            auto dynamic = s.dynamic;
            Calibration first;
            for (size_t n : {500, 1000, 2000, 5000})
            {
                std::vector<ImuMeasurements> w(s.stationary.end() - n, s.stationary.end());
                auto cal = estimate_stationary_bias(w, initial, c, .001);
                require(cal.available, "Suffix calibration rejected");
                for (size_t k = 0; k < s.dynamic.size(); ++k)
                    for (int i = 0; i < 6; ++i)
                        require(s.dynamic[k].measured.gyro[i] == dynamic[k].measured.gyro[i] &&
                                    s.dynamic[k].truth.gyro[i] == dynamic[k].truth.gyro[i] &&
                                    s.dynamic[k].measured.force[i] ==
                                        dynamic[k].measured.force[i] &&
                                    s.dynamic[k].truth.accel[i] == dynamic[k].truth.accel[i],
                                "Duration changed dynamic noise/bias realization");
                if (n == 5000)
                    first = cal;
            }
            if (test == 10)
            {
                for (auto& truth : s.stationary_truth)
                    for (auto& bg : truth.gyro)
                        bg += V3D(10, 20, 30);
                s.boundary_truth.gyro.fill(V3D(100, 100, 100));
                auto second = estimate_stationary_bias(s.stationary, initial, c, .001);
                for (int i = 0; i < 6; ++i)
                    require(first.gyro[i] == second.gyro[i] && first.accel[i] == second.accel[i],
                            "Oracle truth leaked into normal calibration");
                bool rejected = false;
                try
                {
                    calibration_initialization(first, "真值零偏调试");
                }
                catch (const std::invalid_argument&)
                {
                    rejected = true;
                }
                require(rejected, "Normal initialization accepted Oracle strategy");
            }
        }
        if (test == 9)
        {
            auto s = make_calibration_sequence(c, "full", frames, 0);
            ImuNoise original(c["noise"], "full");
            MultiImuEskf baseline(c, model), initialized(c, model);
            Calibration unused;
            apply_initial_bias(initialized.nodes, calibration_initialization(unused, "未标定"));
            for (size_t k = 0; k < frames.size(); ++k)
            {
                auto canonical = original.apply(frames[k].imu, .001);
                for (int i = 0; i < 6; ++i)
                    require(canonical.measured.gyro[i] == s.dynamic[k].measured.gyro[i] &&
                                canonical.measured.force[i] == s.dynamic[k].measured.force[i],
                            "Disabled calibration changed canonical draws");
                baseline.propagate(canonical.measured, .001);
                initialized.propagate(s.dynamic[k].measured, .001);
                if ((k + 1) % 10 == 0)
                {
                    baseline.update((k + 1) * .001);
                    initialized.update((k + 1) * .001);
                }
            }
            require((baseline.nodes[0].p - initialized.nodes[0].p).norm() == 0 &&
                        (baseline.P - initialized.P).norm() == 0,
                    "Disabled calibration changed original propagation/update");
        }
        if (test == 11)
        {
            std::array<V3D, 6> p;
            std::array<Matrix15d, 6> covariance;
            JointEstimator estimate(model, c, "A2", Quaterniond::Identity());
            auto joints = estimate.update(frames[0].imu, .001);
            V3D root(.2, -.11, .02), mean_error = V3D::Zero();
            for (int i = 0; i < 6; ++i)
            {
                p[i] = root + initial.position[i] + V3D(i * .001, -i * .002, i * .003);
                covariance[i].setZero();
                mean_error += (p[i] - initial.position[i] - root) / 6;
            }
            auto fused = fuse_positions(p, covariance, model, initial, joints, c["fusion"]);
            require((fused.uniform - root - mean_error).norm() < 1e-14,
                    "Candidate mean vector does not reproduce uniform fusion");
        }
        std::cout << "Static calibration check " << test << " passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
