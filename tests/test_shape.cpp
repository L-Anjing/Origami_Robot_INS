#include "joint_estimator.h"
#include "simulation_data.h"
#include <fstream>
#include <iostream>
using namespace origami;
void require(bool ok, const std::string& message)
{
    if (!ok)
        throw std::runtime_error(message);
}
Model synthetic()
{
    Model model;
    for (int j = 0; j < 5; ++j)
    {
        auto& h = model.joints[j];
        h.parent = j;
        h.child = j + 1;
        h.axis_child = V3D(0, 0, j % 2 ? -1 : 1);
        h.axis_parent = h.axis_child;
        h.translation = V3D(.0501, 0, 0);
        model.offsets[j] = h.translation;
    }
    model.offsets[5] = V3D(.0501, 0, 0);
    return model;
}
int main(int argc, char** argv)
{
    try
    {
        std::ifstream file(std::string(argv[1]) + "/config/shape_experiment.json");
        Json c;
        file >> c;
        Model model = synthetic();
        std::array<double, 5> zero{};
        auto shape = forward_shape(model, Quaterniond::Identity(), zero);
        require(std::abs(shape.terminal.x() - .3006) < 1e-12, "50.1mm marker geometry");
        require(std::abs(shape.physical_tip.x() - .3005) < 1e-12, "50mm physical tip");
        ImuMeasurements input;
        for (auto& f : input.force)
            f = V3D(0, 0, 9.81);
        for (std::string mode : {"A0", "A2"})
        {
            JointEstimator est(model, c, mode, Quaterniond::Identity());
            for (int k = 0; k < 100; ++k)
            {
                auto result = est.update(input, .001);
                for (double a : result.angle)
                    require(a == 0, "Zero-noise static chain");
            }
        }
        require(std::abs(joint_angle_from_relative(model.joints[0], increment(V3D(0, 0, .3)),
                                                   6 * pi + .3) -
                         (6 * pi + .3)) < 1e-12,
                "Joint angle unwrap");
        // Both joint signs, both motion directions, all five joints and folded priors.
        for (int joint = 0; joint < 5; ++joint)
            for (double sign : {-1., 1.})
            {
                auto config = c;
                std::array<double, 5> initial{.1, .2, .3, .4, .5};
                config["initial_joint_angles_rad"] = initial;
                JointEstimator est(model, config, "A0", Quaterniond::Identity());
                ImuMeasurements m = input;
                for (int node = joint + 1; node < 6; ++node)
                    m.gyro[node] = model.joints[joint].axis_child * sign;
                JointEstimate state;
                for (int k = 0; k < 100; ++k)
                    state = est.update(m, .001);
                for (int j = 0; j < 5; ++j)
                    require(std::abs(state.angle[j] - (initial[j] + (j == joint ? .1 * sign : 0))) <
                                1e-12,
                            "Joint sign or initial fold");
            }
        JointEstimator biased(model, c, "A0", Quaterniond::Identity());
        ImuMeasurements bias = input;
        bias.gyro[1].z() = .01;
        JointEstimate state;
        for (int k = 0; k < 1000; ++k)
            state = biased.update(bias, .001);
        require(std::abs(state.angle[0] - .01) < 1e-12, "Bias theoretical integration drift");
        ImuNoise n1(c["noise"], "full"), n2(c["noise"], "full");
        for (int k = 0; k < 100; ++k)
        {
            auto a = n1.apply(input, .001), b = n2.apply(input, .001);
            for (int i = 0; i < 6; ++i)
                require(a.measured.gyro[i] == b.measured.gyro[i] &&
                            a.measured.force[i] == b.measured.force[i],
                        "Seed reproducibility");
        }
        Json density_config = c["noise"];
        density_config["white_noise_definition"] = "density";
        density_config["gyro_white_density_deg_s_per_sqrt_hz"] =
            c["noise"]["gyro_white_std_deg_s"].get<double>() * std::sqrt(.001);
        density_config["accel_white_density_m_s2_per_sqrt_hz"] =
            c["noise"]["accel_white_std_m_s2"].get<double>() * std::sqrt(.001);
        ImuNoise discrete(c["noise"], "white"), density(density_config, "white");
        auto nd = discrete.apply(input, .001), nc = density.apply(input, .001);
        for (int i = 0; i < 6; ++i)
            require((nd.measured.gyro[i] - nc.measured.gyro[i]).norm() < 1e-15 &&
                        (nd.measured.force[i] - nc.measured.force[i]).norm() < 1e-15,
                    "White density discretization");
        Json walk_config = c["noise"];
        walk_config["gyro_initial_bias_bound_deg_s"] = 0.;
        walk_config["accel_initial_bias_bound_m_s2"] = 0.;
        ImuNoise walk1(walk_config, "full"), walk4(walk_config, "full");
        auto rw1 = walk1.apply(input, .001), rw4 = walk4.apply(input, .004);
        for (int i = 0; i < 6; ++i)
            require((rw4.truth.gyro[i] - 2 * rw1.truth.gyro[i]).norm() < 1e-15,
                    "Bias random walk sqrt(dt)");
        // Generic tilted rest frame and non-Z axis, not the current alternating-Z specialization.
        model.joints[0].rest_rotation = Quaterniond(Eigen::AngleAxisd(.4, V3D::UnitY()));
        model.joints[0].axis_child = V3D::UnitX();
        model.joints[0].axis_parent = model.joints[0].rest_rotation * V3D::UnitX();
        model.joints[0].anchor_child = V3D(.01, .02, .03);
        std::array<double, 5> angles{.3, -.2, .1, -.1, .4};
        shape = forward_shape(model, Quaterniond::Identity(), angles);
        for (int j = 0; j < 5; ++j)
        {
            auto relative = shape.rotation[j].conjugate() * shape.rotation[j + 1];
            require(std::abs(joint_angle_from_relative(model.joints[j], relative, angles[j]) -
                             angles[j]) < 1e-12,
                    "Generic FK twist");
        }
        JointEstimator generic(model, c, "A0", Quaterniond::Identity());
        ImuMeasurements m = input;
        for (int i = 1; i < 6; ++i)
            m.gyro[i] = V3D(.7, 0, 0);
        auto gs = generic.update(m, .01);
        require(std::abs(gs.rate[0] - .7) < 1e-12, "Non-Z gyro axis");
        // Verified static calibration, rejection of dynamic data, and A1 gating.
        auto straight = synthetic();
        auto init = forward_shape(straight, Quaterniond::Identity(), zero);
        std::vector<ImuMeasurements> quiet;
        for (int k = 0; k < 101; ++k)
        {
            auto sample = input;
            sample.time = k * .001;
            for (int i = 0; i < 6; ++i)
                sample.gyro[i] = V3D(.001 * (i + 1), -.0002 * (i + 1), .0003 * i);
            quiet.push_back(sample);
        }
        auto cal = calibrate_initial_static(quiet, init, c);
        require(cal.available, "Static calibration");
        JointEstimator a1(straight, c, "A1", Quaterniond::Identity(), cal);
        auto calibrated = a1.update(quiet.back(), .001);
        require(std::abs(calibrated.angle[0]) < 1e-12, "A1 bias removal");
        quiet[0].force[0].z() = 300;
        require(!calibrate_initial_static(quiet, init, c).available,
                "Impact accepted as calibration");
        // Data-level ideal consistency: truth is evaluation-only and never passed into the
        // estimator.
        auto actual = load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                       c["robot"]["geometry"]);
        auto frames =
            read_simulation(std::string(argv[1]) + "/data/data_L_qawp_20261030.txt", 1000);
        JointEstimator ideal(actual, c, "A0", Quaterniond::Identity());
        SixAxisAhrs root;
        double max_joint = 0, max_shape = 0;
        std::array<double, 5> truth{};
        for (auto& frame : frames)
        {
            auto estimate = ideal.update(frame.imu, .001);
            root.update(frame.imu.gyro[0], frame.imu.force[0], .001, c["robot"]["ahrs"], 9.81,
                        false, true);
            auto rebuilt = forward_shape(actual, root.q, estimate.angle);
            for (int j = 0; j < 5; ++j)
            {
                truth[j] = joint_angle_from_relative(actual.joints[j],
                                                     frame.truth.orientation[j].conjugate() *
                                                         frame.truth.orientation[j + 1],
                                                     truth[j]);
                max_joint = std::max(max_joint, std::abs(estimate.angle[j] - truth[j]));
            }
            for (int i = 0; i < 6; ++i)
                max_shape = std::max(max_shape, (rebuilt.position[i] -
                                                 (frame.truth.markers[i] - frame.truth.markers[0]))
                                                    .norm());
        }
        require(max_joint < 1e-6, "Ideal joint data/model mismatch");
        require(max_shape < 2e-6, "Ideal shape data/model mismatch");
        std::cout << "A-C passed; max ideal joint=" << max_joint << " rad; shape=" << max_shape
                  << " m\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
