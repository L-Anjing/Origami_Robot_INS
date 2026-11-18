#include "multi_imu_fusion.h"
#include "simulation_data.h"
#include <fstream>
#include <iostream>
using namespace origami;
void require(bool value, const std::string& why)
{
    if (!value)
        throw std::runtime_error(why);
}
int main(int argc, char** argv)
{
    try
    {
        if (argc != 2)
            return 2;
        std::ifstream in(std::string(argv[1]) + "/config/shape_experiment.json");
        Json c;
        in >> c;
        auto model = load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                      c["robot"]["geometry"]);
        std::array<double, 5> angle{.3, .2, .4, .1, .5}, rates{.2, -.1, .3, -.2, .1};
        Quaterniond root(Eigen::AngleAxisd(.2, V3D::UnitY()));
        auto shape = forward_shape(model, root, angle);
        V3D omega(.1, .2, .3), velocity(.2, -.1, .3);
        auto v = initial_node_velocities(model, shape, rates, omega, velocity);
        auto next = angle;
        double dt = 1e-7;
        for (int j = 0; j < 5; ++j)
            next[j] += rates[j] * dt;
        auto moved = forward_shape(model, increment(omega * dt) * root, next);
        for (int i = 0; i < 6; ++i)
            require((v[i] - velocity - (moved.position[i] - shape.position[i]) / dt).norm() < 1e-6,
                    "Node velocity initialization inconsistent with FK");
        std::array<V3D, 6> positions;
        std::array<Matrix15d, 6> covariance;
        V3D p(.4, -.2, .1);
        JointEstimate joint;
        joint.valid.fill(true);
        for (int i = 0; i < 6; ++i)
        {
            positions[i] = p + shape.position[i];
            covariance[i] = Matrix15d::Identity() * .001;
        }
        auto fused = fuse_positions(positions, covariance, model, shape, joint, c["fusion"]);
        require((fused.uniform - p).norm() < 1e-12 && (fused.weighted - p).norm() < 1e-12,
                "Six-node shape compensation");
        double sum = 0;
        for (auto w : fused.weights)
            sum += w;
        require(std::abs(sum - 1) < 1e-12, "Normalized fusion weights");
        auto healthy = fused.weights;
        joint.valid[0] = false;
        fused = fuse_positions(positions, covariance, model, shape, joint, c["fusion"]);
        require(fused.weights[5] < healthy[5], "Invalid ancestor did not reduce node weight");
        V3D translation(3, -1, 2);
        for (auto& x : positions)
            x += translation;
        auto translated = fuse_positions(positions, covariance, model, shape, joint, c["fusion"]);
        require((translated.weighted - fused.weighted - translation).norm() < 1e-12,
                "Global translation equivariance");
        require(translated.weighted_trace_bound_m2 >= 0, "Invalid uncertainty bound");
        // General axes, rotated local frames, and finite joint anchors.
        Model general = model;
        for (int j = 0; j < 5; ++j)
        {
            auto& h = general.joints[j];
            h.axis_child = V3D(j + 1, 2 - j, .5).normalized();
            h.rest_rotation = increment(V3D(.1 * j, .05, -.08));
            h.axis_parent = h.rest_rotation * h.axis_child;
            h.anchor_child = V3D(.003, -.004, .005);
        }
        auto gshape = forward_shape(general, root, angle);
        auto gv = initial_node_velocities(general, gshape, rates, omega, velocity);
        auto gnext = forward_shape(general, increment(omega * dt) * root, next);
        for (int i = 0; i < 6; ++i)
            require((gv[i] - velocity - (gnext.position[i] - gshape.position[i]) / dt).norm() <
                        1e-6,
                    "Generic anchor velocity");
        auto gc = c;
        gc["initial_joint_angles_rad"] = angle;
        JointEstimator general_est(general, gc, "A0", root);
        ImuMeasurements general_imu;
        general_imu.gyro[0] = V3D(.2, -.1, .3);
        for (int j = 0; j < 5; ++j)
        {
            const auto& h = general.joints[j];
            Quaterniond relative =
                h.rest_rotation * Quaterniond(Eigen::AngleAxisd(angle[j], h.axis_child));
            general_imu.gyro[j + 1] =
                relative.conjugate() * general_imu.gyro[j] + h.axis_child * rates[j];
            V3D pivot_parent =
                gshape.position[j] +
                gshape.rotation[j] * (h.translation + h.rest_rotation * h.anchor_child);
            V3D pivot_child = gshape.position[j + 1] + gshape.rotation[j + 1] * h.anchor_child;
            require((pivot_parent - pivot_child).norm() < 1e-12, "MJCF pivot preservation");
        }
        auto general_result = general_est.update(general_imu, .001);
        for (int j = 0; j < 5; ++j)
            require(std::abs(general_result.rate[j] - rates[j]) < 1e-12,
                    "Rotated frame joint projection");
        // Observational firewall: perturb all evaluation truth while sensor-only algorithm inputs
        // remain identical.
        auto frames =
            read_simulation(std::string(argv[1]) + "/data/data_L_qawp_20261030.txt", 1000);
        auto corrupted = frames[0];
        for (auto& x : corrupted.truth.markers)
            x += V3D(100, 200, 300);
        for (auto& q : corrupted.truth.orientation)
            q = increment(V3D(1, 2, 3));
        JointEstimator a(model, c, "A0", Quaterniond::Identity()),
            b(model, c, "A0", Quaterniond::Identity());
        auto ja = a.update(frames[0].imu, .001), jb = b.update(corrupted.imu, .001);
        require(ja.angle == jb.angle, "Evaluation truth leaked into estimator");
        // Arbitrary 5-joint motion, not five isolated single-joint checks.
        JointEstimator all(model, c, "A0", Quaterniond::Identity());
        std::array<double, 5> rate{.1, -.2, .3, -.4, .5};
        ImuMeasurements observations;
        double wz = 0;
        for (auto& f : observations.force)
            f = V3D(0, 0, 9.81);
        for (int i = 1; i < 6; ++i)
        {
            wz += model.joints[i - 1].axis_child.z() * rate[i - 1];
            observations.gyro[i] = V3D(0, 0, wz);
        }
        JointEstimate estimate;
        for (int k = 0; k < 1000; ++k)
            estimate = all.update(observations, .001);
        for (int j = 0; j < 5; ++j)
            require(std::abs(estimate.angle[j] - rate[j]) < 1e-11,
                    "Five-joint simultaneous motion");
        std::cout << "Multi-IMU fusion, velocity initialization and truth isolation passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
