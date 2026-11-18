#include "multi_imu_eskf.h"
#include "multi_imu_fusion.h"
#include "simulation_data.h"
#include <fstream>
#include <iostream>
using namespace origami;
void require(bool ok, const std::string& why)
{
    if (!ok)
        throw std::runtime_error(why);
}
void perturb(std::vector<ErrorStateKalmanFilter>& x, int k, double d)
{
    int i = k / 15, j = k % 15;
    if (j < 3)
        x[i].p[j] += d;
    else if (j < 6)
        x[i].v[j - 3] += d;
    else if (j < 9)
    {
        V3D a = V3D::Zero();
        a[j - 6] = d;
        x[i].attitude.q = x[i].attitude.q * increment(a);
    }
    else if (j < 12)
        x[i].bg[j - 9] += d;
    else
        x[i].ba[j - 12] += d;
}
int main(int argc, char** argv)
{
    try
    {
        if (argc != 3)
            return 2;
        int test = std::stoi(argv[2]);
        std::ifstream in(std::string(argv[1]) + "/config/multi_imu_eskf.json");
        Json c;
        in >> c;
        auto model = load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                      c["robot"]["geometry"]);
        MultiImuEskf f(c, model);
        ImuMeasurements m;
        for (auto& force : m.force)
            force = V3D(0, 0, 9.81);
        auto angles = std::array<double, 5>{.1, .2, .3, .4, .5};
        auto fk = forward_shape(model, increment(V3D(.2, -.1, .3)), angles);
        auto moved = f.nodes;
        for (int i = 0; i < 6; ++i)
        {
            moved[i].p = V3D(.2, -.11, .02) + fk.position[i];
            moved[i].attitude.q = fk.rotation[i];
        }
        if (test == 1)
        {
            for (double a : f.angles)
                require(a == 0, "Initial joints");
            for (auto& x : f.nodes)
                require(x.attitude.q.angularDistance(Quaterniond::Identity()) < 1e-14,
                        "Initial orientation");
        }
        if (test == 2)
        {
            for (int i = 0; i < 6; ++i)
                require((f.nodes[i].p - V3D(.2 + .0501 * i, -.11, .02)).norm() < 1e-14,
                        "Initial world positions");
        }
        if (test == 3)
        {
            for (auto& force : m.force)
                force = V3D(1, 0, 9.81);
            f.propagate(m, .001);
            require(std::abs(f.nodes[0].p.x() - .200001) < 1e-13, "t=0 first step integration");
        }
        if (test == 4 || test == 5 || test == 6 || test == 7)
        {
            auto& nodes = (test == 4 || test == 5) ? f.nodes : moved;
            for (int j = 0; j < 5; ++j)
            {
                require(hinge_position(nodes, model.joints[j], j, 90).value.norm() < 1e-12,
                        "Shared hinge");
                require(hinge_axis(nodes, model.joints[j], j, 90).value.norm() < 1e-12,
                        "Same physical axis");
                if (test == 5)
                    require(std::abs(model.joints[j].axis_child.z() - (j % 2 ? -1 : 1)) < 1e-12,
                            "MJCF axis sign");
            }
        }
        if (test == 8 || test == 9)
        {
            moved[2].attitude.q = moved[2].attitude.q * increment(V3D(.03, -.02, .01));
            for (int j = 0; j < 5; ++j)
            {
                auto base = test == 8 ? hinge_position(moved, model.joints[j], j, 90)
                                      : hinge_axis(moved, model.joints[j], j, 90);
                Eigen::MatrixXd numerical = Eigen::MatrixXd::Zero(base.value.size(), 90);
                for (int k = 0; k < 90; ++k)
                {
                    auto plus = moved, minus = moved;
                    perturb(plus, k, 1e-7);
                    perturb(minus, k, -1e-7);
                    auto p = test == 8 ? hinge_position(plus, model.joints[j], j, 90)
                                       : hinge_axis(plus, model.joints[j], j, 90, &base.tangent);
                    auto n = test == 8 ? hinge_position(minus, model.joints[j], j, 90)
                                       : hinge_axis(minus, model.joints[j], j, 90, &base.tangent);
                    numerical.col(k) = (p.value - n.value) / 2e-7;
                }
                require((numerical - base.H).cwiseAbs().maxCoeff() < 1e-7,
                        "Constraint analytical Jacobian");
            }
        }
        if (test == 10 || test == 11 || test == 13)
        {
            f.P = Eigen::MatrixXd::Identity(90, 90) * .01;
            f.nodes[1].p.x() += .01;
            double before = hinge_position(f.nodes, model.joints[0], 0, 90).value.norm();
            f.update(0);
            require(hinge_position(f.nodes, model.joints[0], 0, 90).value.norm() < before,
                    "Correction direction");
            require(f.P.block<15, 15>(0, 15).norm() > 1e-8, "Cross covariance created");
            f.diagnostics();
        }
        if (test == 12)
        {
            double before = f.P.block<15, 15>(0, 15).norm();
            f.propagate(m, .001);
            require(before > 0 && f.P.block<15, 15>(0, 15).norm() > 0,
                    "Cross covariance retained by propagation");
        }
        if (test == 14)
        {
            Eigen::MatrixXd gauge = Eigen::MatrixXd::Zero(90, 7);
            for (int i = 0; i < 6; ++i)
            {
                gauge.block<3, 3>(15 * i, 0).setIdentity();
                gauge.block<3, 3>(15 * i + 3, 3).setIdentity();
                gauge.block<3, 1>(15 * i, 6) = V3D::UnitZ().cross(moved[i].p);
                gauge.block<3, 1>(15 * i + 6, 6) = moved[i].attitude.q.conjugate() * V3D::UnitZ();
            }
            for (int j = 0; j < 5; ++j)
            {
                require((hinge_position(moved, model.joints[j], j, 90).H * gauge).norm() < 1e-12,
                        "False absolute position information");
                require((hinge_axis(moved, model.joints[j], j, 90).H * gauge).norm() < 1e-12,
                        "False yaw information");
            }
        }
        if (test == 15)
        {
            auto independent = f.nodes;
            for (int k = 0; k < 100; ++k)
            {
                for (int i = 0; i < 6; ++i)
                    independent[i].propagate(m.gyro[i], m.force[i], .001);
                f.propagate(m, .001);
            }
            for (int i = 0; i < 6; ++i)
            {
                require((f.nodes[i].p - independent[i].p).norm() < 1e-12,
                        "Independent nominal parity");
                require((f.P.block<15, 15>(15 * i, 15 * i) - independent[i].P).norm() < 1e-10,
                        "Independent covariance parity");
            }
        }
        if (test == 16 || test == 17 || test == 19)
        {
            auto data =
                read_simulation(std::string(argv[1]) + "/data/data_L_qawp_20261030.txt", 1000);
            auto original = f.nodes[0].p;
            MultiImuEskf other(c, model);
            auto altered = data[0];
            for (auto& p : altered.truth.markers)
                p += V3D(100, 20, -30);
            for (auto& q : altered.truth.orientation)
                q = increment(V3D(2, 1, 3));
            f.propagate(data[0].imu, .001);
            other.propagate(altered.imu, .001);
            require((f.nodes[0].p - other.nodes[0].p).norm() == 0, "Truth leak");
            require((f.nodes[0].p - original).norm() > 0, "First frame overwritten");
            require((f.nodes[0].p - data[0].truth.markers[0]).norm() < 1e-8,
                    "First step timestamp mismatch");
        }
        if (test == 18)
        {
            ImuNoise a(c["noise"], "full"), b(c["noise"], "full");
            auto x = a.apply(m, .001), y = b.apply(m, .001);
            for (int i = 0; i < 6; ++i)
                require(x.measured.gyro[i] == y.measured.gyro[i], "Seed reproducibility");
        }
        if (test == 20)
        {
            c["joint_filter"]["enable_shape_state"] = true;
            MultiImuEskf d(c, model);
            d.propagate(m, .001);
            auto& Q = d.last_process_noise();
            require(Q.block(0, 90, 90, 5).norm() > 0, "D4 gyro-angle cross noise omitted");
            require(std::abs(Q(90, 91)) > 0, "Shared adjacent joint gyro noise omitted");
            d.diagnostics();
            double gyro_variance = d.nodes[0].noise_variance[2];
            require(std::abs(Q(8, 90) + gyro_variance) < 1e-20, "D4 parent gyro covariance sign");
            require(std::abs(Q(23, 90) - gyro_variance) < 1e-20, "D4 child gyro covariance sign");
            require(std::abs(Q(90, 91) - gyro_variance) < 1e-20,
                    "Adjacent joint shared gyro covariance sign");
            require(std::abs(d.last_transition()(90, 11) - .001) < 1e-15,
                    "D4 parent bias transition");
            require(std::abs(d.last_transition()(90, 26) + .001) < 1e-15,
                    "D4 child bias transition");
            moved[2].attitude.q = moved[2].attitude.q * increment(V3D(.03, -.02, .01));
            moved[3].p += V3D(.001, -.002, .003);
            auto constraints = shape_constraints(moved, model, angles, 95);
            for (auto& base : constraints)
            {
                Eigen::MatrixXd numerical = Eigen::MatrixXd::Zero(3, 95);
                for (int k = 0; k < 95; ++k)
                {
                    auto plus = moved, minus = moved;
                    auto ap = angles, am = angles;
                    if (k < 90)
                    {
                        perturb(plus, k, 1e-7);
                        perturb(minus, k, -1e-7);
                    }
                    else
                    {
                        ap[k - 90] += 1e-7;
                        am[k - 90] -= 1e-7;
                    }
                    auto ps = shape_constraints(plus, model, ap, 95),
                         ms = shape_constraints(minus, model, am, 95);
                    size_t index = &base - constraints.data();
                    numerical.col(k) = (ps[index].value - ms[index].value) / 2e-7;
                }
                require((numerical - base.H).cwiseAbs().maxCoeff() < 2e-7,
                        "D4 analytical observation Jacobian");
            }
            d.update(.001);
            d.diagnostics();
        }
        std::cout << "T" << test << " passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
