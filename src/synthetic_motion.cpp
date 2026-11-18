#include "synthetic_motion.h"
#include "shape_kinematics.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
namespace origami
{
std::vector<std::string> generate_synthetic_motion(const Json& c, const Model& model,
                                                   const std::string& output)
{
    for (const auto& h : model.joints)
        if (h.rest_rotation.angularDistance(Quaterniond::Identity()) > 1e-12 ||
            h.anchor_child.norm() > 1e-12 || std::abs(std::abs(h.axis_child.z()) - 1) > 1e-12 ||
            std::abs(h.reference) > 1e-12)
            throw std::invalid_argument(
                "This controlled motion generator explicitly targets the current planar "
                "alternating-Z MJCF; the estimator itself supports general axes");
    std::filesystem::create_directories(output);
    double dt = 1 / c.at("sample_rate_hz").get<double>();
    int count = c.at("experiment").value("synthetic_samples", 10864);
    std::vector<std::string> paths;
    // Smooth articulated motions are generated afresh through FK. Sensor samples are discrete
    // interval equivalents.
    for (auto name :
         {"fixed_translation", "small_fold", "large_fold", "alternating_fold", "combined_motion"})
    {
        std::string scene = name, path = output + "/" + scene + ".txt";
        paths.push_back(path);
        std::ofstream out(path);
        out << std::setprecision(17);
        std::array<V3D, 6> previous_position, previous_velocity;
        previous_velocity.fill(V3D::Zero());
        V3D root0 = vector3(c.at("robot").at("initial_root_W_m"));
        for (int i = 0; i < 6; ++i)
            previous_position[i] = root0 + V3D(.0501 * i, 0, 0);
        std::array<double, 6> previous_yaw{};
        for (int k = 1; k <= count; ++k)
        {
            double t = k * dt;
            std::array<double, 5> angles{};
            double root_yaw = scene == "combined_motion" ? .12 * (1 - std::cos(.6 * t)) : 0;
            V3D root = root0;
            if (scene == "fixed_translation" || scene == "combined_motion")
            {
                root.x() += .04 * (t - std::sin(.8 * t) / .8);
                root.y() += .015 * (1 - std::cos(.7 * t));
            }
            for (int j = 0; j < 5; ++j)
                if (scene != "fixed_translation")
                {
                    double amplitude = scene == "small_fold" ? .12 : 1.1;
                    double frequency = scene == "alternating_fold" || scene == "combined_motion"
                                           ? .7 + .13 * j
                                           : .7;
                    angles[j] = amplitude * .5 * (1 - std::cos(frequency * t));
                }
            auto shape = forward_shape(
                model, Quaterniond(Eigen::AngleAxisd(root_yaw, V3D::UnitZ())), angles);
            std::array<V3D, 6> position;
            double yaw = root_yaw;
            for (int i = 0; i < 6; ++i)
            {
                position[i] = root + shape.position[i];
                V3D velocity = (position[i] - previous_position[i]) / dt,
                    acceleration = (velocity - previous_velocity[i]) / dt;
                V3D force = shape.rotation[i].conjugate() *
                            (acceleration + V3D(0, 0, c.at("gravity_m_s2").get<double>()));
                if (i > 0)
                    yaw += model.joints[i - 1].axis_child.z() * angles[i - 1];
                V3D gyro(0, 0, (yaw - previous_yaw[i]) / dt);
                auto q = shape.rotation[i];
                if (i == 0)
                    out << k << ' ';
                out << i << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
                for (double v : force)
                    out << ' ' << v;
                for (double v : gyro)
                    out << ' ' << v;
                out << "; ";
                previous_position[i] = position[i];
                previous_velocity[i] = velocity;
                previous_yaw[i] = yaw;
            }
            out << "0 0 0; V";
            for (auto& p : position)
                for (double x : p)
                    out << ' ' << x;
            for (double x : V3D(root + shape.terminal))
                out << ' ' << x;
            out << '\n';
        }
        if (!out)
            throw std::runtime_error("Synthetic dataset write failed");
    }
    Json manifest = {
        {"samples", count},
        {"dt_s", dt},
        {"source",
         "New smooth joint/root trajectories through FK, not rescaled original endpoints"},
        {"imu_convention", "Interval-equivalent angular increments and discrete velocity "
                           "increments; exact with documented step-end propagation. Not "
                           "instantaneous continuous-time end-sample derivatives."},
        {"initial_shape", {0, 0, 0, 0, 0}},
        {"initial_velocity", {0, 0, 0}},
        {"files", paths}};
    std::ofstream index(output + "/manifest.json");
    index << manifest.dump(2);
    return paths;
}
} // namespace origami
