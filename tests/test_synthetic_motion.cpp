#include "eskf.hpp"
#include "shape_kinematics.h"
#include "simulation_data.h"
#include "synthetic_motion.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc, char** argv)
{
    auto folder = std::filesystem::temp_directory_path() /
                  ("origami_synthetic_" +
                   std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        if (argc != 2)
            return 2;
        std::ifstream file(std::string(argv[1]) + "/config/multi_imu_eskf.json");
        origami::Json c;
        file >> c;
        c["experiment"]["synthetic_samples"] = 1000;
        auto model = origami::load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                               c["robot"]["geometry"]);
        for (auto& path : origami::generate_synthetic_motion(c, model, folder.string()))
        {
            auto data = origami::read_simulation(path, 1000);
            std::vector<origami::ErrorStateKalmanFilter> states;
            for (int i = 0; i < 6; ++i)
            {
                states.emplace_back(c, false, false, false, true);
                states[i].p = origami::V3D(.2 + .0501 * i, -.11, .02);
            }
            for (auto& frame : data)
                for (int i = 0; i < 6; ++i)
                {
                    states[i].propagate(frame.imu.gyro[i], frame.imu.force[i], .001, false);
                    if ((states[i].p - frame.truth.markers[i]).norm() > 1e-10 ||
                        states[i].attitude.q.angularDistance(frame.truth.orientation[i]) > 1e-10)
                        throw std::runtime_error("Synthetic sensor/kinematic inconsistency");
                }
        }
        std::filesystem::remove_all(folder);
        std::cout << "Five physics-consistent discrete motion scenes passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::filesystem::remove_all(folder);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
