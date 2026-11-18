#include "multi_imu_experiment.h"
#include "synthetic_motion.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc, char** argv)
{
    auto temporary = std::filesystem::temp_directory_path() /
                     ("joint_experiment_" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        if (argc != 2)
            return 2;
        std::ifstream in(std::string(argv[1]) + "/config/multi_imu_eskf.json");
        origami::Json c;
        in >> c;
        c["experiment"]["synthetic_samples"] = 100;
        c["active_noise_mode"] = "none";
        auto model = origami::load_robot_model(std::string(argv[1]) + "/robot/zbot_crack_L2.xml",
                                               c["robot"]["geometry"]);
        auto paths = origami::generate_synthetic_motion(c, model, (temporary / "data").string());
        auto report = origami::run_multi_imu_experiment(
            c, paths.at(2), (temporary / "result").string(),
            std::string(argv[1]) + "/robot/zbot_crack_L2.xml", false);
        if (report.at("B1_statistics").at("zupt_accepted").get<size_t>() == 0)
            throw std::runtime_error("B1 must accept ZUPT for a verified static S0");
        if (report.at("metrics").at("B1_ZUPT").at("ate_m").get<double>() > 1e-10)
            throw std::runtime_error("Static B1 navigation error");
        std::filesystem::remove_all(temporary);
        std::cout << "Joint experiment B1 detector parameter order and accepted ZUPT passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::filesystem::remove_all(temporary);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
