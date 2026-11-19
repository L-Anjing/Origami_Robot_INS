#include "mat_export.h"
#include "multi_imu_experiment.h"
#include "robot_odometry.h"
#include "shape_experiment.h"
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc, char** argv)
{
    try
    {
        if (argc == 3 && std::string(argv[1]) == "export-mat")
        {
            origami::export_mat(argv[2]);
            return 0;
        }
        if (argc < 5)
        {
            std::cerr << "Usage: origami_ins real CONFIG DATA OUTPUT [STATIC_SECONDS] "
                         "[glrt|none]\n       origami_ins simulation CONFIG DATA OUTPUT "
                         "MODEL [estimated|ground_truth|all]\n"
                         "       origami_ins shape-fusion CONFIG DATA OUTPUT MODEL\n";
            return 2;
        }
        std::ifstream file(argv[2]);
        if (!file)
            throw std::runtime_error("Cannot read config");
        origami::Json c;
        file >> c;
        std::string task = argv[1];
        if (task == "multi-imu-run" || task == "multi-imu-suite")
        {
            if (argc != 6 && !(task == "multi-imu-run" && argc == 8 &&
                               std::string(argv[6]) == "--imu-count"))
                throw std::invalid_argument(
                    "Usage: origami_ins multi-imu-run CONFIG DATA OUTPUT MODEL [--imu-count N]"
                    " (multi-imu-suite uses CONFIG only)");
            if (argc == 8)
            {
                std::string value = argv[7];
                if (value.size() != 1 || value[0] < '1' || value[0] > '6')
                    throw std::invalid_argument("--imu-count must be an integer in [1, 6]");
                c["joint_filter"]["imu_count"] = value[0] - '0';
            }
            auto report = origami::run_multi_imu_experiment(c, argv[3], argv[4], argv[5],
                                                            task == "multi-imu-suite");
            std::cout << "Saved joint filter results; cases=" << report.value("case_count", 1)
                      << '\n';
        }
        else if (task == "shape-fusion" || task == "shape-only")
        {
            if (argc != 6)
                throw std::invalid_argument(
                    "Usage: origami_ins shape-fusion CONFIG DATA OUTPUT MODEL");
            auto result =
                origami::run_shape_experiment(c, argv[3], argv[4], argv[5], task == "shape-only");
            std::cout << result.dump(2) << '\n';
        }
        else if (task == "simulation")
        {
            if (argc < 6)
                throw std::invalid_argument("Model path required");
            std::string mode = argc > 6 ? argv[6] : "all";
            if (mode == "all")
                for (auto name : {"estimated", "ground_truth"})
                    std::cout << origami::run_simulation(argv[3], argv[5], c, name,
                                                         std::string(argv[4]) + "/" + name)
                                     .dump(2)
                              << '\n';
            else
                std::cout << origami::run_simulation(argv[3], argv[5], c, mode, argv[4]).dump(2)
                          << '\n';
        }
        else if (task == "real")
        {
            std::string mode = argc > 6 ? argv[6] : "glrt";
            if (mode != "glrt" && mode != "none")
                throw std::invalid_argument("Unknown ZUPT mode");
            origami::ImuProcess nav(c, argc > 5 ? std::stod(argv[5]) : 2, mode == "glrt");
            auto samples = origami::read_recording(argv[3]);
            std::filesystem::create_directories(argv[4]);
            std::ofstream out(std::string(argv[4]) + "/frames.jsonl");
            if (!out)
                throw std::runtime_error("Cannot write output");
            size_t count = 0, updates = 0;
            for (auto& s : samples)
                for (auto& state : nav.push(s))
                {
                    out << state.dump() << '\n';
                    if (!out)
                        throw std::runtime_error("Write failed");
                    ++count;
                    updates += state.at("accepted").get<bool>();
                }
            if (!count)
                throw std::runtime_error("Recording ends before alignment completes");
            nav.eskf->check_covariance();
            origami::Json summary = {
                {"samples", count}, {"zupt_updates", updates}, {"final", nav.eskf->snapshot()}};
            std::ofstream report(std::string(argv[4]) + "/summary.json");
            report << summary.dump(2);
            std::cout << summary.dump(2) << '\n';
        }
        else
            throw std::invalid_argument("Unknown task");
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "origami_ins: " << e.what() << '\n';
        return 1;
    }
}
