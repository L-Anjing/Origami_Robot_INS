#include "mat_export.h"
#include "robot_odometry.h"
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
                         "MODEL [estimated|ground_truth|all]\n";
            return 2;
        }
        std::ifstream file(argv[2]);
        if (!file)
            throw std::runtime_error("Cannot read config");
        origami::Json c;
        file >> c;
        std::string task = argv[1];
        if (task == "simulation")
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
