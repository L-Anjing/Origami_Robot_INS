#include "shape_experiment.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs = std::filesystem;
std::string contents(const fs::path& path)
{
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
int main(int argc, char** argv)
{
    fs::path temporary =
        fs::temp_directory_path() /
        ("origami_shape_test_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        if (argc != 2)
            return 2;
        fs::path root = argv[1];
        fs::create_directories(temporary);
        std::ifstream config_file(root / "config/shape_experiment.json");
        origami::Json config;
        config_file >> config;
        std::ifstream original(root / "data/data_L_qawp_20261030.txt");
        std::ofstream prefix(temporary / "input.txt");
        std::string line;
        for (int k = 0; k < 200 && std::getline(original, line); ++k)
            prefix << line << '\n';
        prefix.close();
        auto run = [&](const std::string& folder)
        {
            return origami::run_shape_experiment(config, (temporary / "input.txt").string(),
                                                 (temporary / folder).string(),
                                                 (root / "robot/zbot_crack_L2.xml").string());
        };
        auto first = run("one"), second = run("two");
        for (auto file : {"summary.json", "frames.jsonl", "joint_angles.csv", "shape_errors.csv",
                          "navigation_errors.csv", "bias_truth_full.jsonl"})
            if (contents(temporary / "one" / file) != contents(temporary / "two" / file))
                throw std::runtime_error(std::string("Non-reproducible output: ") + file);
        if (first.at("noise_modes").at("full").at("algorithms").at("A1").at("status") !=
            "unavailable")
            throw std::runtime_error("Invalid static calibration was accepted");
        // Alter an evaluation marker: ideal-model acceptance must fail before any D output is
        // emitted.
        std::string bad = contents(temporary / "input.txt");
        auto location = bad.find(";V ");
        if (location == std::string::npos)
            location = bad.find("; V ");
        auto v = bad.find("V ", location);
        if (v == std::string::npos)
            throw std::runtime_error("Cannot locate marker in fixture");
        auto token_end = bad.find(' ', v + 2);
        bad.replace(v + 2, token_end - (v + 2), "999");
        std::ofstream broken(temporary / "broken.txt");
        broken << bad;
        broken.close();
        bool rejected = false;
        try
        {
            origami::run_shape_experiment(config, (temporary / "broken.txt").string(),
                                          (temporary / "rejected").string(),
                                          (root / "robot/zbot_crack_L2.xml").string());
        }
        catch (const std::runtime_error&)
        {
            rejected = true;
        }
        if (!rejected || fs::exists(temporary / "rejected/navigation_errors.csv"))
            throw std::runtime_error("D ran after failed A-C validation");
        fs::remove_all(temporary);
        std::cout << "Experiment reproducibility and A-C acceptance gate passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        fs::remove_all(temporary);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
