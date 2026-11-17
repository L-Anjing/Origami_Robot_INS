#include "mat_export.h"
#include "robot_odometry.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs = std::filesystem;
using origami::Json;

Json read_json(const fs::path& path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Cannot read " + path.string());
    Json value;
    input >> value;
    return value;
}
void compare(const Json& actual, const Json& expected, const std::string& path)
{
    if (expected.is_number())
    {
        double a = actual.get<double>(), b = expected.get<double>();
        if (!std::isfinite(a) || std::abs(a - b) > 1e-8 + 1e-10 * std::abs(b))
            throw std::runtime_error("Numeric regression: " + path);
    }
    else if (expected.is_array())
    {
        if (actual.size() != expected.size())
            throw std::runtime_error("Shape regression: " + path);
        for (size_t i = 0; i < expected.size(); ++i)
            compare(actual.at(i), expected.at(i), path + "/" + std::to_string(i));
    }
    else if (expected.is_object())
        for (auto it = expected.begin(); it != expected.end(); ++it)
            compare(actual.at(it.key()), it.value(), path + "/" + it.key());
    else if (actual != expected)
        throw std::runtime_error("Value regression: " + path);
}
void compare_state(const Json& actual, const Json& expected, const std::string& label)
{
    for (auto field : {"position", "velocity", "quaternion", "bg", "ba", "covariance_diagonal"})
        compare(actual.at(field), expected.at(field), label + "/" + field);
}
int main(int argc, char** argv)
{
    fs::path temporary =
        fs::temp_directory_path() /
        ("origami_regression_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        if (argc != 2)
            return 2;
        fs::path root = argv[1];
        auto fixture = read_json(root / "tests/fixtures/navigation.json");
        auto config = read_json(root / "config/real_imu.json");
        for (auto name : {"imu_100hz.txt", "imu_500hz.txt"})
        {
            origami::ImuProcess process(config);
            size_t count = 0;
            auto& reference = fixture.at(name);
            for (auto& sample : origami::read_recording((root / "data" / name).string()))
                for (auto& state : process.push(sample))
                {
                    auto key = std::to_string(count++);
                    if (!reference.at("checkpoints").contains(key))
                        continue;
                    auto& expected = reference.at("checkpoints").at(key);
                    for (auto chain : {"raw", "eskf"})
                        compare_state(state.at(chain), expected.at(chain),
                                      std::string(name) + "/" + key + "/" + chain);
                    compare(state.at("accepted"), expected.at("accepted"), key + "/accepted");
                }
            if (count != reference.at("samples").get<size_t>())
                throw std::runtime_error("Real frame count regression");
            process.eskf->check_covariance();
        }
        // Include MAT writing in the native regression so Python is not a test dependency.
        for (auto mode : {"estimated", "ground_truth"})
        {
            auto folder = temporary / mode;
            origami::run_simulation((root / "data/data_L_qawp_20261030.txt").string(),
                                    (root / "robot/zbot_crack_L2.xml").string(), config, mode,
                                    folder.string());
            auto& reference = fixture.at(mode);
            std::ifstream frames(folder / "frames.jsonl");
            std::string line;
            size_t count = 0;
            while (std::getline(frames, line))
            {
                auto key = std::to_string(count++);
                if (!reference.at("checkpoints").contains(key))
                    continue;
                auto actual = Json::parse(line);
                auto& expected = reference.at("checkpoints").at(key);
                for (auto field : {"shape", "quaternion"})
                    compare(actual.at(field), expected.at(field),
                            std::string(mode) + "/" + key + "/" + field);
                auto& ref = actual.at("references").at("S0");
                auto& old = expected.at("references").at("S0");
                compare_state(ref.at("raw"), old.at("raw"), key + "/raw");
                for (int p = 0; p < 3; ++p)
                {
                    compare_state(ref.at("policies")[p], old.at("policies")[p],
                                  key + "/policy" + std::to_string(p));
                    compare(ref.at("policies")[p].at("accepted"),
                            old.at("policies")[p].at("accepted"), key + "/accepted");
                }
            }
            if (count != reference.at("samples").get<size_t>())
                throw std::runtime_error("Simulation frame count regression");
            origami::export_mat(folder.string());
            std::ifstream mat(folder / "robot_odometry.mat", std::ios::binary);
            mat.seekg(126);
            char endian[2];
            mat.read(endian, 2);
            if (!mat || std::string(endian, 2) != "IM")
                throw std::runtime_error("Invalid MAT header");
        }
        fs::remove_all(temporary);
        std::cout << "Full-recording navigation regression passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        fs::remove_all(temporary);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
