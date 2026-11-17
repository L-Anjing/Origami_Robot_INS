#pragma once
#include "IMU_Processing.hpp"
namespace origami
{
Json run_simulation(const std::string& data, const std::string& model, const Json& config,
                    const std::string& mode, const std::string& output);
}
