#pragma once
#include "common_lib.h"
namespace origami
{
Json run_multi_imu_experiment(const Json&, const std::string& data, const std::string& output,
                              const std::string& model, bool suite);
}
