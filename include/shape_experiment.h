#pragma once
#include "common_lib.h"
namespace origami
{
Json run_shape_experiment(const Json&, const std::string& data, const std::string& output,
                          const std::string& model, bool shape_only = false);
}
