#pragma once
#include "robot_model.h"
namespace origami
{
std::vector<std::string> generate_synthetic_motion(const Json&, const Model&,
                                                   const std::string& output);
}
