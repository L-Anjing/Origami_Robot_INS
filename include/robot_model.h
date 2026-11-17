#pragma once
#include "common_lib.h"
namespace origami
{
struct Model
{
    std::array<V3D, 6> offsets;
    Json json;
};
Model load_robot_model(const std::string&, const Json&);
} // namespace origami
