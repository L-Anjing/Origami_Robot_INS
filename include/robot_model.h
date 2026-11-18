#pragma once
#include "common_lib.h"
namespace origami
{
struct JointModel
{
    std::string name;
    int parent = 0, child = 1;
    V3D axis_child = V3D::UnitZ(), axis_parent = V3D::UnitZ();
    V3D translation = V3D::Zero(), anchor_child = V3D::Zero();
    Quaterniond rest_rotation = Quaterniond::Identity();
    double lower = -pi, upper = pi;
    double reference = 0;
    bool limited = false;
};
struct Model
{
    std::array<JointModel, 5> joints;
    double physical_length_m = .05;

    std::array<V3D, 6> offsets;
    Json json;
};
Model load_robot_model(const std::string&, const Json&);
} // namespace origami
