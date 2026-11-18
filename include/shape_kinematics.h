#pragma once
#include "robot_model.h"
namespace origami
{
struct ShapeState
{
    std::array<Quaterniond, 6> rotation;
    std::array<V3D, 6> position;
    V3D terminal = V3D::Zero(), physical_tip = V3D::Zero();
};
ShapeState forward_shape(const Model&, const Quaterniond& root,
                         const std::array<double, 5>& angles);
// For relative sensor attitude or isolated evaluation; never reads world position truth.
double joint_angle_from_relative(const JointModel&, const Quaterniond& relative, double reference);
} // namespace origami
