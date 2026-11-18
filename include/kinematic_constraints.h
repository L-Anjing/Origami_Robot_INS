#pragma once
#include "eskf.hpp"
#include "robot_model.h"
#include "shape_kinematics.h"
namespace origami
{
struct Constraint
{
    std::string kind;
    int joint = -1;
    Eigen::VectorXd value;
    Eigen::MatrixXd H;
    Eigen::Matrix<double, 3, 2> tangent = Eigen::Matrix<double, 3, 2>::Zero();
};
V3D rotation_log(const Quaterniond&);
M3D inverse_right_jacobian(const V3D&);
Constraint hinge_position(const std::vector<ErrorStateKalmanFilter>&, const JointModel&, int joint,
                          int dimension);
Constraint hinge_axis(const std::vector<ErrorStateKalmanFilter>&, const JointModel&, int joint,
                      int dimension, const Eigen::Matrix<double, 3, 2>* frozen_tangent = nullptr);
std::vector<Constraint> shape_constraints(const std::vector<ErrorStateKalmanFilter>&, const Model&,
                                          const std::array<double, 5>& angles, int dimension);
} // namespace origami
