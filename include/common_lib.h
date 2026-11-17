#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
namespace origami
{
using Json = nlohmann::json;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Quaterniond = Eigen::Quaterniond;
constexpr double pi = 3.14159265358979323846;
struct Sample
{
    int64_t stamp = 0;
    V3D force = V3D::Zero(), gyro = V3D::Zero();
    Quaterniond device = Quaterniond::Identity();
};
V3D vector3(const Json& value);
Json array(const Eigen::MatrixXd& value);
} // namespace origami
