#pragma once
#include "eskf.hpp"
#include "imu_noise.h"
#include "shape_kinematics.h"
namespace origami
{
struct Calibration
{
    bool available = false;
    std::string reason;
    std::array<V3D, 6> gyro{}, accel{};
};
Calibration calibrate_initial_static(const std::vector<ImuMeasurements>&, const ShapeState&,
                                     const Json&);
struct JointEstimate
{
    std::array<double, 5> angle{}, rate{}, variance{}, axis_residual{}, orientation_residual{};
    std::array<bool, 5> valid{};
};
class JointEstimator
{
    Model model_;
    Json config_;
    std::string mode_;
    std::array<SixAxisAhrs, 6> attitude_;
    Calibration calibration_;
    JointEstimate state_;
    double elapsed_ = 0;

  public:
    JointEstimator(const Model&, const Json&, const std::string& mode, const Quaterniond& root,
                   const Calibration& calibration = {});
    JointEstimate update(const ImuMeasurements&, double dt);
};
} // namespace origami
