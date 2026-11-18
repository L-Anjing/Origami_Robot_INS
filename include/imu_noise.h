#pragma once
#include "common_lib.h"
#include <random>
namespace origami
{
// Sensor-only interface: no quaternions, endpoints or bias truth are available to an estimator.
struct ImuMeasurements
{
    ImuMeasurements()
    {
        gyro.fill(V3D::Zero());
        force.fill(V3D::Zero());
    }
    double time = 0;
    std::array<V3D, 6> gyro{}, force{};
};
struct BiasTruth
{
    BiasTruth()
    {
        gyro.fill(V3D::Zero());
        accel.fill(V3D::Zero());
    }
    std::array<V3D, 6> gyro{}, accel{};
};
struct NoiseOutput
{
    ImuMeasurements measured;
    BiasTruth truth;
};
class ImuNoise
{
    Json config_;
    std::string mode_;
    std::mt19937_64 engine_;
    BiasTruth bias_, common_bias_;
    std::mt19937_64 common_engine_;
    double common_correlation_ = 0;
    double normal();
    double uniform();

  public:
    ImuNoise(const Json&, const std::string&);
    NoiseOutput apply(const ImuMeasurements&, double dt);
};
} // namespace origami
