#pragma once
#include "imu_noise.h"
namespace origami
{
struct SimulationTruth
{
    std::array<Quaterniond, 6> orientation;
    std::array<V3D, 7> markers;
};
struct SimulationFrame
{
    ImuMeasurements imu;
    std::array<int, 3> controls;
    SimulationTruth truth;
};
std::vector<SimulationFrame> read_simulation(const std::string&, double sample_rate);
} // namespace origami
