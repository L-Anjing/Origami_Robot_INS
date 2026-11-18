#pragma once
#include "multi_imu_fusion.h"
#include "simulation_data.h"
namespace origami
{
Json zero_noise_diagnostics(const Json&, const Model&, const std::vector<SimulationFrame>& original,
                            const std::vector<SimulationFrame>& consistent,
                            const std::string& output);
} // namespace origami
