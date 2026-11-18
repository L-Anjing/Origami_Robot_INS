#pragma once
#include "joint_estimator.h"
#include "simulation_data.h"
namespace origami
{
// Sensor windows are separate from evaluation-only bias truth and dynamic frame truth.
struct CalibrationSequence
{
    std::vector<ImuMeasurements> stationary;
    std::vector<BiasTruth> stationary_truth;
    std::vector<NoiseOutput> dynamic;
    BiasTruth boundary_truth;
};
CalibrationSequence make_calibration_sequence(const Json&, const std::string& noise_mode,
                                              const std::vector<SimulationFrame>&,
                                              size_t stationary_samples = 5000);
Calibration estimate_stationary_bias(const std::vector<ImuMeasurements>&, const ShapeState&,
                                     const Json&, double dt);
Calibration calibration_initialization(const Calibration&, const std::string& strategy);
void apply_initial_bias(std::vector<ErrorStateKalmanFilter>&, const Calibration&);
double mean_bias_error_variance(double white_std, double walk_density, size_t samples, double dt);
} // namespace origami
