#include "static_calibration.h"
namespace origami
{
CalibrationSequence make_calibration_sequence(const Json& config, const std::string& mode,
                                              const std::vector<SimulationFrame>& frames,
                                              size_t count)
{
    double dt = 1 / config.at("sample_rate_hz").get<double>();
    ImuNoise noise(config.at("noise"), mode);
    CalibrationSequence out;
    out.stationary.reserve(count);
    out.stationary_truth.reserve(count);
    out.dynamic.reserve(frames.size());
    for (size_t k = 0; k < count; ++k)
    {
        ImuMeasurements ideal;
        ideal.time = (double(k) + 1 - count) * dt;
        ideal.force.fill(V3D(0, 0, config.at("gravity_m_s2")));
        auto sample = noise.apply(ideal, dt);
        out.stationary.push_back(sample.measured);
        out.stationary_truth.push_back(sample.truth);
        out.boundary_truth = sample.truth;
    }
    // Reuse the same generator: fixed biases, random walk and white-noise draws stay continuous.
    // The requested calibration duration never changes this precomputed dynamic sequence.
    for (const auto& frame : frames)
        out.dynamic.push_back(noise.apply(frame.imu, dt));
    return out;
}
Calibration estimate_stationary_bias(const std::vector<ImuMeasurements>& window,
                                     const ShapeState& initial, const Json& config, double dt)
{
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid calibration dt");
    for (size_t k = 1; k < window.size(); ++k)
        if (std::abs(window[k].time - window[k - 1].time - dt) > 1e-9)
            throw std::invalid_argument("Calibration window must have a uniform time grid");
    Json c = config;
    // N samples cover N step-end intervals; their first-to-last timestamp span is (N-1)dt.
    c["calibration"]["duration_s"] = (window.empty() ? 0 : window.size() - 1) * dt;
    return calibrate_initial_static(window, initial, c);
}
Calibration calibration_initialization(const Calibration& estimate, const std::string& strategy)
{
    Calibration out;
    out.gyro.fill(V3D::Zero());
    out.accel.fill(V3D::Zero());
    if (strategy == "未标定")
        return out;
    if (strategy != "仅陀螺标定" && strategy != "陀螺与加速度标定")
        throw std::invalid_argument("Normal initialization accepts sensor calibration only");
    if (!estimate.available)
        throw std::invalid_argument("Rejected stationary calibration cannot initialize INS");
    out = estimate;
    if (strategy == "仅陀螺标定")
        out.accel.fill(V3D::Zero());
    return out;
}
void apply_initial_bias(std::vector<ErrorStateKalmanFilter>& nodes, const Calibration& initial)
{
    if (!initial.available)
        return;
    if (nodes.size() != 6)
        throw std::invalid_argument("Expected six IMUs for bias initialization");
    for (int i = 0; i < 6; ++i)
    {
        if (!initial.gyro[i].allFinite() || !initial.accel[i].allFinite())
            throw std::invalid_argument("Nonfinite initial bias estimate");
        nodes[i].bg = initial.gyro[i];
        nodes[i].ba = initial.accel[i];
    }
}
double mean_bias_error_variance(double white, double walk, size_t count, double dt)
{
    if (!count || !std::isfinite(white) || !std::isfinite(walk) || !std::isfinite(dt) ||
        white < 0 || walk < 0 || dt <= 0)
        throw std::invalid_argument("Invalid calibration variance arguments");
    double n = count;
    // Error of a sample mean relative to the bias at its final sample, not its initial bias.
    return white * white / n + walk * walk * dt * (n - 1) * (2 * n - 1) / (6 * n);
}
} // namespace origami
