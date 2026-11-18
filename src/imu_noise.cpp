#include "imu_noise.h"
namespace origami
{
double ImuNoise::uniform()
{
    return (double(engine_() >> 11) + .5) * (1. / 9007199254740992.);
}
double ImuNoise::normal()
{
    double u = uniform(), v = uniform();
    return std::sqrt(-2 * std::log(u)) * std::cos(2 * pi * v);
}
ImuNoise::ImuNoise(const Json& config, const std::string& mode)
    : config_(config), mode_(mode), engine_(config.at("seed").get<uint64_t>())
{
    if (mode != "none" && mode != "white" && mode != "fixed_bias" && mode != "full")
        throw std::invalid_argument("Unknown noise mode");
    for (auto key : {"gyro_white_std_deg_s", "accel_white_std_m_s2",
                     "gyro_initial_bias_bound_deg_s", "accel_initial_bias_bound_m_s2",
                     "gyro_bias_rw_deg_s_sqrt_s", "accel_bias_rw_m_s2_sqrt_s"})
    {
        double x = config.at(key);
        if (!std::isfinite(x) || x < 0)
            throw std::invalid_argument("Invalid noise scale");
    }
    auto definition = config.value("white_noise_definition", std::string("per_sample_std"));
    if (definition != "per_sample_std" && definition != "density")
        throw std::invalid_argument("Invalid white noise convention");
    if (definition == "density")
        for (auto key :
             {"gyro_white_density_deg_s_per_sqrt_hz", "accel_white_density_m_s2_per_sqrt_hz"})
        {
            double x = config.at(key);
            if (!std::isfinite(x) || x < 0)
                throw std::invalid_argument("Invalid continuous white noise density");
        }
    common_correlation_ = config.value("common_bias_correlation", 0.);
    if (common_correlation_ < 0 || common_correlation_ > 1)
        throw std::invalid_argument("Invalid common bias correlation");
    common_engine_.seed(config.at("seed").get<uint64_t>() ^ 0x9e3779b97f4a7c15ULL);
    if (common_correlation_ > 0)
        for (int k = 0; k < 3; ++k)
        {
            auto uniform_common = [&]()
            { return (double(common_engine_() >> 11) + .5) / 9007199254740992.; };
            common_bias_.gyro[0][k] = (2 * uniform_common() - 1) *
                                      config.at("gyro_initial_bias_bound_deg_s").get<double>() *
                                      pi / 180;
            common_bias_.accel[0][k] = (2 * uniform_common() - 1) *
                                       config.at("accel_initial_bias_bound_m_s2").get<double>();
        }
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 3; ++k)
        {
            bias_.gyro[i][k] = (2 * uniform() - 1) *
                               config.at("gyro_initial_bias_bound_deg_s").get<double>() * pi / 180;
            bias_.accel[i][k] =
                (2 * uniform() - 1) * config.at("accel_initial_bias_bound_m_s2").get<double>();
        }
}
NoiseOutput ImuNoise::apply(const ImuMeasurements& input, double dt)
{
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid noise dt");
    NoiseOutput out;
    out.measured = input;
    double scale =
        config_.value("white_noise_definition", std::string("per_sample_std")) == "density"
            ? 1 / std::sqrt(dt)
            : 1;
    if (common_correlation_ > 0)
        for (int k = 0; k < 3; ++k)
        {
            auto common_normal = [&]()
            {
                double u = (double(common_engine_() >> 11) + .5) / 9007199254740992.,
                       v = (double(common_engine_() >> 11) + .5) / 9007199254740992.;
                return std::sqrt(-2 * std::log(u)) * std::cos(2 * pi * v);
            };
            double g = common_normal(), a = common_normal();
            if (mode_ == "full")
            {
                common_bias_.gyro[0][k] += config_.at("gyro_bias_rw_deg_s_sqrt_s").get<double>() *
                                           pi / 180 * std::sqrt(dt) * g;
                common_bias_.accel[0][k] +=
                    config_.at("accel_bias_rw_m_s2_sqrt_s").get<double>() * std::sqrt(dt) * a;
            }
        }
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 3; ++k)
        {
            if (!input.gyro[i].allFinite() || !input.force[i].allFinite())
                throw std::invalid_argument("Nonfinite input");
            // Every ablation consumes the same draw sequence, including disabled components.
            double ng = normal(), na = normal(), rg = normal(), ra = normal();
            if (mode_ == "full")
            {
                bias_.gyro[i][k] += config_.at("gyro_bias_rw_deg_s_sqrt_s").get<double>() * pi /
                                    180 * std::sqrt(dt) * rg;
                bias_.accel[i][k] +=
                    config_.at("accel_bias_rw_m_s2_sqrt_s").get<double>() * std::sqrt(dt) * ra;
            }
            bool biased = mode_ == "fixed_bias" || mode_ == "full";
            out.truth.gyro[i][k] =
                biased ? std::sqrt(1 - common_correlation_) * bias_.gyro[i][k] +
                             std::sqrt(common_correlation_) * common_bias_.gyro[0][k]
                       : 0;
            out.truth.accel[i][k] =
                biased ? std::sqrt(1 - common_correlation_) * bias_.accel[i][k] +
                             std::sqrt(common_correlation_) * common_bias_.accel[0][k]
                       : 0;
            out.measured.gyro[i][k] += out.truth.gyro[i][k];
            out.measured.force[i][k] += out.truth.accel[i][k];
            if (mode_ != "none")
            {
                out.measured.gyro[i][k] +=
                    scale *
                    config_
                        .at(config_.value("white_noise_definition",
                                          std::string("per_sample_std")) == "density"
                                ? "gyro_white_density_deg_s_per_sqrt_hz"
                                : "gyro_white_std_deg_s")
                        .get<double>() *
                    pi / 180 * ng;
                out.measured.force[i][k] +=
                    scale *
                    config_
                        .at(config_.value("white_noise_definition",
                                          std::string("per_sample_std")) == "density"
                                ? "accel_white_density_m_s2_per_sqrt_hz"
                                : "accel_white_std_m_s2")
                        .get<double>() *
                    na;
            }
        }
    return out;
}
} // namespace origami
