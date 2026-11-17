#include "IMU_Processing.hpp"
#include <algorithm>
namespace origami
{
ImuProcess::ImuProcess(Json c, double initial, bool zupt)
    : c_(std::move(c)), initial_(initial), zupt_(zupt)
{
    if (!std::isfinite(initial) || initial <= 0)
        throw std::invalid_argument("Invalid alignment duration");
}
std::vector<Json> ImuProcess::push(const Sample& s)
{
    finite(s);
    if (s.stamp < 0)
        throw std::invalid_argument("Negative timestamp");
    if (lastInput_ && s.stamp <= *lastInput_)
        throw std::invalid_argument("Timestamps must increase");
    double maxGap = c_.value("real_imu", Json::object()).value("max_integration_interval_s", .25);
    if (!std::isfinite(maxGap) || maxGap <= 0)
        throw std::invalid_argument("Invalid maximum interval");
    if (lastInput_ && (s.stamp - *lastInput_) * 1e-9 > maxGap)
        throw std::invalid_argument("Timestamp gap exceeds integration limit; reset required");
    lastInput_ = s.stamp;
    if (!eskf)
    {
        pending_.push_back(s);
        if ((s.stamp - pending_[0].stamp) * 1e-9 < initial_)
            return {};
        initialize();
        std::vector<Json> results;
        for (const auto& a : pending_)
            results.push_back(step(a));
        pending_.clear();
        return results;
    }
    return {step(s)};
}
void ImuProcess::initialize()
{
    double g = c_.at("gravity_m_s2");
    auto d = c_.at("robot").at("stationary");
    std::vector<Sample> calibration;
    for (auto& s : pending_)
        if (s.stamp - pending_[0].stamp <= std::llround(initial_ * 1e9))
            calibration.push_back(s);
    if (calibration.size() < 10)
        throw std::invalid_argument("Alignment needs 10 samples");
    V3D mean = V3D::Zero(), bg = V3D::Zero();
    for (auto& s : calibration)
    {
        mean += s.force;
        bg += s.gyro;
    }
    mean /= calibration.size();
    bg /= calibration.size();
    V3D fv = V3D::Zero(), gv = V3D::Zero();
    for (auto& s : calibration)
    {
        fv += (s.force - mean).cwiseAbs2();
        gv += (s.gyro - bg).cwiseAbs2();
    }
    fv = (fv / (calibration.size() - 1)).cwiseSqrt();
    gv = (gv / (calibration.size() - 1)).cwiseSqrt();
    if (std::abs(mean.norm() - g) > d.at("force_tolerance_m_s2").get<double>() ||
        fv.maxCoeff() >= d.at("sigma_accel_m_s2").get<double>() ||
        gv.maxCoeff() >= d.at("sigma_gyro_rad_s").get<double>() ||
        bg.norm() >= d.at("gyro_limit_rad_s").get<double>())
        throw std::invalid_argument("Initial interval is not static");
    for (size_t i = 1; i < calibration.size(); ++i)
    {
        intervals_.push_back((calibration[i].stamp - calibration[i - 1].stamp) * 1e-9);
        if (intervals_.size() > 100)
            intervals_.pop_front();
    }
    if (d.at("window_s").get<double>() < 2 * median(intervals_))
        throw std::invalid_argument("StationaryDetector window too short");
    V3D up = mean.normalized();
    double roll = std::atan2(up.y(), up.z()),
           pitch = std::atan2(-up.x(), std::hypot(up.y(), up.z()));
    Quaterniond q = Eigen::AngleAxisd(pitch, V3D::UnitY()) * Eigen::AngleAxisd(roll, V3D::UnitX());
    initialBg_ = bg;
    initialBa_ = mean - g * up;
    c_["robot"]["eskf"]["initial_gyro_bias_rad_s"] = array(bg);
    c_["robot"]["eskf"]["initial_accel_bias_m_s2"] = array(initialBa_);
    raw = std::make_unique<ErrorStateKalmanFilter>(c_, false, true);
    eskf = std::make_unique<ErrorStateKalmanFilter>(c_, true, true);
    raw->attitude.q = eskf->attitude.q = q;
    detector_ = std::make_unique<StationaryDetector>(d, g, median(intervals_), true);
}
Json ImuProcess::step(const Sample& s)
{
    auto& c = c_.at("robot");
    double g = c_.at("gravity_m_s2"), dt = previous_ ? (s.stamp - *previous_) * 1e-9 : 0,
           gapLimit = 5 * median(intervals_);
    Sample corrected = s;
    corrected.force -= initialBa_;
    corrected.gyro -= initialBg_;
    bool valid = quality(s, c.at("quality"), g, corrected.force), gap = dt > gapLimit;
    auto detection = detector_->update(corrected, valid, dt, gapLimit);
    V3D before = eskf->p, predicted = before;
    Update update;
    bool requested = false;
    auto real = c_.value("real_imu", Json::object()), health = real.value("health", Json::object());
    double speed = health.value("max_speed_m_s", 2.),
           tolerance = health.value("step_tolerance_m", .02),
           maxCorrection = health.value("max_position_correction_m", .10);
    double budget = tolerance + speed * dt;
    if (previous_)
    {
        intervals_.push_back(dt);
        if (intervals_.size() > 100)
            intervals_.pop_front();
        raw->propagate(s.gyro, s.force, dt, valid);
        eskf->propagate(s.gyro, s.force, dt, valid,
                        valid ? 1 : c.at("quality").at("impact_noise_multiplier").get<double>());
        predicted = eskf->p;
        requested = zupt_ && detection.stationary &&
                    (!lastAttempt_ ||
                     s.stamp - *lastAttempt_ >=
                         std::llround(c.at("eskf").at("zupt_min_interval_s").get<double>() * 1e9));
        if (requested)
        {
            lastAttempt_ = s.stamp;
            std::optional<double> limit;
            if (real.value("protect_position_updates", false))
                limit = std::min(maxCorrection, std::max(0., budget - (predicted - before).norm()));
            update = eskf->zupt(true, valid, limit);
        }
    }
    double stepSize = previous_ ? (eskf->p - before).norm() : 0,
           motion = previous_ ? (predicted - before).norm() : 0,
           correction = (eskf->p - predicted).norm();
    bool broken = previous_ && (stepSize > budget || correction > maxCorrection),
         excess = previous_ && motion > budget;
    if (broken || excess)
        ++anomalies_;
    if (update.positionRejected)
        ++positionRejections_;
    Eigen::SelfAdjointEigenSolver<M3D> eigen(eskf->P.topLeftCorner<3, 3>());
    double sigma = std::sqrt(std::max(0., eigen.eigenvalues().maxCoeff()));
    Json h = {{"position_step_m", stepSize},
              {"propagated_step_m", motion},
              {"position_correction_m", correction},
              {"position_std_m", sigma},
              {"position_discontinuity", broken},
              {"excessive_propagation", excess},
              {"continuity_lost", anomalies_ > 0},
              {"anomaly_count", anomalies_},
              {"position_update_rejected", update.positionRejected},
              {"position_update_rejections", positionRejections_},
              {"estimate_unreliable", anomalies_ > 0 || positionRejections_ > 0},
              {"position_uncertain", sigma > health.value("max_position_std_m", .5)}};
    previous_ = s.stamp;
    if (++count_ % c.at("eskf").at("covariance_check_every").get<int>() == 0)
        eskf->check_covariance();
    return {
        {"timestamp_ns", s.stamp},
        {"raw", raw->snapshot()},
        {"eskf", eskf->snapshot()},
        {"covariance", array(eskf->P)},
        {"stationary", detection.stationary},
        {"glrt", std::isfinite(detection.statistic) ? Json(detection.statistic) : Json(nullptr)},
        {"valid", valid},
        {"gap", gap},
        {"zupt_requested", requested},
        {"accepted", update.accepted},
        {"rejected_nis", update.rejected},
        {"nis", update.nis},
        {"health", h}};
}
void ImuProcess::set_stationary_timing(double window, double minimum)
{
    if (!std::isfinite(window) || window <= 0 || !std::isfinite(minimum) || minimum <= 0)
        throw std::invalid_argument("Stationary timings must be positive and finite");
    auto intervals = intervals_;
    if (intervals.empty())
        for (size_t i = 1; i < pending_.size(); ++i)
            intervals.push_back((pending_[i].stamp - pending_[i - 1].stamp) * 1e-9);
    double dt = intervals.empty() ? .001 : median(intervals);
    if (!intervals.empty() && window < 2 * dt)
        throw std::invalid_argument("Stationary window must cover two sample intervals");
    auto candidate = c_.at("robot").at("stationary");
    candidate["window_s"] = window;
    candidate["minimum_stationary_s"] = minimum;
    auto detector =
        std::make_unique<StationaryDetector>(candidate, c_.at("gravity_m_s2"), dt, true);
    c_["robot"]["stationary"] = candidate;
    if (eskf)
        detector_ = std::move(detector);
}
} // namespace origami
