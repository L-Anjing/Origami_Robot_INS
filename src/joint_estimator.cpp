#include "joint_estimator.h"
#include <algorithm>
namespace origami
{
Calibration calibrate_initial_static(const std::vector<ImuMeasurements>& window,
                                     const ShapeState& initial, const Json& c)
{
    Calibration out;
    if (window.size() < 10)
    {
        out.reason = "Initial window requires ten samples";
        return out;
    }
    if (window.back().time - window.front().time + 1e-10 <
        c.at("calibration").at("duration_s").get<double>())
    {
        out.reason = "Initial static duration incomplete";
        return out;
    }
    const auto& gate = c.at("calibration");
    double g = c.at("gravity_m_s2");
    for (int i = 0; i < 6; ++i)
    {
        V3D force = V3D::Zero(), gyro = V3D::Zero();
        for (auto& s : window)
        {
            if (!s.gyro[i].allFinite() || !s.force[i].allFinite() ||
                s.gyro[i].norm() > gate.at("gyro_limit_rad_s").get<double>() ||
                std::abs(s.force[i].norm() - g) > gate.at("force_tolerance_m_s2").get<double>())
            {
                out.reason = "Initial IMU static gates fail at S" + std::to_string(i);
                return out;
            }
            force += s.force[i];
            gyro += s.gyro[i];
        }
        force /= window.size();
        gyro /= window.size();
        V3D sf = V3D::Zero(), sg = V3D::Zero();
        for (auto& s : window)
        {
            sf += (s.force[i] - force).cwiseAbs2();
            sg += (s.gyro[i] - gyro).cwiseAbs2();
        }
        sf = (sf / (window.size() - 1)).cwiseSqrt();
        sg = (sg / (window.size() - 1)).cwiseSqrt();
        if (sf.maxCoeff() > gate.at("accel_std_limit_m_s2").get<double>() ||
            sg.maxCoeff() > gate.at("gyro_std_limit_rad_s").get<double>())
        {
            out.reason = "Initial variance gates fail";
            return out;
        }
        out.gyro[i] = gyro;
        out.accel[i] = force - initial.rotation[i].conjugate() * V3D(0, 0, g);
    }
    out.available = true;
    out.reason = "Verified initial IMU window and configured attitude prior";
    return out;
}
JointEstimator::JointEstimator(const Model& model, const Json& c, const std::string& mode,
                               const Quaterniond& root, const Calibration& calibration)
    : model_(model), config_(c), mode_(mode), calibration_(calibration)
{
    if (mode != "A0" && mode != "A1" && mode != "A2")
        throw std::invalid_argument("Unknown joint mode");
    if (mode == "A1" && !calibration.available)
        throw std::invalid_argument("A1 requires verified initial static data");
    auto angles = c.at("initial_joint_angles_rad");
    if (angles.size() != 5)
        throw std::invalid_argument("Expected five joint priors");
    for (int j = 0; j < 5; ++j)
        state_.angle[j] = angles[j];
    auto initial = forward_shape(model, root, state_.angle);
    for (int i = 0; i < 6; ++i)
        attitude_[i].q = initial.rotation[i];
}
JointEstimate JointEstimator::update(const ImuMeasurements& input, double dt)
{
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid joint dt");
    elapsed_ += dt;
    for (int i = 0; i < 6; ++i)
    {
        V3D bias = calibration_.available ? calibration_.gyro[i] : V3D::Zero(),
            abias = calibration_.available ? calibration_.accel[i] : V3D::Zero();
        bool aid = std::abs(input.force[i].norm() - config_.at("gravity_m_s2").get<double>()) <
                   config_.at("robot").at("ahrs").at("force_tolerance_m_s2").get<double>();
        attitude_[i].update(input.gyro[i] - bias, input.force[i] - abias, dt,
                            config_.at("robot").at("ahrs"), config_.at("gravity_m_s2"), aid,
                            !config_.at("joint").value("relative_ahrs_feedback", false));
    }
    bool density = config_.at("noise").value("white_noise_definition",
                                             std::string("per_sample_std")) == "density";
    double sigma =
        config_.at("noise")
            .at(density ? "gyro_white_density_deg_s_per_sqrt_hz" : "gyro_white_std_deg_s")
            .get<double>() *
        pi / 180;
    if (density)
        sigma /= std::sqrt(dt);
    double b = config_.at("noise").at("gyro_initial_bias_bound_deg_s").get<double>() * pi / 180 /
               std::sqrt(3.),
           rw = config_.at("noise").at("gyro_bias_rw_deg_s_sqrt_s").get<double>() * pi / 180;
    for (int j = 0; j < 5; ++j)
    {
        const auto& h = model_.joints[j];
        int p = h.parent, ch = h.child;
        V3D wp = input.gyro[p] - (calibration_.available ? calibration_.gyro[p] : V3D::Zero()),
            wc = input.gyro[ch] - (calibration_.available ? calibration_.gyro[ch] : V3D::Zero());
        state_.rate[j] = h.axis_child.dot(wc) - h.axis_parent.dot(wp);
        state_.angle[j] += state_.rate[j] * dt;
        Quaterniond relative = attitude_[p].q.conjugate() * attitude_[ch].q,
                    expected =
                        h.rest_rotation *
                        Quaterniond(Eigen::AngleAxisd(state_.angle[j] - h.reference, h.axis_child));
        state_.axis_residual[j] =
            (wc - expected.conjugate() * wp - h.axis_child * state_.rate[j]).norm();
        state_.orientation_residual[j] = relative.angularDistance(expected);
        state_.valid[j] =
            std::isfinite(state_.angle[j]) &&
            state_.axis_residual[j] <=
                config_.at("joint").at("axis_residual_limit_rad_s").get<double>() &&
            state_.orientation_residual[j] <=
                config_.at("joint").at("orientation_residual_limit_rad").get<double>();
        if (mode_ == "A2")
        {
            if (state_.valid[j])
            {
                double observed = joint_angle_from_relative(h, relative, state_.angle[j]),
                       alpha = 1 - std::exp(-config_.at("joint")
                                                 .at("relative_orientation_gain_s_inv")
                                                 .get<double>() *
                                            dt);
                state_.angle[j] += alpha * (observed - state_.angle[j]);
            }
            double slack = config_.at("joint").at("soft_limit_tolerance_rad");
            if (h.limited &&
                (state_.angle[j] < h.lower - slack || state_.angle[j] > h.upper + slack))
            {
                state_.valid[j] = false;
                state_.angle[j] = std::clamp(state_.angle[j], h.lower - slack, h.upper + slack);
            }
        }
        state_.variance[j] = 2 * sigma * sigma * dt * elapsed_ +
                             (calibration_.available ? 0 : 2 * b * b * elapsed_ * elapsed_) +
                             2 * rw * rw * elapsed_ * elapsed_ * elapsed_ / 3;
    }
    return state_;
}
} // namespace origami
