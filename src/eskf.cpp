#include "eskf.hpp"
#include <algorithm>
namespace origami
{
void SixAxisAhrs::update(const V3D& omega, const V3D& force, double dt, const Json& c, double g,
                         bool allowed, bool gyroOnly)
{
    Quaterniond predicted = (q * increment(omega * dt)).normalized();
    double mag = force.norm();
    V3D direction = force / std::max(mag, 1e-12),
        up = predicted.toRotationMatrix().row(2).transpose();
    double angle = std::acos(std::clamp(direction.dot(up), -1., 1.));
    bool valid = allowed && std::abs(mag - g) <= c.at("force_tolerance_m_s2").get<double>() &&
                 angle <= c.at("innovation_limit_deg").get<double>() * pi / 180;
    reliable = valid ? reliable + dt : 0;
    used = valid && reliable + 1e-10 >= c.at("minimum_reliable_s").get<double>() && !gyroOnly;
    V3D correction = V3D::Zero();
    double gain = c.at("gain_s_inv");
    if (used)
        correction = gain * direction.cross(up);
    Quaterniond dq = increment(correction * dt);
    feedback.setIdentity();
    biasFeedback.setZero();
    if (used)
    {
        M3D jr = right_jacobian(correction * dt);
        feedback =
            dq.toRotationMatrix().transpose() + jr * (gain * skew(direction) * skew(up)) * dt;
        biasFeedback =
            jr * (gain * skew(up) * (M3D::Identity() - direction * direction.transpose()) / mag) *
            dt;
    }
    q = (predicted * dq).normalized();
}
ErrorStateKalmanFilter::ErrorStateKalmanFilter(const Json& c, bool cov, bool constant,
                                               bool prescribed, bool gyro)
    : c_(c), covariance_(cov), constant_(constant), prescribed_(prescribed), gyroOnly_(gyro),
      gravity_(c.at("gravity_m_s2"))
{
    if (!std::isfinite(gravity_) || gravity_ <= 0)
        throw std::invalid_argument("Invalid gravity");
    const auto& e = c_.at("robot").at("eskf");
    if (!e.at("covariance_check_every").is_number_integer() ||
        e.at("covariance_check_every").get<int>() < 1)
        throw std::invalid_argument("Invalid covariance check interval");
    for (const auto& group : {"ahrs", "quality"})
    {
        for (auto it = c_.at("robot").at(group).begin(); it != c_.at("robot").at(group).end(); ++it)
        {
            if (it.value().is_number())
            {
                double x = it.value();
                if (!std::isfinite(x) || x <= 0)
                    throw std::invalid_argument("Invalid AHRS or quality setting");
            }
        }
    }
    bg = vector3(e.at("initial_gyro_bias_rad_s"));
    ba = vector3(e.at("initial_accel_bias_m_s2"));
    for (auto group : {"initial_std", "noise_density"})
        for (auto it = e.at(group).begin(); it != e.at(group).end(); ++it)
        {
            double x = it.value();
            if (!std::isfinite(x) || x <= 0)
                throw std::invalid_argument("Invalid noise/prior");
        }
    const auto& s = e.at("initial_std");
    double stds[] = {s.at("position_m"), s.at("velocity_m_s"),
                     s.at("attitude_deg").get<double>() * pi / 180, s.at("gyro_bias_rad_s"),
                     s.at("accel_bias_m_s2")};
    if (cov)
        for (int k = 0; k < 5; ++k)
            P.block<3, 3>(3 * k, 3 * k) = M3D::Identity() * stds[k] * stds[k];
    if (prescribed)
    {
        P.block<6, 15>(6, 0).setZero();
        P.block<15, 6>(0, 6).setZero();
    }
    for (auto key : {"zupt_velocity_std_m_s", "zupt_nis_limit"})
    {
        double x = e.at(key);
        if (!std::isfinite(x) || x <= 0)
            throw std::invalid_argument("Invalid ZUPT setting");
    }
}
void ErrorStateKalmanFilter::propagate(const V3D& gyro, const V3D& force, double dt, bool allowed,
                                       double multiplier, const std::optional<Quaterniond>& debug)
{
    if (!gyro.allFinite() || !force.allFinite() || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(multiplier) || multiplier <= 0)
        throw std::invalid_argument("Invalid propagation input");
    if (prescribed_ != debug.has_value())
        throw std::invalid_argument("Debug quaternion mode mismatch");
    V3D omega = gyro - bg, f = force - ba;
    if (debug)
    {
        if (!debug->coeffs().allFinite() || debug->norm() < 1e-12)
            throw std::invalid_argument("Invalid quaternion");
        attitude.q = debug->normalized();
        attitude.used = false;
    }
    else
        attitude.update(omega, f, dt, c_.at("robot").at("ahrs"), gravity_, allowed, gyroOnly_);
    M3D R = attitude.q.toRotationMatrix();
    acceleration = R * f + V3D(0, 0, -gravity_);
    double factor = constant_ ? .5 : 1.;
    p += v * dt + factor * acceleration * dt * dt;
    v += acceleration * dt;
    if (!covariance_)
        return;
    M3D A = M3D::Zero(), B = M3D::Zero(), C = M3D::Zero(), ng = M3D::Zero(), na = M3D::Zero();
    if (!prescribed_)
    {
        M3D jr = right_jacobian(omega * dt);
        A = attitude.feedback * increment(omega * dt).toRotationMatrix().transpose();
        B = -attitude.feedback * jr * dt;
        C = attitude.biasFeedback;
        ng = -attitude.feedback * jr;
        na = C / dt;
    }
    M3D sensitivity = -R * skew(f);
    Matrix15d phi = Matrix15d::Identity();
    phi.block<3, 3>(0, 3) = M3D::Identity() * dt;
    phi.block<3, 3>(6, 6) = A;
    phi.block<3, 3>(6, 9) = B;
    phi.block<3, 3>(6, 12) = C;
    phi.block<3, 3>(3, 6) = sensitivity * A * dt;
    phi.block<3, 3>(3, 9) = sensitivity * B * dt;
    phi.block<3, 3>(3, 12) = (sensitivity * C - R) * dt;
    phi.block<3, 9>(0, 6) = factor * phi.block<3, 9>(3, 6) * dt;
    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
    G.block<3, 3>(6, 0) = ng;
    G.block<3, 3>(6, 3) = na;
    G.block<3, 3>(3, 0) = sensitivity * ng * dt;
    G.block<3, 3>(3, 3) = sensitivity * na * dt - R;
    G.block<3, 6>(0, 0) = factor * G.block<3, 6>(3, 0) * dt;
    G.block<3, 3>(9, 6).setIdentity();
    G.block<3, 3>(12, 9).setIdentity();
    if (prescribed_)
    {
        phi.block<6, 15>(6, 0).setZero();
        G.block<6, 12>(6, 0).setZero();
    }
    auto n = c_.at("robot").at("eskf").at("noise_density");
    double densities[] = {n.at("gyro_rad_s_sqrt_hz"), n.at("accel_m_s2_sqrt_hz"),
                          n.at("gyro_bias_random_walk"), n.at("accel_bias_random_walk")};
    Eigen::Matrix<double, 12, 1> d;
    for (int i = 0; i < 12; ++i)
        d[i] = densities[i / 3] * densities[i / 3] * dt * multiplier * multiplier;
    P = (phi * P * phi.transpose() + G * d.asDiagonal() * G.transpose()).eval();
    P = (.5 * (P + P.transpose())).eval();
    transition = phi;
    if (!P.allFinite())
        throw std::runtime_error("Nonfinite covariance");
}
Update ErrorStateKalmanFilter::zupt(bool candidate, bool valid, std::optional<double> limit)
{
    if (limit && (!std::isfinite(*limit) || *limit < 0))
        throw std::invalid_argument("Invalid position limit");
    Update out;
    if (!candidate || !valid || !covariance_)
        return out;
    const auto& c = c_.at("robot").at("eskf");
    double sigma = c.at("zupt_velocity_std_m_s");
    M3D r = M3D::Identity() * sigma * sigma, S = P.block<3, 3>(3, 3) + r;
    Eigen::LDLT<M3D> solve(S);
    if (solve.info() != Eigen::Success || !solve.isPositive())
        throw std::runtime_error("Invalid innovation covariance");
    V3D innovation = -v;
    out.nis = innovation.dot(solve.solve(innovation));
    if (!std::isfinite(out.nis) || out.nis > c.at("zupt_nis_limit").get<double>())
    {
        out.rejected = true;
        return out;
    }
    Eigen::Matrix<double, 15, 3> K = solve.solve(P.block<3, 15>(3, 0)).transpose();
    out.proposed = (K * innovation).head<3>().norm();
    out.positionRejected = limit && out.proposed > *limit;
    if (out.positionRejected)
        K.topRows<3>().setZero();
    Vector15d dx = K * innovation;
    Matrix15d ikh = Matrix15d::Identity();
    ikh.block<15, 3>(0, 3) -= K;
    Matrix15d posterior = ikh * P * ikh.transpose() + K * r * K.transpose();
    p += dx.segment<3>(0);
    v += dx.segment<3>(3);
    attitude.q = (attitude.q * increment(dx.segment<3>(6))).normalized();
    bg += dx.segment<3>(9);
    ba += dx.segment<3>(12);
    Matrix15d reset = Matrix15d::Identity();
    reset.block<3, 3>(6, 6) = right_jacobian(dx.segment<3>(6));
    P = reset * posterior * reset.transpose();
    P = (.5 * (P + P.transpose())).eval();
    injection = dx;
    out.accepted = true;
    return out;
}
Json ErrorStateKalmanFilter::snapshot() const
{
    auto q = attitude.q;
    return {{"position", array(p)},
            {"velocity", array(v)},
            {"quaternion", {q.w(), q.x(), q.y(), q.z()}},
            {"bg", array(bg)},
            {"ba", array(ba)},
            {"acceleration", array(acceleration)},
            {"covariance_diagonal", covariance_ ? array(P.diagonal()) : Json(nullptr)}};
}
void ErrorStateKalmanFilter::check_covariance() const
{
    Eigen::SelfAdjointEigenSolver<Matrix15d> e(P);
    if (e.info() != Eigen::Success ||
        e.eigenvalues()[0] <
            -c_.at("robot").at("eskf").at("covariance_negative_tolerance").get<double>())
        throw std::runtime_error("Covariance lost PSD");
}
} // namespace origami
