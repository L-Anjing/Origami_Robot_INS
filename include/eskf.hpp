#pragma once
#include "so3_math.h"
namespace origami
{
struct SixAxisAhrs
{
    Quaterniond q = Quaterniond::Identity();
    M3D feedback = M3D::Identity(), biasFeedback = M3D::Zero();
    double reliable = 0;
    bool used = false;
    void update(const V3D& omega, const V3D& force, double dt, const Json& c, double gravity,
                bool allowed, bool gyroOnly = false);
};
struct Update
{
    bool accepted = false, rejected = false, positionRejected = false;
    double nis = 0, proposed = 0;
};
class ErrorStateKalmanFilter
{
  public:
    V3D p = V3D::Zero(), v = V3D::Zero(), bg = V3D::Zero(), ba = V3D::Zero(),
        acceleration = V3D::Zero();
    SixAxisAhrs attitude;
    Matrix15d P = Matrix15d::Zero(), transition = Matrix15d::Identity();
    Vector15d injection = Vector15d::Zero();
    ErrorStateKalmanFilter(const Json& config, bool covariance = true,
                           bool constantAcceleration = false, bool prescribed = false,
                           bool gyroOnly = false);
    void propagate(const V3D& gyro, const V3D& force, double dt, bool allowed = true,
                   double noiseMultiplier = 1,
                   const std::optional<Quaterniond>& debug = std::nullopt);
    Update zupt(bool candidate, bool valid = true,
                std::optional<double> maxPosition = std::nullopt);
    Json snapshot() const;
    void check_covariance() const;

  private:
    Json c_;
    bool covariance_, constant_, prescribed_, gyroOnly_;
    double gravity_;
};
} // namespace origami
