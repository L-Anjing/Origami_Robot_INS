#pragma once
#include "eskf.hpp"
#include "preprocess.h"
namespace origami
{
class ImuProcess
{
    Json c_;
    double initial_;
    bool zupt_;
    std::vector<Sample> pending_;
    std::deque<double> intervals_;
    std::unique_ptr<StationaryDetector> detector_;
    std::optional<int64_t> lastInput_, previous_, lastAttempt_;
    V3D initialBg_ = V3D::Zero(), initialBa_ = V3D::Zero();
    int anomalies_ = 0, positionRejections_ = 0, count_ = 0;
    Json step(const Sample& s);
    void initialize();

  public:
    std::unique_ptr<ErrorStateKalmanFilter> raw, eskf;
    ImuProcess(Json c, double initial = 2, bool zupt = true);
    std::vector<Json> push(const Sample& s);
    void set_stationary_timing(double window, double minimum);
};
} // namespace origami
