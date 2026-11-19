#pragma once
#include "imu_noise.h"
#include "kinematic_constraints.h"
namespace origami
{
// imu_count selects S0..S(N-1); legacy explicit subsets remain supported.
Json normalize_multi_imu_config(const Json&);
struct InnovationRecord
{
    double time = 0, nis = 0, correction_norm = 0, bias_change_norm = 0;
    std::string kind;
    int joint = -1;
    bool accepted = false;
    Eigen::VectorXd innovation;
};
struct JointRuntime
{
    double propagation_s = 0, position_update_s = 0, axis_update_s = 0, total_update_s = 0;
    size_t propagation_count = 0, update_count = 0;
};
class MultiImuEskf
{
    Json config_;
    Model model_;
    std::array<bool, 6> selected_{};
    bool shape_state_;
    size_t sample_ = 0;
    int node_count_ = 6;
    Eigen::MatrixXd phi_, process_noise_;
    void inject(const Eigen::VectorXd&);

  public:
    std::vector<ErrorStateKalmanFilter> nodes;
    Eigen::MatrixXd P;
    std::array<double, 5> angles{};
    JointRuntime runtime;
    std::vector<InnovationRecord> last_innovations;
    Eigen::VectorXd last_correction;
    explicit MultiImuEskf(const Json&, const Model&);
    void propagate(const ImuMeasurements&, double dt);
    void update(double time);
    const Eigen::MatrixXd& last_transition() const
    {
        return phi_;
    }
    const Eigen::MatrixXd& last_process_noise() const
    {
        return process_noise_;
    }
    Json diagnostics() const;
    bool active(int i) const
    {
        return selected_.at(i);
    }
    bool shape_state() const
    {
        return shape_state_;
    }
};
} // namespace origami
