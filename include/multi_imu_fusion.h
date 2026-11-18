#pragma once
#include "joint_estimator.h"
namespace origami
{
std::array<V3D, 6> initial_node_velocities(const Model&, const ShapeState&,
                                           const std::array<double, 5>& rates,
                                           const V3D& root_omega, const V3D& root_velocity);
struct FusionResult
{
    std::array<V3D, 6> compensated;
    std::array<double, 6> weights{}, uncertainty_proxy_m2{};
    V3D single = V3D::Zero(), uniform = V3D::Zero(), weighted = V3D::Zero();
    // Unknown-correlation upper bound proxy; not an independent-information posterior covariance.
    double weighted_trace_bound_m2 = 0;
};
FusionResult fuse_positions(const std::array<V3D, 6>& positions,
                            const std::array<Matrix15d, 6>& covariance, const Model&,
                            const ShapeState&, const JointEstimate&, const Json& config);
} // namespace origami
