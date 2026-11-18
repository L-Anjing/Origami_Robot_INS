#include "multi_imu_fusion.h"
namespace origami
{
std::array<V3D, 6> initial_node_velocities(const Model& model, const ShapeState& shape,
                                           const std::array<double, 5>& rates,
                                           const V3D& root_omega, const V3D& root_velocity)
{
    std::array<V3D, 6> velocity, omega;
    velocity[0] = root_velocity;
    omega[0] = root_omega;
    for (int j = 0; j < 5; ++j)
    {
        const auto& h = model.joints[j];
        int p = h.parent, ch = h.child;
        V3D axis_w = shape.rotation[p] * h.axis_parent;
        omega[ch] = omega[p] + axis_w * rates[j];
        V3D motion = -(shape.rotation[ch] * (h.axis_child.cross(h.anchor_child))) * rates[j];
        velocity[ch] =
            velocity[p] + omega[p].cross(shape.position[ch] - shape.position[p]) + motion;
    }
    return velocity;
}
FusionResult fuse_positions(const std::array<V3D, 6>& positions,
                            const std::array<Matrix15d, 6>& cov, const Model& model,
                            const ShapeState& shape, const JointEstimate& joint, const Json& c)
{
    FusionResult out;
    double weight_sum = 0;
    double floor = c.at("variance_floor_m2");
    if (!std::isfinite(floor) || floor <= 0)
        throw std::invalid_argument("Invalid fusion variance floor");
    for (int i = 0; i < 6; ++i)
    {
        if (!positions[i].allFinite() || !cov[i].allFinite())
            throw std::invalid_argument("Invalid navigation fusion input");
        out.compensated[i] = positions[i] - shape.position[i];
        out.uniform += out.compensated[i] / 6;
        // Standard deviations are summed, rather than variances, to retain an unknown-correlation
        // bound.
        double std_bound = std::sqrt(std::max(0., cov[i].topLeftCorner<3, 3>().trace()));
        std_bound +=
            shape.position[i].norm() * std::sqrt(std::max(0., cov[0].block<3, 3>(6, 6).trace()));
        double quality = 1;
        for (int j = 0; j < i; ++j)
        {
            const auto& h = model.joints[j];
            V3D pivot =
                shape.position[h.parent] +
                shape.rotation[h.parent] * (h.translation + h.rest_rotation * h.anchor_child);
            V3D derivative =
                (shape.rotation[h.parent] * h.axis_parent).cross(shape.position[i] - pivot);
            std_bound += derivative.norm() * std::sqrt(std::max(0., joint.variance[j]));
            if (!joint.valid[j])
                quality = std::min(quality, c.at("invalid_joint_weight_multiplier").get<double>());
        }
        out.uncertainty_proxy_m2[i] = std::max(floor, std_bound * std_bound);
        out.weights[i] = quality / out.uncertainty_proxy_m2[i];
        weight_sum += out.weights[i];
    }
    if (!std::isfinite(weight_sum) || weight_sum <= 0)
        throw std::runtime_error("Fusion weights are not normalizable");
    double bound = 0;
    for (int i = 0; i < 6; ++i)
    {
        out.weights[i] /= weight_sum;
        out.weighted += out.weights[i] * out.compensated[i];
        bound += out.weights[i] * std::sqrt(out.uncertainty_proxy_m2[i]);
    }
    out.single = positions[0];
    out.weighted_trace_bound_m2 = bound * bound;
    return out;
}
} // namespace origami
