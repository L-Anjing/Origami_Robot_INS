#include "shape_kinematics.h"
namespace origami
{
double joint_angle_from_relative(const JointModel& h, const Quaterniond& relative, double reference)
{
    Quaterniond twist = (h.rest_rotation.conjugate() * relative).normalized();
    double angle = 2 * std::atan2(twist.vec().dot(h.axis_child), twist.w()) + h.reference;
    return angle + 2 * pi * std::round((reference - angle) / (2 * pi));
}
ShapeState forward_shape(const Model& model, const Quaterniond& root,
                         const std::array<double, 5>& angles)
{
    if (!root.coeffs().allFinite() || root.norm() < 1e-12)
        throw std::invalid_argument("Invalid root quaternion");
    ShapeState s;
    s.rotation[0] = root.normalized();
    s.position[0] = V3D::Zero();
    for (int j = 0; j < 5; ++j)
    {
        if (!std::isfinite(angles[j]))
            throw std::invalid_argument("Nonfinite joint angle");
        const auto& h = model.joints[j];
        if (h.parent < 0 || h.child != j + 1 || h.parent >= h.child)
            throw std::invalid_argument("Expected serial hinge chain");
        Quaterniond relative =
            h.rest_rotation * Quaterniond(Eigen::AngleAxisd(angles[j] - h.reference, h.axis_child));
        V3D offset = h.translation + h.rest_rotation * h.anchor_child - relative * h.anchor_child;
        s.rotation[h.child] = (s.rotation[h.parent] * relative).normalized();
        s.position[h.child] = s.position[h.parent] + s.rotation[h.parent] * offset;
    }
    s.terminal = s.position[5] + s.rotation[5] * model.offsets[5];
    s.physical_tip = s.position[5] + s.rotation[5] * V3D(model.physical_length_m, 0, 0);
    return s;
}
} // namespace origami
