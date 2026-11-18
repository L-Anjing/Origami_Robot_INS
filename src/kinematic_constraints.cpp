#include "kinematic_constraints.h"
namespace origami
{
V3D rotation_log(const Quaterniond& input)
{
    Quaterniond q = input.normalized();
    if (q.w() < 0)
        q.coeffs() *= -1;
    double s = q.vec().norm();
    if (s < 1e-10)
        return 2 * q.vec();
    return q.vec() * (2 * std::atan2(s, q.w()) / s);
}
M3D inverse_right_jacobian(const V3D& v)
{
    double t = v.norm();
    M3D cross = skew(v);
    double c = t < 1e-5 ? 1. / 12. + t * t / 720. : (1 - .5 * t / std::tan(.5 * t)) / (t * t);
    return M3D::Identity() + .5 * cross + c * cross * cross;
}
Constraint hinge_position(const std::vector<ErrorStateKalmanFilter>& x, const JointModel& h,
                          int index, int n)
{
    Constraint c;
    c.kind = "position";
    c.joint = index;
    c.H = Eigen::MatrixXd::Zero(3, n);
    auto& a = x.at(h.parent);
    auto& b = x.at(h.child);
    M3D Ra = a.attitude.q.toRotationMatrix(), Rb = b.attitude.q.toRotationMatrix();
    V3D la = h.translation + h.rest_rotation * h.anchor_child, lb = h.anchor_child;
    c.value = a.p + Ra * la - b.p - Rb * lb;
    c.H.block<3, 3>(0, 15 * h.parent).setIdentity();
    c.H.block<3, 3>(0, 15 * h.child) = -M3D::Identity();
    c.H.block<3, 3>(0, 15 * h.parent + 6) = -Ra * skew(la);
    c.H.block<3, 3>(0, 15 * h.child + 6) = Rb * skew(lb);
    return c;
}
Constraint hinge_axis(const std::vector<ErrorStateKalmanFilter>& x, const JointModel& h, int index,
                      int n, const Eigen::Matrix<double, 3, 2>* frozen)
{
    Constraint c;
    c.kind = "axis";
    c.joint = index;
    c.H = Eigen::MatrixXd::Zero(2, n);
    M3D Ra = x.at(h.parent).attitude.q.toRotationMatrix(),
        Rb = x.at(h.child).attitude.q.toRotationMatrix();
    V3D a = Ra * h.axis_parent, b = Rb * h.axis_child;
    if (frozen)
        c.tangent = *frozen;
    else
    {
        V3D seed = std::abs(a.x()) < .8 ? V3D::UnitX() : V3D::UnitY();
        c.tangent.col(0) = (seed - a * a.dot(seed)).normalized();
        c.tangent.col(1) = a.cross(c.tangent.col(0));
    }
    c.value = c.tangent.transpose() * a.cross(b);
    c.H.block<2, 3>(0, 15 * h.parent + 6) =
        c.tangent.transpose() * skew(b) * Ra * skew(h.axis_parent);
    c.H.block<2, 3>(0, 15 * h.child + 6) =
        -c.tangent.transpose() * skew(a) * Rb * skew(h.axis_child);
    return c;
}
std::vector<Constraint> shape_constraints(const std::vector<ErrorStateKalmanFilter>& x,
                                          const Model& model, const std::array<double, 5>& angles,
                                          int n)
{
    auto fk = forward_shape(model, Quaterniond::Identity(), angles);
    M3D R0 = x[0].attitude.q.toRotationMatrix();
    std::vector<Constraint> out;
    for (int i = 1; i < 6; ++i)
    {
        V3D displacement = R0.transpose() * (x[i].p - x[0].p);
        Constraint p;
        p.kind = "shape_position";
        p.joint = i - 1;
        p.H = Eigen::MatrixXd::Zero(3, n);
        p.value = displacement - fk.position[i];
        p.H.block<3, 3>(0, 0) = -R0.transpose();
        p.H.block<3, 3>(0, 15 * i) = R0.transpose();
        p.H.block<3, 3>(0, 6) = skew(displacement);
        Quaterniond relative =
            fk.rotation[i].conjugate() * x[0].attitude.q.conjugate() * x[i].attitude.q;
        V3D error = rotation_log(relative);
        M3D right = inverse_right_jacobian(error), left = inverse_right_jacobian(-error);
        Constraint r;
        r.kind = "shape_rotation";
        r.joint = i - 1;
        r.H = Eigen::MatrixXd::Zero(3, n);
        r.value = error;
        r.H.block<3, 3>(0, 6) = -left * fk.rotation[i].toRotationMatrix().transpose();
        r.H.block<3, 3>(0, 15 * i + 6) = right;
        for (int j = 0; j < i; ++j)
        {
            const auto& h = model.joints[j];
            V3D axis = fk.rotation[h.parent] * h.axis_parent;
            V3D pivot = fk.position[h.parent] +
                        fk.rotation[h.parent] * (h.translation + h.rest_rotation * h.anchor_child);
            p.H.block<3, 1>(0, 90 + j) = -axis.cross(fk.position[i] - pivot);
            r.H.block<3, 1>(0, 90 + j) = -left * (fk.rotation[i].conjugate() * axis);
        }
        out.push_back(p);
        out.push_back(r);
    }
    return out;
}
} // namespace origami
