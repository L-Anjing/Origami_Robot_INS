#include "so3_math.h"
namespace origami
{
M3D skew(const V3D& v)
{
    M3D a;
    a << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return a;
}
Quaterniond increment(const V3D& v)
{
    double a = v.norm();
    double s = a < 1e-12 ? .5 : std::sin(a / 2) / a;
    return Quaterniond(std::cos(a / 2), s * v.x(), s * v.y(), s * v.z());
}
M3D right_jacobian(const V3D& v)
{
    double a = v.norm();
    M3D s = skew(v);
    if (a < 1e-6)
        return M3D::Identity() - .5 * s + s * s / 6;
    return M3D::Identity() - (1 - std::cos(a)) / (a * a) * s +
           (a - std::sin(a)) / (a * a * a) * s * s;
}
} // namespace origami
