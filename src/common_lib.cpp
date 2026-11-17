#include "common_lib.h"
namespace origami
{
V3D vector3(const Json& j)
{
    if (!j.is_array() || j.size() != 3)
        throw std::invalid_argument("Expected 3-vector");
    V3D v;
    for (int i = 0; i < 3; ++i)
        v[i] = j.at(i).get<double>();
    if (!v.allFinite())
        throw std::invalid_argument("Nonfinite vector");
    return v;
}
Json array(const Eigen::MatrixXd& m)
{
    Json a = Json::array();
    for (int i = 0; i < m.rows(); ++i)
    {
        if (m.cols() == 1)
            a.push_back(m(i, 0));
        else
        {
            Json row = Json::array();
            for (int k = 0; k < m.cols(); ++k)
                row.push_back(m(i, k));
            a.push_back(row);
        }
    }
    return a;
}
} // namespace origami
