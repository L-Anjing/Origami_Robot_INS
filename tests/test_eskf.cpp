#include "IMU_Processing.hpp"
#include <fstream>
#include <iostream>
void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}
int main(int argc, char** argv)
{
    try
    {
        if (argc != 2)
            return 2;
        std::ifstream in(argv[1]);
        origami::Json c;
        in >> c;
        origami::ErrorStateKalmanFilter f(c, true, true);
        for (int k = 0; k < 1000; ++k)
            f.propagate(origami::V3D::Zero(), origami::V3D(0, 0, 9.81), .001);
        require(f.p.norm() < 1e-10, "Static drift");
        f.check_covariance();
        origami::ErrorStateKalmanFilter a(c, false, true), b(c, false, false);
        for (int k = 0; k < 100; ++k)
        {
            a.propagate(origami::V3D::Zero(), origami::V3D(1, 0, 9.81), .01, false);
            b.propagate(origami::V3D::Zero(), origami::V3D(1, 0, 9.81), .01, false);
        }
        require(std::abs(a.p.x() - .5) < 1e-10, "Constant acceleration");
        require(std::abs(b.p.x() - .505) < 1e-10, "Step-end acceleration");
        f = origami::ErrorStateKalmanFilter(c, true, true);
        f.v = origami::V3D(.01, 0, 0);
        f.P(12, 3) = f.P(3, 12) = .0001;
        auto u = f.zupt(true);
        require(u.accepted, "ZUPT rejected");
        require(f.ba.norm() > 0, "Bias not updated");
        require(f.v.norm() < .01, "Velocity not corrected");
        f.check_covariance();
        bool failed = false;
        try
        {
            f.propagate(origami::V3D::Zero(), origami::V3D::Zero(), 0);
        }
        catch (const std::invalid_argument&)
        {
            failed = true;
        }
        require(failed, "Invalid dt accepted");
        require((origami::right_jacobian(origami::V3D::Zero()) - origami::M3D::Identity()).norm() <
                    1e-12,
                "Jacobian at zero");
        std::cout << "core tests passed\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
