#include "simulation_data.h"
#include "preprocess.h"
#include <fstream>
namespace origami
{
std::vector<SimulationFrame> read_simulation(const std::string& path, double hz)
{
    if (!std::isfinite(hz) || hz <= 0)
        throw std::invalid_argument("Invalid simulation sample rate");
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot read simulation");
    std::vector<SimulationFrame> frames;
    std::string line;
    while (std::getline(file, line))
    {
        if (frames.empty() && line.compare(0, 3, "\xef\xbb\xbf") == 0)
            line.erase(0, 3);
        try
        {
            auto blocks = split(line, ';');
            if (blocks.size() != 8)
                throw std::invalid_argument("Expected eight blocks");
            SimulationFrame frame;
            int step = 0;
            for (int i = 0; i < 6; ++i)
            {
                std::istringstream in(blocks[i]);
                int id;
                double w, x, y, z;
                if (i == 0)
                    in >> step;
                in >> id >> w >> x >> y >> z;
                in >> frame.imu.force[i].x() >> frame.imu.force[i].y() >> frame.imu.force[i].z() >>
                    frame.imu.gyro[i].x() >> frame.imu.gyro[i].y() >> frame.imu.gyro[i].z();
                end(in);
                Quaterniond q(w, x, y, z);
                if (id != i || !q.coeffs().allFinite() || std::abs(q.norm() - 1) > .01 ||
                    !frame.imu.force[i].allFinite() || !frame.imu.gyro[i].allFinite())
                    throw std::invalid_argument("Invalid IMU block");
                frame.truth.orientation[i] = q.normalized();
            }
            if (step != int(frames.size() + 1))
                throw std::invalid_argument("Expected consecutive simulation steps from 1");
            frame.imu.time = step / hz;
            std::istringstream controls(blocks[6]);
            for (auto& value : frame.controls)
            {
                controls >> value;
                if (value != 0 && value != 1)
                    throw std::invalid_argument("Expected binary controls");
            }
            end(controls);
            std::istringstream points(blocks[7]);
            std::string tag;
            points >> tag;
            if (tag != "V")
                throw std::invalid_argument("Missing V block");
            for (auto& p : frame.truth.markers)
            {
                points >> p.x() >> p.y() >> p.z();
                if (!p.allFinite())
                    throw std::invalid_argument("Invalid endpoint");
            }
            end(points);
            frames.push_back(frame);
        }
        catch (const std::exception& error)
        {
            throw std::invalid_argument(path + ":" + std::to_string(frames.size() + 1) + ": " +
                                        error.what());
        }
    }
    if (frames.size() < 3)
        throw std::invalid_argument("Need three simulation frames");
    return frames;
}
} // namespace origami
