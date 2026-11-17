#include "preprocess.h"
#include <algorithm>
#include <fstream>
namespace origami
{
std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> r;
    std::istringstream in(s);
    std::string t;
    while (std::getline(in, t, delimiter))
        r.push_back(t);
    return r;
}
void end(std::istringstream& in)
{
    std::string extra;
    if (!in || in >> extra)
        throw std::invalid_argument("Wrong input field count");
}
double median(const std::deque<double>& v)
{
    if (v.empty())
        throw std::runtime_error("No sample intervals");
    std::vector<double> a(v.begin(), v.end());
    std::sort(a.begin(), a.end());
    return (a[(a.size() - 1) / 2] + a[a.size() / 2]) * .5;
}
bool quality(const Sample& s, const Json& c, double g, const V3D& corrected)
{
    return corrected.norm() <= c.at("impact_force_norm_g").get<double>() * g &&
           s.force.cwiseAbs().maxCoeff() < c.at("accelerometer_range_g").get<double>() * g &&
           s.gyro.cwiseAbs().maxCoeff() < c.at("gyroscope_range_deg_s").get<double>() * pi / 180;
}
void finite(const Sample& s)
{
    if (!s.force.allFinite() || !s.gyro.allFinite())
        throw std::invalid_argument("Nonfinite input");
}
std::vector<Sample> read_recording(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot read " + path);
    std::vector<Sample> samples;
    std::string line;
    int64_t lastStep = 0;
    int lineNo = 0;
    while (std::getline(file, line))
    {
        ++lineNo;
        if (lineNo == 1 && line.compare(0, 3, "\xef\xbb\xbf") == 0)
            line.erase(0, 3);
        auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#')
            continue;
        try
        {
            auto blocks = split(line, ';');
            if (blocks.size() != 2)
                throw std::invalid_argument("Expected 2 blocks");
            std::istringstream in(blocks[0]), ts(blocks[1]);
            Sample s;
            int64_t step;
            int sensor;
            double w, x, y, z;
            in >> step >> sensor >> w >> x >> y >> z >> s.force.x() >> s.force.y() >> s.force.z() >>
                s.gyro.x() >> s.gyro.y() >> s.gyro.z();
            end(in);
            std::string t;
            ts >> t >> s.stamp;
            end(ts);
            s.device = Quaterniond(w, x, y, z);
            finite(s);
            if (step <= lastStep || sensor != 0 || s.stamp < 0 ||
                s.stamp / 1000000000LL > 2147483647LL ||
                (!samples.empty() && s.stamp <= samples.back().stamp) ||
                !s.device.coeffs().allFinite() || std::abs(s.device.norm() - 1) >= .01 || t != "T")
                throw std::invalid_argument("Invalid sample ID, timestamp or quaternion");
            s.device.normalize();
            samples.push_back(s);
            lastStep = step;
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error(path + ":" + std::to_string(lineNo) + ": " + e.what());
        }
    }
    if (samples.size() < 3)
        throw std::invalid_argument("Need 3 samples");
    return samples;
}
StationaryDetector::StationaryDetector(const Json& c, double g, double dt, bool timestamped)
    : c_(c), g_(g), size_(std::max(2, int(std::ceil(c.at("window_s").get<double>() / dt - 1e-9)))),
      timestamped_(timestamped)
{
    for (auto k : {"window_s", "sigma_accel_m_s2", "sigma_gyro_rad_s", "minimum_stationary_s",
                   "glrt_threshold", "gyro_limit_rad_s", "force_tolerance_m_s2"})
    {
        double v = c.at(k);
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Invalid detector setting");
    }
}
Detection StationaryDetector::update(const Sample& s, bool valid, double dt, double gapLimit)
{
    bool quiet = valid && s.gyro.norm() <= c_.at("gyro_limit_rad_s").get<double>() &&
                 std::abs(s.force.norm() - g_) <= c_.at("force_tolerance_m_s2").get<double>();
    bool gap = !window_.empty() && (s.stamp - window_.back().first.stamp) * 1e-9 > gapLimit;
    quiet_ = quiet ? (gap ? 0 : quiet_ + dt) : 0;
    window_.push_back({s, timestamped_ ? quiet : valid});
    int64_t cutoff = s.stamp - std::llround(c_.at("window_s").get<double>() * 1e9);
    if (timestamped_)
    {
        while (window_.size() > 1 && window_[1].first.stamp <= cutoff)
            window_.pop_front();
        if (window_.size() < 3 || window_.front().first.stamp > cutoff)
            return {};
    }
    else
    {
        while (int(window_.size()) > size_)
            window_.pop_front();
        if (int(window_.size()) < size_)
            return {};
    }
    V3D mean = V3D::Zero();
    for (auto& item : window_)
        mean += item.first.force;
    mean /= window_.size();
    V3D expected = g_ * mean / std::max(mean.norm(), 1e-12);
    double sum = 0;
    bool guard = true;
    double sa = c_.at("sigma_accel_m_s2"), sg = c_.at("sigma_gyro_rad_s");
    for (size_t i = 0; i < window_.size(); ++i)
    {
        auto& [a, ok] = window_[i];
        sum += (a.force - expected).squaredNorm() / (sa * sa) + a.gyro.squaredNorm() / (sg * sg);
        guard = guard && ok && a.gyro.norm() <= c_.at("gyro_limit_rad_s").get<double>() &&
                std::abs(a.force.norm() - g_) <= c_.at("force_tolerance_m_s2").get<double>();
        if (timestamped_ && i > 0)
            guard = guard && (a.stamp - window_[i - 1].first.stamp) * 1e-9 <= gapLimit;
    }
    Detection d;
    d.statistic = sum / window_.size();
    d.stationary = guard && quiet_ + 1e-10 >= c_.at("minimum_stationary_s").get<double>() &&
                   d.statistic < c_.at("glrt_threshold").get<double>();
    return d;
}
} // namespace origami
