#pragma once
#include "common_lib.h"
#include <sstream>
namespace origami
{
std::vector<Sample> read_recording(const std::string& path);
struct Detection
{
    bool stationary = false;
    double statistic = INFINITY;
};
class StationaryDetector
{
    Json c_;
    double g_, quiet_ = 0;
    int size_;
    bool timestamped_;
    std::deque<std::pair<Sample, bool>> window_;

  public:
    StationaryDetector(const Json& c, double g, double dt, bool timestamped);
    Detection update(const Sample& s, bool valid, double dt, double gapLimit = INFINITY);
};
std::vector<std::string> split(const std::string&, char);
void end(std::istringstream&);
double median(const std::deque<double>&);
bool quality(const Sample&, const Json&, double, const V3D&);
void finite(const Sample&);
} // namespace origami
