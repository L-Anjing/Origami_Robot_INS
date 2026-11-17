#include "robot_odometry.h"
#include "robot_model.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
namespace origami
{
Json run_simulation(const std::string& path, const std::string& modelPath, const Json& c,
                    const std::string& mode, const std::string& output)
{
    if (mode != "estimated" && mode != "ground_truth")
        throw std::invalid_argument("Mode must be estimated or ground_truth");
    if (c.at("integration") != "step_end" || c.at("initial_velocity_epoch") != "simulation_start")
        throw std::invalid_argument("Simulation requires step_end, simulation_start");
    double hz = c.at("sample_rate_hz"), g = c.at("gravity_m_s2");
    if (!std::isfinite(hz) || hz <= 0)
        throw std::invalid_argument("Invalid sample rate");
    const auto& r = c.at("robot");
    auto model = load_robot_model(modelPath, r.at("geometry"));
    std::vector<int> refs = c.at("precompute_reference_indices").get<std::vector<int>>();
    refs.push_back(c.at("reference_imu_index"));
    std::sort(refs.begin(), refs.end());
    refs.erase(std::unique(refs.begin(), refs.end()), refs.end());
    for (int i : refs)
        if (i < 0 || i >= 6)
            throw std::invalid_argument("Reference index outside 0..5");
    struct Reference
    {
        int index;
        ErrorStateKalmanFilter raw;
        std::array<ErrorStateKalmanFilter, 3> filters;
        StationaryDetector detector;
        std::array<double, 3> attempts;
        std::array<double, 3> sum{}, final{};
        Reference(int i, const Json& c, double hz, bool debug)
            : index(i),
              raw(c, false, false, debug), filters{ErrorStateKalmanFilter(c, true, false, debug),
                                                   ErrorStateKalmanFilter(c, true, false, debug),
                                                   ErrorStateKalmanFilter(c, true, false, debug)},
              detector(c.at("robot").at("stationary"), c.at("gravity_m_s2"), 1 / hz, false),
              attempts{-INFINITY, -INFINITY, -INFINITY}
        {
        }
    };
    std::vector<Reference> references;
    bool debug = mode == "ground_truth";
    for (int index : refs)
    {
        references.emplace_back(index, c, hz, debug);
        auto& ref = references.back();
        V3D initial = vector3(r.at("initial_root_W_m"));
        for (int k = 0; k < index; ++k)
            initial += model.offsets[k];
        auto iv = c.at("initial_velocity_m_s");
        V3D velocity = vector3(iv.at(0).is_array() ? iv.at(index) : iv);
        ref.raw.p = initial;
        ref.raw.v = velocity;
        for (auto& f : ref.filters)
        {
            f.p = initial;
            f.v = velocity;
        }
    }
    std::filesystem::create_directories(output);
    std::ofstream out(output + "/frames.jsonl");
    if (!out)
        throw std::runtime_error("Cannot write frames");
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot read simulation");
    std::array<SixAxisAhrs, 6> shape;
    std::array<int, 3> previous{};
    std::array<double, 3> off{}, fall{-INFINITY, -INFINITY, -INFINITY};
    std::string line;
    int count = 0;
    std::array<long, 3> accepted{};
    while (std::getline(file, line))
    {
        if (count == 0 && line.compare(0, 3, "\xef\xbb\xbf") == 0)
            line.erase(0, 3);
        auto blocks = split(line, ';');
        if (blocks.size() != 8)
            throw std::invalid_argument("Expected 8 simulation blocks at line " +
                                        std::to_string(count + 1));
        std::array<Sample, 6> samples;
        int step = 0;
        for (int n = 0; n < 6; ++n)
        {
            std::istringstream in(blocks[n]);
            int id;
            if (n == 0)
                in >> step;
            in >> id;
            double w, x, y, z;
            auto& s = samples[n];
            in >> w >> x >> y >> z >> s.force.x() >> s.force.y() >> s.force.z() >> s.gyro.x() >>
                s.gyro.y() >> s.gyro.z();
            end(in);
            s.device = Quaterniond(w, x, y, z);
            finite(s);
            if (id != n || !s.device.coeffs().allFinite() || std::abs(s.device.norm() - 1) > .01)
                throw std::invalid_argument("Invalid simulation node/quaternion");
            s.device.normalize();
            s.stamp = std::llround(step / hz * 1e9);
        }
        if (step != count + 1)
            throw std::invalid_argument("Simulation steps must begin at 1 and be consecutive");
        double t = step / hz, dt = 1 / hz;
        std::array<int, 3> controls;
        std::istringstream ci(blocks[6]);
        for (auto& x : controls)
        {
            ci >> x;
            if (x != 0 && x != 1)
                throw std::invalid_argument("Nonbinary drive");
        }
        end(ci);
        std::array<V3D, 7> truth;
        std::istringstream ti(blocks[7]);
        std::string tag;
        ti >> tag;
        if (tag != "V")
            throw std::invalid_argument("Missing V block");
        for (auto& v : truth)
        {
            ti >> v.x() >> v.y() >> v.z();
            if (!v.allFinite())
                throw std::invalid_argument("Nonfinite truth");
        }
        end(ti);
        bool phase = false;
        auto pc = r.at("phase");
        for (int k = 0; k < 3; ++k)
        {
            if (count > 0 && previous[k] == 1 && controls[k] == 0)
                fall[k] = t;
            off[k] = controls[k] == 0 ? off[k] + dt : 0;
            previous[k] = controls[k];
        }
        for (int k : pc.at("relevant_drives").get<std::vector<int>>())
        {
            if (k < 0 || k > 2)
                throw std::invalid_argument("Invalid drive index");
            double age = t - fall[k];
            phase = phase || (controls[k] == 0 &&
                              off[k] + 1e-10 >= pc.at("stable_off_min_s").get<double>() &&
                              age >= pc.at("post_falling_delay_s").get<double>() &&
                              age <= pc.at("post_falling_window_s").get<double>());
        }
        Json frame = {{"time_s", t},
                      {"controls", controls},
                      {"truth_endpoints", Json::array()},
                      {"quaternion", Json::array()},
                      {"shape", Json::array()},
                      {"references", Json::object()},
                      {"phase_candidate", phase}};
        V3D point = V3D::Zero();
        frame["shape"].push_back(array(point));
        for (int k = 0; k < 6; ++k)
        {
            auto& s = samples[k];
            bool valid = quality(s, r.at("quality"), g, s.force);
            if (debug)
                shape[k].q = s.device;
            else
                shape[k].update(s.gyro, s.force, dt, r.at("ahrs"), g, valid);
            auto q = shape[k].q;
            frame["quaternion"].push_back({q.w(), q.x(), q.y(), q.z()});
            point += q * model.offsets[k];
            frame["shape"].push_back(array(point));
        }
        for (auto& v : truth)
            frame["truth_endpoints"].push_back(array(v));
        for (auto& ref : references)
        {
            auto s = samples[ref.index];
            bool valid = quality(s, r.at("quality"), g, s.force);
            std::optional<Quaterniond> dq;
            if (debug)
                dq = s.device;
            ref.raw.propagate(s.gyro, s.force, dt, valid, 1, dq);
            auto detection = ref.detector.update(s, valid, dt);
            Json result = {
                {"raw", ref.raw.snapshot()},
                {"policies", Json::array()},
                {"glrt_candidate", detection.stationary},
                {"glrt_valid", std::isfinite(detection.statistic)},
                {"glrt_statistic", std::isfinite(detection.statistic) ? detection.statistic : 0.},
                {"valid", valid}};
            for (int policy = 0; policy < 3; ++policy)
            {
                auto& f = ref.filters[policy];
                f.propagate(s.gyro, s.force, dt, valid,
                            valid ? 1 : r.at("quality").at("impact_noise_multiplier").get<double>(),
                            dq);
                bool candidate = policy == 1 ? detection.stationary
                                             : policy == 2 && detection.stationary && phase;
                bool requested =
                    candidate && t - ref.attempts[policy] + 1e-10 >=
                                     r.at("eskf").at("zupt_min_interval_s").get<double>();
                if (requested)
                    ref.attempts[policy] = t;
                auto u = f.zupt(requested, valid);
                accepted[policy] += u.accepted;
                Json state = f.snapshot();
                state["requested"] = requested;
                state["accepted"] = u.accepted;
                state["nis"] = u.nis;
                state["rejected_nis"] = u.rejected;
                result["policies"].push_back(state);
                auto idx = c.at("truth").at("endpoint_indices").at(ref.index);
                if (!idx.is_null())
                {
                    int endpoint = idx.get<int>();
                    if (endpoint < 0 || endpoint > 6)
                        throw std::invalid_argument("Invalid truth endpoint");
                    double error = (f.p - truth[endpoint]).norm();
                    ref.sum[policy] += error * error;
                    ref.final[policy] = error;
                }
                if (count % r.at("eskf").at("covariance_check_every").get<int>() == 0)
                    f.check_covariance();
            }
            frame["references"]["S" + std::to_string(ref.index)] = result;
        }
        out << frame.dump() << '\n';
        if (!out)
            throw std::runtime_error("Frame write failed");
        ++count;
    }
    if (count < 3)
        throw std::invalid_argument("Need 3 simulation frames");
    Json summary = {{"schema_version", 1},
                    {"samples", count},
                    {"mode", mode},
                    {"policies", {"none", "glrt", "phase"}},
                    {"zupt_updates", accepted},
                    {"model", model.json},
                    {"config", c},
                    {"references", Json::object()}};
    for (auto& ref : references)
    {
        for (auto& f : ref.filters)
            f.check_covariance();
        Json rmse = Json::array(), last = Json::array();
        for (int p = 0; p < 3; ++p)
        {
            bool known = !c.at("truth").at("endpoint_indices").at(ref.index).is_null();
            rmse.push_back(known ? Json(std::sqrt(ref.sum[p] / count)) : Json(nullptr));
            last.push_back(known ? Json(ref.final[p]) : Json(nullptr));
        }
        summary["references"]["S" + std::to_string(ref.index)] = {{"rmse_m", rmse},
                                                                  {"final_error_m", last}};
    }
    std::ofstream report(output + "/summary.json");
    report << summary.dump(2);
    if (!report)
        throw std::runtime_error("Summary write failed");
    return summary;
}
} // namespace origami
