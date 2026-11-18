#include "calibration_diagnostics.h"
#include "multi_imu_eskf.h"
#include "static_calibration.h"
#include "synthetic_motion.h"
#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
using namespace origami;
namespace
{
constexpr double dt = .001;
const std::array<double, 4> durations{.5, 1, 2, 5};
struct Metric
{
    double square = 0, rpe = 0, last = 0;
    size_t n = 0, pairs = 0;
    std::deque<std::pair<V3D, V3D>> history;
    void add(const V3D& p, const V3D& truth)
    {
        last = (p - truth).norm();
        square += last * last;
        ++n;
        history.push_back({p, truth});
        if (history.size() > 1000)
        {
            rpe += ((p - history.front().first) - (truth - history.front().second)).squaredNorm();
            ++pairs;
            history.pop_front();
        }
    }
    Json json() const
    {
        return {{"ate_m", std::sqrt(square / n)},
                {"rpe_m", std::sqrt(rpe / pairs)},
                {"final_error_m", last}};
    }
};
uint64_t signature(const CalibrationSequence& sequence)
{
    uint64_t h = 1469598103934665603ULL;
    for (const auto& sample : sequence.dynamic)
        for (int i = 0; i < 6; ++i)
            for (const auto* v : {&sample.measured.gyro[i], &sample.measured.force[i],
                                  &sample.truth.gyro[i], &sample.truth.accel[i]})
                for (double x : *v)
                {
                    uint64_t bits;
                    std::memcpy(&bits, &x, sizeof(bits));
                    h = (h ^ bits) * 1099511628211ULL;
                }
    return h;
}
Json navigate(const Json& c, const Model& model, const std::vector<SimulationFrame>& data,
              const CalibrationSequence& sequence, const Calibration& initial, bool detail)
{
    Json options = c;
    options["joint_filter"]["enable_joint_position_constraint"] = true;
    options["joint_filter"]["enable_joint_axis_constraint"] = true;
    options["joint_filter"]["enable_shape_state"] = false;
    MultiImuEskf joint(options, model);
    apply_initial_bias(joint.nodes, initial);
    std::vector<ErrorStateKalmanFilter> independent;
    for (int i = 0; i < 6; ++i)
    {
        independent.emplace_back(c, false, false, false, true);
        independent.back().p = joint.nodes[i].p;
        independent.back().v = joint.nodes[i].v;
        independent.back().attitude.q = joint.nodes[i].attitude.q;
    }
    apply_initial_bias(independent, initial);
    JointEstimator angles(model, c, "A2", independent[0].attitude.q, initial);
    std::array<Metric, 3> metric;
    Json curve = Json::array();
    for (size_t k = 0; k < data.size(); ++k)
    {
        const auto& sample = sequence.dynamic[k].measured;
        for (int i = 0; i < 6; ++i)
            independent[i].propagate(sample.gyro[i], sample.force[i], dt);
        auto estimate = angles.update(sample, dt);
        auto shape = forward_shape(model, independent[0].attitude.q, estimate.angle);
        V3D mean = V3D::Zero();
        for (int i = 0; i < 6; ++i)
            mean += (independent[i].p - shape.position[i]) / 6;
        joint.propagate(sample, dt);
        if ((k + 1) % 10 == 0)
            joint.update(sample.time);
        const auto& truth = data[k].truth.markers[0];
        metric[0].add(independent[0].p, truth);
        metric[1].add(mean, truth);
        metric[2].add(joint.nodes[0].p, truth);
        if (detail && (k % 10 == 0 || k + 1 == data.size()))
            curve.push_back({sample.time, metric[0].last, metric[1].last, metric[2].last});
    }
    auto diagnostic = joint.diagnostics();
    return {{"methods",
             {{"单IMU积分", metric[0].json()},
              {"多IMU均值融合", metric[1].json()},
              {"铰链位置与轴约束", metric[2].json()}}},
            {"curve", curve},
            {"covariance", diagnostic}};
}
void write_static_example(const CalibrationSequence& s, const std::string& mode,
                          const std::string& folder)
{
    std::ofstream sensors(folder + "/static_" + mode + ".csv"),
        truth(folder + "/static_" + mode + "_bias_truth.csv");
    sensors << std::setprecision(15);
    truth << std::setprecision(15);
    sensors << "time_s,node,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,force_x_m_s2,force_y_m_s2,force_"
               "z_m_s2\n";
    truth << "time_s,node,bg_x_rad_s,bg_y_rad_s,bg_z_rad_s,ba_x_m_s2,ba_y_m_s2,ba_z_m_s2\n";
    for (size_t k = 0; k < s.stationary.size(); ++k)
        for (int i = 0; i < 6; ++i)
        {
            sensors << s.stationary[k].time << ',' << i;
            truth << s.stationary[k].time << ',' << i;
            for (const auto* v : {&s.stationary[k].gyro[i], &s.stationary[k].force[i]})
                for (double x : *v)
                    sensors << ',' << x;
            for (const auto* v : {&s.stationary_truth[k].gyro[i], &s.stationary_truth[k].accel[i]})
                for (double x : *v)
                    truth << ',' << x;
            sensors << '\n';
            truth << '\n';
        }
}
Json experiment(Json config, const Model& model, const std::vector<SimulationFrame>& data, int seed,
                const std::string& mode, const std::string& generated)
{
    config["noise"]["seed"] = seed;
    if (mode != "full")
    {
        config["noise"]["gyro_bias_rw_deg_s_sqrt_s"] = 0.;
        config["noise"]["accel_bias_rw_m_s2_sqrt_s"] = 0.;
    }
    auto sequence = make_calibration_sequence(config, mode, data, 5000);
    if (seed == 42)
        write_static_example(sequence, mode, generated);
    auto q = config.at("initial_root_quaternion_wxyz");
    std::array<double, 5> zero{};
    auto shape = forward_shape(model, Quaterniond(q[0], q[1], q[2], q[3]), zero);
    Json result = {{"seed", seed},
                   {"noise", mode},
                   {"dynamic_signature", std::to_string(signature(sequence))},
                   {"calibrations", Json::array()},
                   {"navigation", Json::array()}};
    auto run = [&](const std::string& name, double duration, const Calibration& initial)
    {
        bool detail =
            seed == 42 && (name == "未标定" || (duration == 2 && name == "陀螺与加速度标定"));
        auto value = navigate(config, model, data, sequence, initial, detail);
        value["strategy"] = name;
        value["duration_s"] = duration;
        value["oracle_debug_only"] = name == "真值零偏调试";
        result["navigation"].push_back(value);
    };
    Calibration empty;
    empty.gyro.fill(V3D::Zero());
    empty.accel.fill(V3D::Zero());
    run("未标定", 0, empty);
    // This branch alone consumes evaluation truth for an explicitly labelled debug control.
    Calibration oracle;
    oracle.available = true;
    oracle.gyro = sequence.boundary_truth.gyro;
    oracle.accel = sequence.boundary_truth.accel;
    run("真值零偏调试", 0, oracle);
    for (double duration : durations)
    {
        size_t n = std::llround(duration / dt);
        std::vector<ImuMeasurements> window(sequence.stationary.end() - n,
                                            sequence.stationary.end());
        auto estimate = estimate_stationary_bias(window, shape, config, dt);
        Json item = {{"duration_s", duration},
                     {"accepted", estimate.available},
                     {"reason", estimate.reason},
                     {"samples", estimate.available ? n : 0},
                     {"nodes", Json::array()}};
        double gw = config["noise"]["gyro_white_std_deg_s"].get<double>() * pi / 180,
               aw = config["noise"]["accel_white_std_m_s2"],
               gr = config["noise"]["gyro_bias_rw_deg_s_sqrt_s"].get<double>() * pi / 180,
               ar = config["noise"]["accel_bias_rw_m_s2_sqrt_s"];
        item["gyro_white_mean_std_rad_s"] = gw / std::sqrt(n);
        item["accel_white_mean_std_m_s2"] = aw / std::sqrt(n);
        item["gyro_end_bias_error_std_rad_s"] = std::sqrt(mean_bias_error_variance(gw, gr, n, dt));
        item["accel_end_bias_error_std_m_s2"] = std::sqrt(mean_bias_error_variance(aw, ar, n, dt));
        for (int i = 0; i < 6; ++i)
        {
            Json node = {{"node", i},
                         {"true_bg", array(sequence.boundary_truth.gyro[i])},
                         {"true_ba", array(sequence.boundary_truth.accel[i])}};
            if (estimate.available)
            {
                node["estimated_bg"] = array(estimate.gyro[i]);
                node["estimated_ba"] = array(estimate.accel[i]);
                node["error_bg"] = array(V3D(estimate.gyro[i] - sequence.boundary_truth.gyro[i]));
                node["error_ba"] = array(V3D(estimate.accel[i] - sequence.boundary_truth.accel[i]));
            }
            item["nodes"].push_back(node);
        }
        result["calibrations"].push_back(item);
        if (estimate.available)
            for (auto strategy : {"仅陀螺标定", "陀螺与加速度标定"})
                run(strategy, duration, calibration_initialization(estimate, strategy));
    }
    return result;
}
Json online_bias_audit(const Json& config, const Model& model,
                       const std::vector<SimulationFrame>& data)
{
    Json audit = Json::array();
    auto initial_shape = forward_shape(model, Quaterniond::Identity(), {});
    for (auto mode : {"fixed_bias", "full"})
    {
        Json c = config;
        c["noise"]["seed"] = 42;
        auto sequence = make_calibration_sequence(c, mode, data, 5000);
        std::vector<ImuMeasurements> window(sequence.stationary.end() - 2000,
                                            sequence.stationary.end());
        auto estimate = estimate_stationary_bias(window, initial_shape, c, dt);
        for (auto strategy : {"未标定", "陀螺与加速度标定"})
        {
            auto initial = calibration_initialization(estimate, strategy);
            c["joint_filter"]["enable_joint_position_constraint"] = true;
            c["joint_filter"]["enable_joint_axis_constraint"] = true;
            c["joint_filter"]["enable_shape_state"] = false;
            MultiImuEskf filter(c, model);
            apply_initial_bias(filter.nodes, initial);
            std::array<V3D, 6> frozen_g, frozen_a;
            for (int i = 0; i < 6; ++i)
            {
                frozen_g[i] = filter.nodes[i].bg;
                frozen_a[i] = filter.nodes[i].ba;
            }
            std::array<double, 6> frozen_g2{}, frozen_a2{}, online_g2{}, online_a2{};
            Metric nav;
            for (size_t k = 0; k < data.size(); ++k)
            {
                filter.propagate(sequence.dynamic[k].measured, dt);
                if ((k + 1) % 10 == 0)
                    filter.update(sequence.dynamic[k].measured.time);
                nav.add(filter.nodes[0].p, data[k].truth.markers[0]);
                // Evaluation only: bias truth never enters propagation or a measurement update.
                const auto& truth = sequence.dynamic[k].truth;
                for (int i = 0; i < 6; ++i)
                {
                    frozen_g2[i] += (frozen_g[i] - truth.gyro[i]).squaredNorm();
                    frozen_a2[i] += (frozen_a[i] - truth.accel[i]).squaredNorm();
                    online_g2[i] += (filter.nodes[i].bg - truth.gyro[i]).squaredNorm();
                    online_a2[i] += (filter.nodes[i].ba - truth.accel[i]).squaredNorm();
                }
            }
            Json record = {{"noise", mode},
                           {"seed", 42},
                           {"strategy", strategy},
                           {"duration_s", std::string(strategy) == "未标定" ? 0 : 2},
                           {"navigation", nav.json()},
                           {"nodes", Json::array()}};
            for (int i = 0; i < 6; ++i)
                record["nodes"].push_back(
                    {{"node", i},
                     {"frozen_gyro_vector_rmse_rad_s", std::sqrt(frozen_g2[i] / data.size())},
                     {"online_gyro_vector_rmse_rad_s", std::sqrt(online_g2[i] / data.size())},
                     {"frozen_accel_vector_rmse_m_s2", std::sqrt(frozen_a2[i] / data.size())},
                     {"online_accel_vector_rmse_m_s2", std::sqrt(online_a2[i] / data.size())}});
            audit.push_back(record);
        }
    }
    return audit;
}
void write_results(const Json& results, const std::string& output)
{
    std::ofstream bias(output + "/bias_calibration.csv"),
        nav(output + "/calibration_navigation.csv"), curves(output + "/calibration_errors.csv");
    for (auto* out : {&bias, &nav, &curves})
        *out << std::setprecision(15);
    bias << "noise,seed,duration_s,node,accepted,samples,estimated_bg_x,estimated_bg_y,estimated_"
            "bg_z,estimated_ba_x,estimated_ba_y,estimated_ba_z,true_bg_x,true_bg_y,true_bg_z,true_"
            "ba_x,true_ba_y,true_ba_z,error_bg_x,error_bg_y,error_bg_z,error_ba_x,error_ba_y,error_"
            "ba_z,gyro_white_mean_std,accel_white_mean_std,gyro_end_bias_error_std,accel_end_bias_"
            "error_std\n";
    nav << "noise,seed,duration_s,strategy,method,oracle_debug_only,ate_m,rpe_m,final_error_m,"
           "paired_ate_gain_m,paired_rpe_gain_m,dynamic_signature\n";
    curves << "noise,duration_s,strategy,time_s,method,error_m\n";
    const std::array<std::string, 3> names{"单IMU积分", "多IMU均值融合", "铰链位置与轴约束"};
    for (const auto& job : results)
    {
        if (job.is_null())
            continue;
        for (const auto& c : job["calibrations"])
            for (const auto& node : c["nodes"])
            {
                bias << job["noise"].get<std::string>() << ',' << job["seed"] << ','
                     << c["duration_s"] << ',' << node["node"] << ',' << c["accepted"] << ','
                     << c["samples"];
                for (auto key :
                     {"estimated_bg", "estimated_ba", "true_bg", "true_ba", "error_bg", "error_ba"})
                    for (int k = 0; k < 3; ++k)
                        bias << ',' << (node.contains(key) ? node[key][k] : Json(nullptr));
                for (auto key : {"gyro_white_mean_std_rad_s", "accel_white_mean_std_m_s2",
                                 "gyro_end_bias_error_std_rad_s", "accel_end_bias_error_std_m_s2"})
                    bias << ',' << c[key];
                bias << '\n';
            }
        const auto& baseline = job["navigation"][0]["methods"];
        for (const auto& value : job["navigation"])
        {
            for (const auto& name : names)
            {
                const auto& m = value["methods"][name];
                nav << job["noise"].get<std::string>() << ',' << job["seed"] << ','
                    << value["duration_s"] << ',' << value["strategy"].get<std::string>() << ','
                    << name << ',' << value["oracle_debug_only"] << ',' << m["ate_m"] << ','
                    << m["rpe_m"] << ',' << m["final_error_m"] << ','
                    << baseline[name]["ate_m"].get<double>() - m["ate_m"].get<double>() << ','
                    << baseline[name]["rpe_m"].get<double>() - m["rpe_m"].get<double>() << ','
                    << job["dynamic_signature"].get<std::string>() << '\n';
            }
            for (const auto& point : value["curve"])
                for (int i = 0; i < 3; ++i)
                    curves << job["noise"].get<std::string>() << ',' << value["duration_s"] << ','
                           << value["strategy"].get<std::string>() << ',' << point[0] << ','
                           << names[i] << ',' << point[i + 1] << '\n';
        }
    }
}
} // namespace
int main(int argc, char** argv)
{
    try
    {
        if (argc != 6 && argc != 7)
            throw std::invalid_argument(
                "Usage: calibration_study CONFIG DATA MODEL OUTPUT GENERATED [--seed42]");
        bool feedback_only = argc == 7 && std::string(argv[6]) == "--bias-feedback-only";
        bool pilot = argc == 7 && !feedback_only;
        if (pilot && std::string(argv[6]) != "--seed42")
            throw std::invalid_argument("Unknown mode");
        Json config;
        std::ifstream(argv[1]) >> config;
        if (config.at("sample_rate_hz") != 1000 || config.at("integration") != "step_end")
            throw std::invalid_argument(
                "This study requires the documented 1000 Hz step-end baseline");
        const std::string output = argv[4], generated = argv[5];
        std::filesystem::create_directories(output);
        std::filesystem::create_directories(generated);
        auto model = load_robot_model(argv[3], config["robot"]["geometry"]);
        auto frames = read_simulation(argv[2], 1000);
        const auto initial_hash_path = std::filesystem::path(output) / "summary.json";
        Json summary;
        if (std::filesystem::exists(initial_hash_path))
            std::ifstream(initial_hash_path) >> summary;
        if (feedback_only)
        {
            if (summary.value("status", std::string()) != "completed")
                throw std::runtime_error(
                    "Complete navigation study required before feedback audit");
            summary["online_bias_audit_seed42"] = online_bias_audit(config, model, frames);
            std::ofstream(initial_hash_path) << summary.dump(2);
            return 0;
        }
        if (!summary.contains("zero_noise"))
        {
            auto temp = std::filesystem::temp_directory_path() / "origami_calibration_consistent";
            auto paths = generate_synthetic_motion(config, model, temp.string());
            auto consistent_path = std::filesystem::path(generated) / "consistent_motion.txt";
            std::filesystem::copy_file(paths.back(), consistent_path,
                                       std::filesystem::copy_options::overwrite_existing);
            auto consistent = read_simulation(consistent_path.string(), 1000);
            summary["zero_noise"] =
                zero_noise_diagnostics(config, model, frames, consistent, output);
            std::filesystem::remove_all(temp);
        }
        std::ofstream ideal(generated + "/static_ideal.csv");
        ideal << std::setprecision(15);
        ideal << "time_s,node,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,force_x_m_s2,force_y_m_s2,"
                 "force_z_m_s2\n";
        for (int k = 0; k < 5000; ++k)
            for (int i = 0; i < 6; ++i)
                ideal << (k + 1 - 5000) * dt << ',' << i << ",0,0,0,0,0,9.81\n";
        Json results = Json::array();
        int seeds = pilot ? 1 : 30;
        for (int k = 0; k < 2 * seeds; ++k)
            results.push_back(nullptr);
        summary["protocol"] = {
            {"stationary_seconds", 5},
            {"durations_s", durations},
            {"dynamic_samples", frames.size()},
            {"initialization", "Known unfolded shape, identity attitudes, zero initial velocity"},
            {"noise_continuity",
             "One 5s stationary prefix per seed/mode, then unchanged original dynamic time axis; "
             "shorter calibration uses a suffix of the same prefix"},
            {"covariance_policy", "Same original joint-filter covariance priors for every "
                                  "strategy; only initial bias means change"},
            {"oracle_policy", "True bias is read only by labelled debug initialization and "
                              "evaluator; never by normal calibration"}};
        summary["config"] = config;
        summary["input"] = argv[2];
        summary["pilot"] = pilot;
        summary["status"] = "running";
        summary["completed_jobs"] = 0;
        summary["expected_jobs"] = 2 * seeds;
        summary.erase("cases");
        std::ofstream(initial_hash_path) << summary.dump(2);
        std::atomic<int> next{0}, complete{0};
        std::mutex output_mutex;
        std::exception_ptr failure;
        size_t workers = std::min(6, 2 * seeds);
        std::vector<std::thread> pool;
        for (size_t w = 0; w < workers; ++w)
            pool.emplace_back(
                [&]()
                {
                    try
                    {
                        for (;;)
                        {
                            int index = next++;
                            if (index >= 2 * seeds)
                                break;
                            int seed = 42 + index / 2;
                            std::string mode = index % 2 ? "full" : "fixed_bias";
                            auto value = experiment(config, model, frames, seed, mode, generated);
                            std::lock_guard<std::mutex> lock(output_mutex);
                            results[index] = value;
                            std::cout << "Completed " << ++complete << '/' << 2 * seeds << ": seed "
                                      << seed << ' ' << mode << std::endl;
                        }
                    }
                    catch (...)
                    {
                        std::lock_guard<std::mutex> lock(output_mutex);
                        failure = std::current_exception();
                        next = 2 * seeds;
                    }
                });
        for (auto& worker : pool)
            worker.join();
        summary["cases"] = results;
        summary["completed_jobs"] = complete.load();
        summary["expected_jobs"] = 2 * seeds;
        summary["status"] = failure ? "failed" : "completed";
        if (!failure)
            summary["online_bias_audit_seed42"] = online_bias_audit(config, model, frames);
        write_results(results, output);
        std::ofstream(initial_hash_path) << summary.dump(2);
        if (failure)
            std::rethrow_exception(failure);
        std::cout << "Saved zero-noise diagnostics and calibration comparisons" << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
