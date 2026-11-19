#include "multi_imu_experiment.h"
#include "multi_imu_eskf.h"
#include "multi_imu_fusion.h"
#include "preprocess.h"
#include "simulation_data.h"
#include "synthetic_motion.h"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sys/resource.h>
#include <thread>
namespace origami
{
namespace
{
using Clock = std::chrono::steady_clock;
struct Metric
{
    double square = 0, rpe_square = 0, last = 0, terminal_square = 0, attitude_square = 0,
           yaw_square = 0;
    size_t count = 0, rpe_count = 0, terminal_count = 0;
    V3D xyz = V3D::Zero();
    std::deque<std::pair<V3D, V3D>> history;
    void add(const V3D& p, const Quaterniond& q, const SimulationTruth& truth, int reference,
             size_t lag, const std::optional<V3D>& terminal)
    {
        V3D error = p - truth.markers[reference];
        double e = error.norm();
        square += e * e;
        last = e;
        xyz += error.cwiseAbs2();
        double angle = q.angularDistance(truth.orientation[reference]);
        attitude_square += angle * angle;
        auto a = q.toRotationMatrix(), b = truth.orientation[reference].toRotationMatrix();
        double yaw = std::atan2(a(1, 0), a(0, 0)) - std::atan2(b(1, 0), b(0, 0));
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));
        yaw_square += yaw * yaw;
        ++count;
        if (terminal)
        {
            terminal_square += (*terminal - truth.markers[6]).squaredNorm();
            ++terminal_count;
        }
        history.push_back({p, truth.markers[reference]});
        if (history.size() > lag)
        {
            rpe_square +=
                ((p - history.front().first) - (truth.markers[reference] - history.front().second))
                    .squaredNorm();
            ++rpe_count;
            history.pop_front();
        }
    }
    Json json() const
    {
        return {{"samples", count},
                {"ate_m", std::sqrt(square / std::max(size_t(1), count))},
                {"rpe_1s_translation_m",
                 rpe_count ? Json(std::sqrt(rpe_square / rpe_count)) : Json(nullptr)},
                {"final_error_m", last},
                {"xyz_rmse_m", array(V3D((xyz / std::max(size_t(1), count)).cwiseSqrt()))},
                {"attitude_rmse_rad", std::sqrt(attitude_square / std::max(size_t(1), count))},
                {"yaw_rmse_rad", std::sqrt(yaw_square / std::max(size_t(1), count))},
                {"terminal_ate_m", terminal_count
                                       ? Json(std::sqrt(terminal_square / terminal_count))
                                       : Json(nullptr)}};
    }
};
struct FilterStats
{
    size_t accepted = 0, rejected = 0;
    double nis_sum = 0, nis_max = 0, min_eigen = INFINITY, max_symmetry = 0, max_cross = 0,
           max_joint_cross = 0, max_correction = 0;
    std::array<double, 6> gyro_bias_square{}, accel_bias_square{};
    size_t samples = 0;
    Json json() const
    {
        Json g = Json::array(), a = Json::array();
        for (int i = 0; i < 6; ++i)
        {
            g.push_back(std::sqrt(gyro_bias_square[i] / std::max(size_t(1), samples)));
            a.push_back(std::sqrt(accel_bias_square[i] / std::max(size_t(1), samples)));
        }
        return {{"accepted", accepted},
                {"rejected", rejected},
                {"mean_nis", accepted + rejected ? nis_sum / (accepted + rejected) : 0.},
                {"max_nis", nis_max},
                {"min_eigenvalue", std::isfinite(min_eigen) ? Json(min_eigen) : Json(nullptr)},
                {"max_symmetry_error", max_symmetry},
                {"max_cross_node_covariance_norm", max_cross},
                {"max_node_joint_cross_covariance_norm", max_joint_cross},
                {"max_state_correction_norm", max_correction},
                {"gyro_bias_rmse_vector_rad_s", g},
                {"accel_bias_rmse_vector_m_s2", a}};
    }
};
std::array<double, 5> prior_angles(const Json& c)
{
    std::array<double, 5> a;
    for (int j = 0; j < 5; ++j)
        a[j] = c.at("initial_joint_angles_rad")[j];
    return a;
}
Quaterniond prior_q(const Json& c)
{
    auto a = c.at("initial_root_quaternion_wxyz");
    return Quaterniond(a[0].get<double>(), a[1].get<double>(), a[2].get<double>(),
                       a[3].get<double>())
        .normalized();
}
Json scenario(const Json& c, const std::vector<SimulationFrame>& data, const Model& model,
              const std::string& folder, const std::string& noise_mode)
{
    auto start = Clock::now();
    std::filesystem::create_directories(folder);
    double dt = 1 / c.at("sample_rate_hz").get<double>();
    size_t lag = std::llround(1 / dt);
    auto init = forward_shape(model, prior_q(c), prior_angles(c));
    std::array<double, 5> rates{};
    for (int j = 0; j < 5; ++j)
        rates[j] = c.at("initial_joint_rates_rad_s")[j];
    auto initial_v = initial_node_velocities(
        model, init, rates, prior_q(c) * vector3(c.at("initial_root_angular_velocity_rad_s")),
        vector3(c.at("initial_velocity_m_s")));
    std::vector<ErrorStateKalmanFilter> independent;
    for (int i = 0; i < 6; ++i)
    {
        independent.emplace_back(c, true, false, false, !c.value("root_ahrs_feedback", false));
        independent.back().p = vector3(c.at("robot").at("initial_root_W_m")) + init.position[i];
        independent.back().v = initial_v[i];
        independent.back().attitude.q = init.rotation[i];
    }
    auto zupt = independent[0];
    StationaryDetector stationary(c.at("robot").at("stationary"), c.at("gravity_m_s2"), dt, false);
    double last_zupt = -INFINITY;
    size_t zupt_requested = 0, zupt_accepted = 0;
    Json active = c;
    if (noise_mode == "none")
    {
        active["noise"]["gyro_white_std_deg_s"] = 0.;
        active["noise"]["accel_white_std_m_s2"] = 0.;
    }
    if (noise_mode == "none" || noise_mode == "white")
    {
        active["noise"]["gyro_initial_bias_bound_deg_s"] = 0.;
        active["noise"]["accel_initial_bias_bound_m_s2"] = 0.;
    }
    if (noise_mode != "full")
    {
        active["noise"]["gyro_bias_rw_deg_s_sqrt_s"] = 0.;
        active["noise"]["accel_bias_rw_m_s2_sqrt_s"] = 0.;
    }
    JointEstimator a0(model, active, "A0", prior_q(c)), a2(model, active, "A2", prior_q(c));
    ImuNoise noise(c.at("noise"), noise_mode);
    std::map<std::string, std::unique_ptr<MultiImuEskf>> filters;
    for (auto name : {"D3a", "D3b", "D4"})
    {
        if (std::string(name) == "D4" &&
            (!c.at("experiment").value("run_d4", true) ||
             c.at("joint_filter").at("participating_imus").size() != 6))
            continue;
        Json cfg = c;
        cfg["joint_filter"]["enable_joint_position_constraint"] = true;
        cfg["joint_filter"]["enable_joint_axis_constraint"] = std::string(name) == "D3b";
        cfg["joint_filter"]["enable_shape_state"] = std::string(name) == "D4";
        filters[name] = std::make_unique<MultiImuEskf>(cfg, model);
    }
    std::map<std::string, Metric> metrics;
    std::map<std::string, FilterStats> stats;
    FilterStats independent_bias_stats;
    std::map<std::string, std::array<double, 5>> joint_squares, joint_max, joint_final,
        shape_joint_prior;
    std::map<std::string, std::array<double, 6>> shape_squares;
    std::map<std::string, double> terminal_shape_square;
    size_t update_stride = std::max(
        size_t(1), size_t(std::llround(c.at("sample_rate_hz").get<double>() /
                                       c.at("joint_filter").at("update_rate_hz").get<double>())));
    bool detail = c.at("noise").at("seed") == c.at("experiment").at("write_detailed_seed");
    size_t stride = c.at("experiment").value("detail_stride", 10);
    size_t diagnostic_stride = c.at("experiment").value("diagnostic_stride", 1000);
    std::array<double, 5> truth_angles = prior_angles(c);
    std::ofstream innovations(folder + "/innovation_statistics.csv"),
        corrections(folder + "/state_corrections.csv"),
        covariance(folder + "/covariance_diagnostics.csv"), biases(folder + "/bias_estimates.csv"),
        navigation(folder + "/navigation_errors.csv"), joints(folder + "/joint_angles.csv"),
        shapes(folder + "/shape_errors.csv"), frames;
    if (detail)
        frames.open(folder + "/frames.jsonl");
    for (auto file :
         {&innovations, &corrections, &covariance, &biases, &navigation, &joints, &shapes})
        *file << std::setprecision(12);
    innovations << "time_s,method,kind,joint,dimension,nis,accepted,innovation0,innovation1,"
                   "innovation2,batch_correction_norm,batch_bias_change_norm\n";
    corrections << "time_s,method";
    for (int i = 0; i < 95; ++i)
        corrections << ",dx" << i;
    corrections << '\n';
    covariance << "time_s,method,dimension,min_eigenvalue,symmetry_error,cross_node_norm,node_"
                  "joint_cross_norm,position_std_m\n";
    biases << "time_s,method,node,bgx,bgy,bgz,bax,bay,baz,true_bgx,true_bgy,true_bgz,true_bax,true_"
              "bay,true_baz\n";
    navigation
        << "time_s,method,reference_node,x_m,y_m,z_m,truth_x_m,truth_y_m,truth_z_m,error_m\n";
    joints << "time_s,method,joint,angle_rad,truth_rad,error_rad\n";
    shapes << "time_s,method,node,relative_position_error_m,terminal_shape_error_m\n";
    for (size_t k = 0; k < data.size(); ++k)
    {
        auto generated = noise.apply(data[k].imu, dt);
        const auto& m = generated.measured;
        const auto& truth = data[k].truth;
        for (int i = 0; i < 6; ++i)
        {
            Sample s;
            s.gyro = m.gyro[i];
            s.force = m.force[i];
            bool valid = quality(s, c.at("robot").at("quality"), c.at("gravity_m_s2"), s.force);
            independent[i].propagate(
                s.gyro, s.force, dt, valid,
                valid ? 1
                      : c.at("robot").at("quality").at("impact_noise_multiplier").get<double>());
            if (i == 0)
            {
                zupt.propagate(
                    s.gyro, s.force, dt, valid,
                    valid
                        ? 1
                        : c.at("robot").at("quality").at("impact_noise_multiplier").get<double>());
                auto detection = stationary.update(s, valid, dt);
                bool request = detection.stationary &&
                               m.time - last_zupt + 1e-10 >=
                                   c.at("robot").at("eskf").at("zupt_min_interval_s").get<double>();
                if (request)
                    last_zupt = m.time;
                V3D zupt_innovation = -zupt.v;
                auto zupt_update = zupt.zupt(request, valid);
                zupt_requested += request;
                zupt_accepted += zupt_update.accepted;
                if (request)
                {
                    innovations << m.time << ",B1_ZUPT,zupt,-1,3," << zupt_update.nis << ','
                                << zupt_update.accepted;
                    for (double x : zupt_innovation)
                        innovations << ',' << x;
                    innovations << ',' << (zupt_update.accepted ? zupt.injection.norm() : 0.) << ','
                                << (zupt_update.accepted ? zupt.injection.tail<6>().norm() : 0.)
                                << '\n';
                }
            }
        }
        ++independent_bias_stats.samples;
        for (int i = 0; i < 6; ++i)
        {
            independent_bias_stats.gyro_bias_square[i] +=
                (independent[i].bg - generated.truth.gyro[i]).squaredNorm();
            independent_bias_stats.accel_bias_square[i] +=
                (independent[i].ba - generated.truth.accel[i]).squaredNorm();
        }
        auto joint0 = a0.update(m, dt), joint2 = a2.update(m, dt);
        auto shape0 = forward_shape(model, independent[0].attitude.q, joint0.angle),
             shape2 = forward_shape(model, independent[0].attitude.q, joint2.angle),
             fixed = forward_shape(model, independent[0].attitude.q, prior_angles(c));
        // Truth is consumed exclusively by evaluation and the named Oracle output branch.
        for (int j = 0; j < 5; ++j)
            truth_angles[j] = joint_angle_from_relative(
                model.joints[j], truth.orientation[j].conjugate() * truth.orientation[j + 1],
                truth_angles[j]);
        auto oracle = forward_shape(model, independent[0].attitude.q, truth_angles);
        std::array<V3D, 6> position;
        std::array<Matrix15d, 6> P;
        for (int i = 0; i < 6; ++i)
        {
            position[i] = independent[i].p;
            P[i] = independent[i].P;
        }
        auto fusion = fuse_positions(position, P, model, shape2, joint2, c.at("fusion"));
        V3D a0_mean = V3D::Zero(), fixed_mean = V3D::Zero(), oracle_mean = V3D::Zero();
        for (int i = 0; i < 6; ++i)
        {
            a0_mean += (position[i] - shape0.position[i]) / 6;
            fixed_mean += (position[i] - fixed.position[i]) / 6;
            oracle_mean += (position[i] - oracle.position[i]) / 6;
        }
        Json frame = {{"time_s", m.time},
                      {"truth_S0", array(truth.markers[0])},
                      {"estimates", Json::object()}};
        auto record = [&](const std::string& name, const V3D& p, const Quaterniond& q,
                          int reference, const std::optional<V3D>& tip)
        {
            metrics[name].add(p, q, truth, reference, lag, tip);
            if (detail && k % stride == 0)
            {
                navigation << m.time << ',' << name << ',' << reference;
                for (double x : p)
                    navigation << ',' << x;
                for (double x : truth.markers[reference])
                    navigation << ',' << x;
                navigation << ',' << (p - truth.markers[reference]).norm() << '\n';
                frame["estimates"][name] = array(p);
            }
        };
        record("B0_RAW", independent[0].p, independent[0].attitude.q, 0,
               independent[0].p + shape2.terminal);
        record("B1_ZUPT", zupt.p, zupt.attitude.q, 0, zupt.p + shape2.terminal);
        for (int i = 0; i < 6; ++i)
            record("B2_S" + std::to_string(i), independent[i].p, independent[i].attitude.q, i,
                   std::nullopt);
        record("B3_D1_A2", fusion.uniform, independent[0].attitude.q, 0,
               fusion.uniform + shape2.terminal);
        record("D2_A2", fusion.weighted, independent[0].attitude.q, 0,
               fusion.weighted + shape2.terminal);
        record("B4_FIXED", fixed_mean, independent[0].attitude.q, 0, fixed_mean + fixed.terminal);
        record("A0_D1", a0_mean, independent[0].attitude.q, 0, a0_mean + shape0.terminal);
        record("ORACLE_D1", oracle_mean, independent[0].attitude.q, 0,
               oracle_mean + oracle.terminal);
        auto shape_metric = [&](const std::string& name, const std::array<double, 5>& estimate,
                                const ShapeState& shape)
        {
            for (int j = 0; j < 5; ++j)
            {
                double e = estimate[j] - truth_angles[j];
                joint_squares[name][j] += e * e;
                joint_max[name][j] = std::max(joint_max[name][j], std::abs(e));
                joint_final[name][j] = e;
                if (detail && k % stride == 0)
                    joints << m.time << ',' << name << ',' << j << ',' << estimate[j] << ','
                           << truth_angles[j] << ',' << e << '\n';
            }
            double tip = (shape.terminal - (truth.markers[6] - truth.markers[0])).norm();
            terminal_shape_square[name] += tip * tip;
            for (int i = 0; i < 6; ++i)
            {
                double e = (shape.position[i] - (truth.markers[i] - truth.markers[0])).norm();
                shape_squares[name][i] += e * e;
                if (detail && k % stride == 0)
                    shapes << m.time << ',' << name << ',' << i << ',' << e << ',' << tip << '\n';
            }
        };
        shape_metric("A0", joint0.angle, shape0);
        shape_metric("A2", joint2.angle, shape2);
        for (auto& [name, filter] : filters)
        {
            filter->propagate(m, dt);
            if ((k + 1) % update_stride == 0)
            {
                filter->update(m.time);
                bool accepted = false;
                for (auto& info : filter->last_innovations)
                {
                    auto& s = stats[name];
                    s.accepted += info.accepted;
                    s.rejected += !info.accepted;
                    if (std::isfinite(info.nis))
                    {
                        s.nis_sum += info.nis;
                        s.nis_max = std::max(s.nis_max, info.nis);
                    }
                    s.max_correction = std::max(s.max_correction, info.correction_norm);
                    innovations << m.time << ',' << name << ',' << info.kind << ',' << info.joint
                                << ',' << info.innovation.size() << ',' << info.nis << ','
                                << info.accepted;
                    for (int j = 0; j < 3; ++j)
                        innovations << ','
                                    << (j < info.innovation.size() ? info.innovation[j] : 0.);
                    innovations << ',' << info.correction_norm << ',' << info.bias_change_norm
                                << '\n';
                    accepted = accepted || info.accepted;
                }
                if (accepted)
                {
                    corrections << m.time << ',' << name;
                    for (int j = 0; j < 95; ++j)
                        corrections
                            << ','
                            << (j < filter->last_correction.size() ? filter->last_correction[j]
                                                                   : 0.);
                    corrections << '\n';
                }
            }
            auto& nodes = filter->nodes;
            std::optional<V3D> tip;
            if (filter->active(5))
                tip = nodes[5].p + nodes[5].attitude.q * model.offsets[5];
            record(name, nodes[0].p, nodes[0].attitude.q, 0, tip);
            auto& s = stats[name];
            ++s.samples;
            for (int i = 0; i < 6; ++i)
                if (filter->active(i))
                {
                    s.gyro_bias_square[i] += (nodes[i].bg - generated.truth.gyro[i]).squaredNorm();
                    s.accel_bias_square[i] +=
                        (nodes[i].ba - generated.truth.accel[i]).squaredNorm();
                    if (detail && k % stride == 0)
                    {
                        biases << m.time << ',' << name << ',' << i;
                        for (const V3D* v : {&nodes[i].bg, &nodes[i].ba, &generated.truth.gyro[i],
                                             &generated.truth.accel[i]})
                            for (double x : *v)
                                biases << ',' << x;
                        biases << '\n';
                    }
                }
            if (k % diagnostic_stride == 0 || k + 1 == data.size())
            {
                auto d = filter->diagnostics();
                s.min_eigen = std::min(s.min_eigen, d.at("min_eigenvalue").get<double>());
                s.max_symmetry = std::max(s.max_symmetry, d.at("symmetry_error").get<double>());
                s.max_cross =
                    std::max(s.max_cross, d.at("cross_node_covariance_norm").get<double>());
                s.max_joint_cross = std::max(
                    s.max_joint_cross, d.at("node_joint_cross_covariance_norm").get<double>());
                covariance << m.time << ',' << name << ',' << d.at("dimension") << ','
                           << d.at("min_eigenvalue") << ',' << d.at("symmetry_error") << ','
                           << d.at("cross_node_covariance_norm") << ','
                           << d.at("node_joint_cross_covariance_norm") << ','
                           << d.at("position_std_m") << '\n';
            }
            if (filter->P.diagonal().minCoeff() < -1e-8)
                throw std::runtime_error("Negative covariance diagonal");
            if (c.at("joint_filter").at("participating_imus").size() == 6)
            {
                std::array<double, 5> estimated_angles = filter->angles;
                if (!filter->shape_state())
                    for (int j = 0; j < 5; ++j)
                        estimated_angles[j] = joint_angle_from_relative(
                            model.joints[j],
                            nodes[j].attitude.q.conjugate() * nodes[j + 1].attitude.q,
                            shape_joint_prior[name][j]);
                shape_joint_prior[name] = estimated_angles;
                ShapeState actual;
                for (int i = 0; i < 6; ++i)
                {
                    actual.position[i] = nodes[i].p - nodes[0].p;
                    actual.rotation[i] = nodes[i].attitude.q;
                }
                actual.terminal = actual.position[5] + actual.rotation[5] * model.offsets[5];
                shape_metric(name, estimated_angles, actual);
            }
        }
        if (detail && k % stride == 0)
        {
            frames << frame.dump() << '\n';
        }
    }
    Json summary = {{"status", "completed"},
                    {"samples", data.size()},
                    {"noise_mode", noise_mode},
                    {"config", c},
                    {"metrics", Json::object()},
                    {"filter_diagnostics", Json::object()},
                    {"shape_metrics", Json::object()},
                    {"runtime", Json::object()},
                    {"metric_contract", "Unaligned world-frame ATE; 1s world translation-increment "
                                        "RPE; B2_Si evaluated at its own body origin"}};
    summary["B1_statistics"] = {{"zupt_requested", zupt_requested},
                                {"zupt_accepted", zupt_accepted}};
    for (auto& [name, metric] : metrics)
        summary["metrics"][name] = metric.json();
    for (auto& [name, s] : stats)
        summary["filter_diagnostics"][name] = s.json();
    summary["filter_diagnostics"]["B2_independent"] = independent_bias_stats.json();
    for (auto& [name, squares] : joint_squares)
    {
        Json angle = Json::array(), position_rmse = Json::array();
        for (int j = 0; j < 5; ++j)
            angle.push_back({{"rmse_rad", std::sqrt(squares[j] / data.size())},
                             {"max_error_rad", joint_max[name][j]},
                             {"final_error_rad", joint_final[name][j]}});
        for (double x : shape_squares[name])
            position_rmse.push_back(std::sqrt(x / data.size()));
        summary["shape_metrics"][name] = {
            {"joint_angles", angle},
            {"panel_relative_rmse_m", position_rmse},
            {"terminal_relative_rmse_m", std::sqrt(terminal_shape_square[name] / data.size())}};
    }
    for (auto& [name, filter] : filters)
    {
        auto& t = filter->runtime;
        summary["runtime"][name] = {
            {"propagation_mean_us", 1e6 * t.propagation_s / t.propagation_count},
            {"joint_update_mean_us", t.update_count ? 1e6 * t.total_update_s / t.update_count : 0},
            {"position_gate_total_s", t.position_update_s},
            {"axis_gate_total_s", t.axis_update_s},
            {"total_propagation_s", t.propagation_s},
            {"total_update_s", t.total_update_s},
            {"update_count", t.update_count},
            {"state_dimension", filter->P.rows()}};
    }
    struct rusage resource;
    getrusage(RUSAGE_SELF, &resource);
    summary["runtime"]["process_peak_rss_mib"] = resource.ru_maxrss / 1024.;
    summary["runtime"]["wall_s"] = std::chrono::duration<double>(Clock::now() - start).count();
    for (auto file :
         {&innovations, &corrections, &covariance, &biases, &navigation, &joints, &shapes})
        if (!*file)
            throw std::runtime_error("Experiment write failed");
    std::ofstream report(folder + "/summary.json");
    report << summary.dump(2);
    if (!report)
        throw std::runtime_error("Summary write failed");
    return summary;
}
} // namespace
Json run_multi_imu_experiment(const Json& input, const std::string& path, const std::string& output,
                              const std::string& model_path, bool suite)
{
    const Json c = normalize_multi_imu_config(input);
    auto data = read_simulation(path, c.at("sample_rate_hz"));
    auto model = load_robot_model(model_path, c.at("robot").at("geometry"));
    if (!suite)
        return scenario(c, data, model, output, c.value("active_noise_mode", std::string("full")));
    struct Case
    {
        Json config;
        std::string id, noise, category, dataset;
    };
    std::vector<Case> cases;
    for (auto seed : c.at("experiment").at("seeds"))
        for (auto mode : c.at("experiment").at("noise_modes"))
        {
            Json cfg = c;
            cfg["noise"]["seed"] = seed;
            std::string n = mode;
            cases.push_back(
                {cfg, "nominal_" + std::to_string(seed.get<int>()) + "_" + n, n, "nominal"});
        }
    for (auto profile : c.at("experiment").at("noise_profiles"))
        for (auto seed : c.at("experiment").at("sensitivity_seeds"))
        {
            Json cfg = c;
            cfg["noise"]["seed"] = seed;
            cfg["noise"]["common_bias_correlation"] = profile.value("common_bias_correlation", 0.);
            for (auto key : {"gyro_initial_bias_bound_deg_s", "accel_initial_bias_bound_m_s2",
                             "gyro_bias_rw_deg_s_sqrt_s", "accel_bias_rw_m_s2_sqrt_s"})
                cfg["noise"][key] =
                    c.at("noise").at(key).get<double>() * profile.value("bias_scale", 1.);
            cfg["noise"]["gyro_white_std_deg_s"] =
                c.at("noise").at("gyro_white_std_deg_s").get<double>() *
                profile.value("gyro_scale", 1.);
            cfg["noise"]["accel_white_std_m_s2"] =
                c.at("noise").at("accel_white_std_m_s2").get<double>() *
                profile.value("accel_scale", 1.);
            std::string name = profile.at("name");
            cases.push_back({cfg, name + "_" + std::to_string(seed.get<int>()), "full", name});
        }
    for (auto subset : c.at("experiment").at("imu_subsets"))
    {
        if (subset.size() == 6)
            continue;
        Json cfg = c;
        cfg["noise"]["seed"] = 42;
        cfg["joint_filter"].erase("imu_count");
        cfg["joint_filter"].erase("compact_prefix_state");
        cfg["joint_filter"]["participating_imus"] = subset;
        std::string name = "subset";
        for (auto i : subset)
            name += "_" + std::to_string(i.get<int>());
        cases.push_back({cfg, name, "full", "imu_subset"});
    }
    for (auto rate : c.at("experiment").at("update_rates_hz"))
    {
        if (rate == c.at("joint_filter").at("update_rate_hz"))
            continue;
        Json cfg = c;
        cfg["noise"]["seed"] = 42;
        cfg["joint_filter"]["update_rate_hz"] = rate;
        cases.push_back({cfg, "rate_" + std::to_string(rate.get<int>()), "full", "update_rate"});
    }
    std::map<std::string, std::vector<SimulationFrame>> extra_data;
    if (c.at("experiment").value("run_synthetic", true))
    {
        for (auto& file : generate_synthetic_motion(c, model, output + "/synthetic_data"))
        {
            std::string scene = std::filesystem::path(file).stem().string();
            extra_data[scene] = read_simulation(file, c.at("sample_rate_hz"));
            for (auto seed : c.at("experiment").at("seeds"))
            {
                Json cfg = c;
                cfg["noise"]["seed"] = seed;
                cases.push_back({cfg, "synthetic_" + scene + "_" + std::to_string(seed.get<int>()),
                                 "full", "synthetic", scene});
            }
        }
    }
    std::filesystem::create_directories(output);
    std::vector<Json> summaries(cases.size());
    std::atomic<size_t> next{0}, finished{0};
    std::mutex print_mutex;
    int workers = c.at("experiment").value("workers", 2);
    std::vector<std::thread> threads;
    for (int worker = 0; worker < workers; ++worker)
        threads.emplace_back(
            [&]()
            {
                while (true)
                {
                    size_t index = next.fetch_add(1);
                    if (index >= cases.size())
                        break;
                    auto& item = cases[index];
                    try
                    {
                        summaries[index] = scenario(
                            item.config, item.dataset.empty() ? data : extra_data.at(item.dataset),
                            model, output + "/runs/" + item.id, item.noise);
                        summaries[index]["case_id"] = item.id;
                        summaries[index]["category"] = item.category;
                        summaries[index]["dataset"] =
                            item.dataset.empty() ? "original_recording" : item.dataset;
                    }
                    catch (const std::exception& error)
                    {
                        summaries[index] = {{"status", "failed"},
                                            {"error", error.what()},
                                            {"case_id", item.id},
                                            {"category", item.category}};
                    }
                    size_t done = ++finished;
                    std::lock_guard<std::mutex> lock(print_mutex);
                    std::cout << "Completed " << done << "/" << cases.size() << ": " << item.id
                              << " (" << summaries[index].at("status").get<std::string>() << ")\n"
                              << std::flush;
                }
            });
    for (auto& thread : threads)
        thread.join();
    Json report = {{"config", c},
                   {"data_file", std::filesystem::absolute(path).string()},
                   {"model_file", std::filesystem::absolute(model_path).string()},
                   {"cases", summaries},
                   {"case_count", cases.size()},
                   {"workers", workers},
                   {"D4_status", c.at("experiment").value("run_d4", true)
                                     ? "implemented_with_shared_gyro_noise"
                                     : "disabled"}};
    std::ofstream out(output + "/summary.json");
    out << report.dump(2);
    return report;
}
} // namespace origami
