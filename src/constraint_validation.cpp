#include "joint_estimator.h"
#include "multi_imu_eskf.h"
#include "preprocess.h"
#include "simulation_data.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
using namespace origami;
namespace
{
struct Moments
{
    size_t count = 0, accepted = 0, posterior_increase = 0;
    double pre2 = 0, post2 = 0, nis = 0, max_nis = 0, hph = 0, r = 0, max_post = 0;
    Json json() const
    {
        double n = std::max(size_t(1), count);
        return {{"count", count},
                {"accepted", accepted},
                {"rejected", count - accepted},
                {"pre_rmse", std::sqrt(pre2 / n)},
                {"post_rmse", std::sqrt(post2 / n)},
                {"mean_nis", nis / n},
                {"max_nis", max_nis},
                {"mean_trace_HPH", hph / n},
                {"mean_trace_R", r / n},
                {"posterior_increase_count", posterior_increase},
                {"max_post_norm", max_post}};
    }
};
Json bias_baseline(Json config, const Model& model, const std::vector<SimulationFrame>& data,
                   int seed, const std::string& profile)
{
    config["noise"]["seed"] = seed;
    for (auto key : profile == "gyro_bias_double"
                        ? std::vector<std::string>{"gyro_initial_bias_bound_deg_s",
                                                   "gyro_bias_rw_deg_s_sqrt_s"}
                        : std::vector<std::string>{"accel_initial_bias_bound_m_s2",
                                                   "accel_bias_rw_m_s2_sqrt_s"})
        config["noise"][key] = 2 * config["noise"][key].get<double>();
    ImuNoise noise(config["noise"], "full");
    MultiImuEskf initial(config, model);
    std::vector<ErrorStateKalmanFilter> nodes;
    for (int i = 0; i < 6; ++i)
    {
        nodes.emplace_back(config, false, false, false, true);
        nodes.back().p = initial.nodes[i].p;
        nodes.back().v = initial.nodes[i].v;
        nodes.back().attitude.q = initial.nodes[i].attitude.q;
    }
    JointEstimator a2(model, config, "A2", initial.nodes[0].attitude.q);
    std::map<std::string, double> squares, rpes, last;
    std::map<std::string, size_t> count;
    std::map<std::string, std::deque<std::pair<V3D, V3D>>> history;
    for (const auto& frame : data)
    {
        auto m = noise.apply(frame.imu, .001).measured;
        for (int i = 0; i < 6; ++i)
        {
            Sample in;
            in.gyro = m.gyro[i];
            in.force = m.force[i];
            bool valid = quality(in, config["robot"]["quality"], config["gravity_m_s2"], in.force);
            nodes[i].propagate(in.gyro, in.force, .001, valid);
        }
        auto joint = a2.update(m, .001);
        auto shape = forward_shape(model, nodes[0].attitude.q, joint.angle);
        V3D mean = V3D::Zero();
        for (int i = 0; i < 6; ++i)
            mean += (nodes[i].p - shape.position[i]) / 6;
        for (auto name : {"D0", "D1"})
        {
            V3D p = std::string(name) == "D0" ? nodes[0].p : mean;
            V3D truth = frame.truth.markers[0];
            double e = (p - truth).norm();
            squares[name] += e * e;
            last[name] = e;
            auto& h = history[name];
            h.push_back({p, truth});
            if (h.size() > 1000)
            {
                rpes[name] += ((p - h.front().first) - (truth - h.front().second)).squaredNorm();
                ++count[name];
                h.pop_front();
            }
        }
    }
    Json out;
    for (auto name : {"D0", "D1"})
        out[name] = {{"ate_m", std::sqrt(squares[name] / data.size())},
                     {"rpe_m", std::sqrt(rpes[name] / count[name])},
                     {"final_error_m", last[name]}};
    return out;
}
Json run(Json config, const Model& model, const std::vector<SimulationFrame>& data,
         const std::string& mode, int seed, const std::string& profile, double ps, double as,
         const std::string& output)
{
    config["noise"]["seed"] = seed;
    if (profile == "gyro_bias_double")
        for (auto key : {"gyro_initial_bias_bound_deg_s", "gyro_bias_rw_deg_s_sqrt_s"})
            config["noise"][key] = 2 * config["noise"][key].get<double>();
    if (profile == "accel_bias_double")
        for (auto key : {"accel_initial_bias_bound_m_s2", "accel_bias_rw_m_s2_sqrt_s"})
            config["noise"][key] = 2 * config["noise"][key].get<double>();
    config["joint_filter"]["position_std_m"] = .001 * ps;
    config["joint_filter"]["axis_std_rad"] = .01 * as;
    Json result = {{"noise_mode", mode},   {"seed", seed},     {"profile", profile},
                   {"position_scale", ps}, {"axis_scale", as}, {"methods", Json::object()}};
    bool detail = seed == 42 && ps == 1 && as == 1 && profile == "nominal";
    ImuNoise noise(config["noise"], mode);
    std::map<std::string, std::unique_ptr<MultiImuEskf>> filters;
    std::map<std::string, std::map<std::string, Moments>> moments;
    std::map<std::string, double> square, rpe, last, min_eigen, max_sym, max_cross, max_nis_error,
        max_joseph_error, max_feedback_error, max_bias_effect;
    std::map<std::string, std::deque<std::pair<V3D, V3D>>> history;
    std::map<std::string, size_t> rpe_count, updates;
    for (auto name : {"D3a", "D3b"})
    {
        Json c = config;
        c["joint_filter"]["enable_joint_position_constraint"] = true;
        c["joint_filter"]["enable_joint_axis_constraint"] = std::string(name) == "D3b";
        c["joint_filter"]["enable_shape_state"] = false;
        filters[name] = std::make_unique<MultiImuEskf>(c, model);
        min_eigen[name] = INFINITY;
    }
    std::ofstream log;
    if (detail)
    {
        log.open(output + "/residuals_" + mode + ".csv");
        log.precision(15);
        log << "time_s,method,kind,joint,pre_norm,post_norm,nis,accepted,trace_HPH,trace_R\n";
    }
    for (size_t k = 0; k < data.size(); ++k)
    {
        auto sample = noise.apply(data[k].imu, .001).measured;
        for (auto& [name, ptr] : filters)
        {
            auto& f = *ptr;
            // Independently exercise the corrected node through the next propagation.
            if (k > 10 && k < 100 && (k + 1) % 10 == 1)
            {
                auto expected = f.nodes[0], without_bias = expected;
                Sample in;
                in.gyro = sample.gyro[0];
                in.force = sample.force[0];
                bool valid =
                    quality(in, config["robot"]["quality"], config["gravity_m_s2"], in.force);
                double multiplier =
                    valid ? 1 : config["robot"]["quality"]["impact_noise_multiplier"].get<double>();
                expected.propagate(in.gyro, in.force, .001, valid, multiplier);
                without_bias.bg.setZero();
                without_bias.ba.setZero();
                without_bias.propagate(in.gyro, in.force, .001, valid, multiplier);
                f.propagate(sample, .001);
                max_feedback_error[name] =
                    std::max(max_feedback_error[name], (expected.p - f.nodes[0].p).norm());
                max_bias_effect[name] =
                    std::max(max_bias_effect[name],
                             (without_bias.v - f.nodes[0].v).norm() +
                                 without_bias.attitude.q.angularDistance(f.nodes[0].attitude.q));
            }
            else
                f.propagate(sample, .001);
            if ((k + 1) % 10 == 0)
            {
                std::vector<Constraint> before;
                for (int j = 0; j < 5; ++j)
                {
                    before.push_back(hinge_position(f.nodes, model.joints[j], j, 90));
                    if (name == "D3b")
                        before.push_back(hinge_axis(f.nodes, model.joints[j], j, 90));
                }
                Eigen::MatrixXd prior = f.P;
                f.update(sample.time);
                ++updates[name];
                int total = 0;
                for (size_t j = 0; j < before.size(); ++j)
                {
                    auto& b = before[j];
                    auto& record = f.last_innovations.at(j);
                    auto after =
                        b.kind == "position"
                            ? hinge_position(f.nodes, model.joints[b.joint], b.joint, 90)
                            : hinge_axis(f.nodes, model.joints[b.joint], b.joint, 90, &b.tangent);
                    double sigma = b.kind == "position" ? .001 * ps : .01 * as;
                    Eigen::MatrixXd hph = b.H * prior * b.H.transpose(), S = hph;
                    S.diagonal().array() += sigma * sigma;
                    double nis = b.value.dot(S.ldlt().solve(b.value));
                    max_nis_error[name] = std::max(max_nis_error[name], std::abs(nis - record.nis));
                    if ((record.innovation + b.value).norm() > 1e-12)
                        throw std::runtime_error("Innovation was not evaluated at prediction");
                    auto& v = moments[name][b.kind];
                    ++v.count;
                    v.accepted += record.accepted;
                    v.pre2 += b.value.squaredNorm();
                    v.post2 += after.value.squaredNorm();
                    v.nis += nis;
                    v.max_nis = std::max(v.max_nis, nis);
                    v.hph += hph.trace();
                    v.r += sigma * sigma * b.value.size();
                    v.max_post = std::max(v.max_post, after.value.norm());
                    v.posterior_increase += after.value.norm() > b.value.norm() + 1e-12;
                    if (record.accepted)
                        total += b.value.size();
                    if (detail)
                        log << sample.time << ',' << name << ',' << b.kind << ',' << b.joint << ','
                            << b.value.norm() << ',' << after.value.norm() << ',' << nis << ','
                            << record.accepted << ',' << hph.trace() << ','
                            << sigma * sigma * b.value.size() << '\n';
                }
                // Check the factored Joseph against the direct expression, including reset.
                if ((k == 9 || k == 10859) && total)
                {
                    Eigen::MatrixXd H(total, 90);
                    Eigen::VectorXd nu(total), variance(total);
                    int row = 0;
                    for (size_t j = 0; j < before.size(); ++j)
                        if (f.last_innovations[j].accepted)
                        {
                            auto& b = before[j];
                            int n = b.value.size();
                            H.middleRows(row, n) = b.H;
                            nu.segment(row, n) = -b.value;
                            double sigma = b.kind == "position" ? .001 * ps : .01 * as;
                            variance.segment(row, n).setConstant(sigma * sigma);
                            row += n;
                        }
                    Eigen::MatrixXd S =
                        H * prior * H.transpose() + variance.asDiagonal().toDenseMatrix();
                    Eigen::MatrixXd K = S.ldlt().solve(H * prior).transpose();
                    Eigen::VectorXd dx = K * nu;
                    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(90, 90) - K * H;
                    Eigen::MatrixXd post =
                        A * prior * A.transpose() + K * variance.asDiagonal() * K.transpose();
                    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(90, 90);
                    for (int i = 0; i < 6; ++i)
                        J.block<3, 3>(15 * i + 6, 15 * i + 6) =
                            right_jacobian(dx.segment<3>(15 * i + 6));
                    post = (J * post * J.transpose()).eval();
                    max_joseph_error[name] =
                        std::max(max_joseph_error[name], (post - f.P).cwiseAbs().maxCoeff());
                    if ((dx - f.last_correction).norm() > 1e-9)
                        throw std::runtime_error("State correction disagrees with reference batch");
                }
            }
            V3D truth = data[k].truth.markers[0];
            double e = (f.nodes[0].p - truth).norm();
            square[name] += e * e;
            last[name] = e;
            auto& h = history[name];
            h.push_back({f.nodes[0].p, truth});
            if (h.size() > 1000)
            {
                rpe[name] +=
                    ((f.nodes[0].p - h.front().first) - (truth - h.front().second)).squaredNorm();
                ++rpe_count[name];
                h.pop_front();
            }
            if (k % 1000 == 0 || k + 1 == data.size())
            {
                auto diagnostic = f.diagnostics();
                min_eigen[name] =
                    std::min(min_eigen[name], diagnostic["min_eigenvalue"].get<double>());
                max_sym[name] = std::max(max_sym[name], diagnostic["symmetry_error"].get<double>());
                max_cross[name] = std::max(max_cross[name],
                                           diagnostic["cross_node_covariance_norm"].get<double>());
            }
        }
    }
    for (auto& [name, ptr] : filters)
    {
        Json constraints = Json::object();
        for (auto& [kind, value] : moments[name])
            constraints[kind] = value.json();
        result["methods"][name] = {{"ate_m", std::sqrt(square[name] / data.size())},
                                   {"rpe_m", std::sqrt(rpe[name] / rpe_count[name])},
                                   {"final_error_m", last[name]},
                                   {"constraints", constraints},
                                   {"update_count", updates[name]},
                                   {"min_eigenvalue", min_eigen[name]},
                                   {"max_symmetry_error", max_sym[name]},
                                   {"max_cross_covariance_norm", max_cross[name]},
                                   {"max_nis_recomputation_error", max_nis_error[name]},
                                   {"max_joseph_reset_error", max_joseph_error[name]},
                                   {"max_next_propagation_error_m", max_feedback_error[name]},
                                   {"max_bias_effect_next_step", max_bias_effect[name]}};
        if (max_nis_error[name] > 1e-8 || max_joseph_error[name] > 1e-8 ||
            max_feedback_error[name] > 1e-12)
            throw std::runtime_error("Constraint audit failed");
    }
    return result;
}
} // namespace
int main(int argc, char** argv)
{
    try
    {
        if (argc != 3 && argc != 4)
            return 2;
        std::string root = argv[1], output = argv[2];
        std::filesystem::create_directories(output);
        Json config;
        std::ifstream(root + "/config/multi_imu_eskf.json") >> config;
        auto model =
            load_robot_model(root + "/robot/zbot_crack_L2.xml", config["robot"]["geometry"]);
        auto data = read_simulation(root + "/data/data_L_qawp_20261030.txt", 1000);
        if (argc == 4)
        {
            if (std::string(argv[3]) != "--bias-baselines-only")
                throw std::invalid_argument("Unknown argument");
            Json existing;
            std::ifstream(output + "/audit.json") >> existing;
            for (auto& value : existing["cases"])
                if (value["profile"] != "nominal")
                    value["baselines"] =
                        bias_baseline(config, model, data, value["seed"], value["profile"]);
            std::ofstream(output + "/audit.json") << existing.dump(2);
            return 0;
        }
        Json cases = Json::array();
        auto append =
            [&](const std::string& mode, int seed, const std::string& profile, double ps, double as)
        {
            auto value = run(config, model, data, mode, seed, profile, ps, as, output);
            if (profile != "nominal")
                value["baselines"] = bias_baseline(config, model, data, seed, profile);
            cases.push_back(value);
            std::ofstream(output + "/audit.json") << Json({{"cases", cases},
                                                           {"samples", data.size()},
                                                           {"sensitivity_seeds", {42, 55, 71}}})
                                                         .dump(2);
            std::cout << "Completed " << cases.size() << ": " << mode << ' ' << seed << ' '
                      << profile << ' ' << ps << ' ' << as << std::endl;
        };
        for (auto mode : {"none", "white", "fixed_bias"})
            append(mode, 42, "nominal", 1, 1);
        for (int seed : {42, 55, 71})
        {
            for (auto scale : {std::pair<double, double>{1, 1},
                               {.5, 1},
                               {2, 1},
                               {1, .5},
                               {1, 2},
                               {.5, .5},
                               {2, 2}})
                append("full", seed, "nominal", scale.first, scale.second);
            append("full", seed, "gyro_bias_double", 1, 1);
            append("full", seed, "accel_bias_double", 1, 1);
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
