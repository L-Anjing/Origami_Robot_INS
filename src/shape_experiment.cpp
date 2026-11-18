#include "shape_experiment.h"
#include "joint_estimator.h"
#include "multi_imu_fusion.h"
#include "preprocess.h"
#include "simulation_data.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
namespace origami
{
namespace
{
Quaterniond quaternion(const Json& q)
{
    if (!q.is_array() || q.size() != 4)
        throw std::invalid_argument("Expected wxyz quaternion prior");
    Quaterniond out(q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>());
    if (!out.coeffs().allFinite() || out.norm() < 1e-12)
        throw std::invalid_argument("Invalid attitude prior");
    return out.normalized();
}
template <size_t N> std::array<double, N> doubles(const Json& a)
{
    if (!a.is_array() || a.size() != N)
        throw std::invalid_argument("Invalid prior vector size");
    std::array<double, N> out;
    for (size_t i = 0; i < N; ++i)
    {
        out[i] = a[i];
        if (!std::isfinite(out[i]))
            throw std::invalid_argument("Nonfinite prior");
    }
    return out;
}
Json vectors(const std::array<V3D, 6>& input)
{
    Json out = Json::array();
    for (auto& v : input)
        out.push_back(array(v));
    return out;
}
struct Errors
{
    double square = 0, maximum = 0, last = 0;
    size_t count = 0;
    void push(double e)
    {
        if (!std::isfinite(e))
            throw std::runtime_error("Nonfinite metric");
        square += e * e;
        maximum = std::max(maximum, std::abs(e));
        last = e;
        ++count;
    }
    Json json() const
    {
        return {{"rmse", count ? Json(std::sqrt(square / count)) : Json(nullptr)},
                {"max_abs", maximum},
                {"final", last},
                {"samples", count}};
    }
};
struct NavigationMetric
{
    Errors ate, rpe, terminal;
    std::deque<std::pair<V3D, V3D>> history;
    void update(const V3D& estimate, const V3D& truth, size_t lag)
    {
        ate.push((estimate - truth).norm());
        history.push_back({estimate, truth});
        if (history.size() > lag)
        {
            auto& before = history.front();
            rpe.push(((estimate - before.first) - (truth - before.second)).norm());
            history.pop_front();
        }
    }
    Json json() const
    {
        return {{"ate_m", ate.json()},
                {"rpe_translation_m", rpe.json()},
                {"terminal_ate_m", terminal.json()}};
    }
};
struct Trial
{
    std::string name;
    JointEstimator estimator;
    std::vector<ErrorStateKalmanFilter> filters;
    std::vector<StationaryDetector> detectors;
    std::array<double, 6> attempts;
    std::array<Errors, 5> joints;
    std::array<Errors, 6> shape_errors, local_shape_errors;
    Errors terminal, physical, attitude;
    std::array<Errors, 6> independent_attitude;
    std::map<std::string, NavigationMetric> navigation;
    size_t anomalies = 0, accepted = 0;
    double max_ideal_joint = 0, max_ideal_position = 0;
    Trial(const std::string& mode, const Model& model, const Json& c, const Quaterniond& q,
          const Calibration& calibration)
        : name(mode), estimator(model, c, mode, q, mode == "A1" ? calibration : Calibration{})
    {
        auto angles = doubles<5>(c.at("initial_joint_angles_rad")),
             rates = doubles<5>(c.at("initial_joint_rates_rad_s"));
        auto initial = forward_shape(model, q, angles);
        auto velocities = initial_node_velocities(
            model, initial, rates, q * vector3(c.at("initial_root_angular_velocity_rad_s")),
            vector3(c.at("initial_velocity_m_s")));
        for (int i = 0; i < 6; ++i)
        {
            Json config = c;
            if (mode == "A1")
            {
                config["robot"]["eskf"]["initial_gyro_bias_rad_s"] = array(calibration.gyro[i]);
                config["robot"]["eskf"]["initial_accel_bias_m_s2"] = array(calibration.accel[i]);
            }
            filters.emplace_back(config, true, false, false, !c.value("root_ahrs_feedback", false));
            auto& f = filters.back();
            f.p = vector3(c.at("robot").at("initial_root_W_m")) + initial.position[i];
            f.v = velocities[i];
            f.attitude.q = initial.rotation[i];
            detectors.emplace_back(c.at("robot").at("stationary"), c.at("gravity_m_s2"),
                                   1 / c.at("sample_rate_hz").get<double>(), false);
            attempts[i] = -INFINITY;
        }
    }
};
Json input_consistency(const std::vector<SimulationFrame>& data, const Model& model, double dt,
                       double gravity)
{
    std::array<Errors, 6> acceleration_residual, velocity_residual, attitude_residual;
    std::array<V3D, 6> velocity;
    velocity.fill(V3D::Zero());
    std::array<SixAxisAhrs, 6> integrated_attitude;
    // Audit in the documented t=0 identity/zero-rate experiment. This never modifies sensor input.
    for (size_t k = 0; k < data.size(); ++k)
    {
        const auto& frame = data[k];
        std::array<V3D, 6> analytic;
        for (int i = 0; i < 6; ++i)
        {
            V3D a = frame.truth.orientation[i] * frame.imu.force[i] + V3D(0, 0, -gravity);
            velocity[i] += a * dt;
            if (i == 0)
                analytic[0] = velocity[0];
            else
            {
                V3D omega = frame.truth.orientation[i - 1] * frame.imu.gyro[i - 1];
                analytic[i] = analytic[i - 1] +
                              omega.cross(frame.truth.orientation[i - 1] * model.offsets[i - 1]);
            }
            velocity_residual[i].push((velocity[i] - analytic[i]).norm());
            integrated_attitude[i].q =
                (integrated_attitude[i].q * increment(frame.imu.gyro[i] * dt)).normalized();
            attitude_residual[i].push(
                integrated_attitude[i].q.angularDistance(frame.truth.orientation[i]));
            if (k >= 2)
            {
                V3D discrete = (frame.truth.markers[i] - 2 * data[k - 1].truth.markers[i] +
                                data[k - 2].truth.markers[i]) /
                               (dt * dt);
                acceleration_residual[i].push((a - discrete).norm());
            }
        }
    }
    Json out = {{"force_vs_marker_second_difference_m_s2", Json::array()},
                {"integrated_force_velocity_vs_instantaneous_FK_m_s", Json::array()},
                {"independent_gyro_attitude_error_rad", Json::array()},
                {"interpretation",
                 "At non-root nodes, instantaneous articulated velocity is not identical to "
                 "end-step finite-difference marker velocity. Position integration and local gyro "
                 "exponential integration carry discretization errors even for ideal input. Audit "
                 "is evaluation-only; no fitted truth correction is applied."}};
    for (int i = 0; i < 6; ++i)
    {
        out["force_vs_marker_second_difference_m_s2"].push_back(acceleration_residual[i].json());
        out["integrated_force_velocity_vs_instantaneous_FK_m_s"].push_back(
            velocity_residual[i].json());
        out["independent_gyro_attitude_error_rad"].push_back(attitude_residual[i].json());
    }
    return out;
}
Json validate_shape_ideal(const std::vector<SimulationFrame>& data, const Model& model,
                          const Json& c)
{
    auto root_prior = quaternion(c.at("initial_root_quaternion_wxyz"));
    JointEstimator estimator(model, c, "A0", root_prior);
    SixAxisAhrs root;
    root.q = root_prior;
    std::array<double, 5> reference = doubles<5>(c.at("initial_joint_angles_rad"));
    double max_joint = 0, max_position = 0, max_orientation = 0, low = 0;
    double dt = 1 / c.at("sample_rate_hz").get<double>();
    for (auto& frame : data)
    {
        auto joint = estimator.update(frame.imu, dt);
        root.update(frame.imu.gyro[0], frame.imu.force[0], dt, c.at("robot").at("ahrs"),
                    c.at("gravity_m_s2"), false, true);
        auto shape = forward_shape(model, root.q, joint.angle);
        for (int j = 0; j < 5; ++j)
        {
            reference[j] = joint_angle_from_relative(model.joints[j],
                                                     frame.truth.orientation[j].conjugate() *
                                                         frame.truth.orientation[j + 1],
                                                     reference[j]);
            max_joint = std::max(max_joint, std::abs(joint.angle[j] - reference[j]));
            low = std::min(low, reference[j] - model.joints[j].lower);
        }
        for (int i = 0; i < 6; ++i)
        {
            max_position = std::max(
                max_position,
                (shape.position[i] - (frame.truth.markers[i] - frame.truth.markers[0])).norm());
            max_orientation = std::max(
                max_orientation, shape.rotation[i].angularDistance(frame.truth.orientation[i]));
        }
    }
    bool passed =
        max_joint <= c.at("fusion").at("shape_validation_max_joint_rad").get<double>() &&
        max_position <= c.at("fusion").at("shape_validation_max_position_m").get<double>();
    return {{"passed", passed},
            {"max_joint_error_rad", max_joint},
            {"max_relative_position_error_m", max_position},
            {"max_panel_orientation_error_rad", max_orientation},
            {"minimum_lower_limit_penetration_rad", low},
            {"samples", data.size()},
            {"truth_usage",
             "Evaluation only; gyro integration and configured t=0 prior produce estimates"}};
}
} // namespace
Json run_shape_experiment(const Json& c, const std::string& path, const std::string& output,
                          const std::string& model_path, bool shape_only)
{
    if (c.at("integration") != "step_end" || c.at("initial_velocity_epoch") != "simulation_start")
        throw std::invalid_argument(
            "Shape experiment uses documented step-end, simulation-start convention");
    auto data = read_simulation(path, c.at("sample_rate_hz"));
    auto model = load_robot_model(model_path, c.at("robot").at("geometry"));
    auto root_prior = quaternion(c.at("initial_root_quaternion_wxyz"));
    auto initial = forward_shape(model, root_prior, doubles<5>(c.at("initial_joint_angles_rad")));
    double dt = 1 / c.at("sample_rate_hz").get<double>();
    size_t lag =
        std::max(size_t(1), size_t(std::llround(c.at("rpe_interval_s").get<double>() / dt)));
    std::filesystem::create_directories(output);
    Json report = {{"schema_version", 1},
                   {"experiment", "shape-aware output-layer multi-IMU fusion"},
                   {"config", c},
                   {"data_file", std::filesystem::absolute(path).string()},
                   {"model_file", std::filesystem::absolute(model_path).string()},
                   {"samples", data.size()},
                   {"initialization_epoch_s", 0.},
                   {"ideal_validation", validate_shape_ideal(data, model, c)},
                   {"noise_modes", Json::object()},
                   {"joints", Json::array()}};
    for (auto& h : model.joints)
        report["joints"].push_back(
            {{"name", h.name},
             {"parent", h.parent},
             {"child", h.child},
             {"axis_parent", array(h.axis_parent)},
             {"axis_child", array(h.axis_child)},
             {"rest_quaternion_wxyz",
              {h.rest_rotation.w(), h.rest_rotation.x(), h.rest_rotation.y(), h.rest_rotation.z()}},
             {"translation_m", array(h.translation)},
             {"anchor_child_m", array(h.anchor_child)},
             {"lower_rad", h.lower},
             {"upper_rad", h.upper}});
    bool audit_supported = (root_prior.angularDistance(Quaterniond::Identity()) < 1e-12);
    for (auto& h : model.joints)
        audit_supported = audit_supported && h.anchor_child.norm() < 1e-12;
    audit_supported = audit_supported && vector3(c.at("initial_velocity_m_s")).norm() < 1e-12;
    for (double x : doubles<5>(c.at("initial_joint_angles_rad")))
        audit_supported = audit_supported && std::abs(x) < 1e-12;
    if (audit_supported)
        report["input_consistency_audit"] =
            input_consistency(data, model, dt, c.at("gravity_m_s2"));
    else
        report["input_consistency_audit"] = {
            {"status", "not_applicable"},
            {"reason", "Specialized documented zero-offset identity-prior logging audit; general "
                       "kinematics is tested separately"}};
    report["metric_definitions"] = {
        {"ATE",
         "sqrt(mean(||p_est-p_truth||^2)) in documented world frame; no trajectory alignment"},
        {"RPE", "sqrt(mean(||(p_est[k]-p_est[k-L])-(p_truth[k]-p_truth[k-L])||^2)); world-frame "
                "translational displacement error"},
        {"rpe_lag_samples", lag},
        {"rpe_actual_interval_s", lag * dt},
        {"panel_shape", "IMU/body origin relative to S0; estimated root orientation; separate "
                        "root-frame shape metric also saved"},
        {"terminal", "logged 50.1mm final offset; physical tip independently uses 50mm"}};
    bool density_definition =
        c.at("noise").value("white_noise_definition", std::string("per_sample_std")) == "density";
    double gyro_sigma = c.at("noise")
                            .at(density_definition ? "gyro_white_density_deg_s_per_sqrt_hz"
                                                   : "gyro_white_std_deg_s")
                            .get<double>() *
                        pi / 180;
    double accel_sigma = c.at("noise")
                             .at(density_definition ? "accel_white_density_m_s2_per_sqrt_hz"
                                                    : "accel_white_std_m_s2")
                             .get<double>();
    if (density_definition)
    {
        gyro_sigma /= std::sqrt(dt);
        accel_sigma /= std::sqrt(dt);
    }
    report["noise_discretization"] = {
        {"dt_s", dt},
        {"gyro_white_per_sample_std_rad_s", gyro_sigma},
        {"accel_white_per_sample_std_m_s2", accel_sigma},
        {"gyro_white_density_rad_s_per_sqrt_hz", gyro_sigma * std::sqrt(dt)},
        {"accel_white_density_m_s2_per_sqrt_hz", accel_sigma * std::sqrt(dt)},
        {"white_convention",
         "sigma_sample=density/sqrt(dt); integrated sample noise covariance=density^2*dt"},
        {"bias_walk_convention", "bias[k]=bias[k-1]+random_walk_density*sqrt(dt)*N(0,1)"},
        {"random_generator", "mt19937_64; explicit Box-Muller; independent draws per node/axis; "
                             "common noise realization across algorithms"}};
    report["estimator_contract"] = {
        {"sensor_only_input", true},
        {"normal_modes_use_truth", false},
        {"oracle", "True joint angles only; noisy IMUs and estimated S0 attitude remain unchanged"},
        {"initial_shape", "Explicit t=0 shape/attitude/velocity priors; gyro observations alone "
                          "cannot recover unknown initial folds"},
        {"fusion", "Output-layer D0/D1/D2, not a joint ESKF"},
        {"uncertainty", "Six estimates and shape share measurements; weights are inverse "
                        "conservative marginal trace proxies. Unknown cross-correlation is not "
                        "treated as independence; no posterior covariance reduction is claimed."}};
    // This gate precedes every navigation trial. A-C model errors cannot be concealed by fusion.
    if (!report["ideal_validation"]["passed"].get<bool>())
    {
        std::ofstream fail(output + "/summary.json");
        fail << report.dump(2);
        throw std::runtime_error("A-C ideal validation failed; D is not executed");
    }
    std::ofstream frames(output + "/frames.jsonl"), jcsv(output + "/joint_angles.csv"),
        scsv(output + "/shape_errors.csv"), ncsv(output + "/navigation_errors.csv");
    if (!frames || !jcsv || !scsv || !ncsv)
        throw std::runtime_error("Cannot create experiment outputs");
    jcsv << "noise,algorithm,time_s,joint,estimate_rad,truth_rad,error_rad,axis_residual_rad_s,"
            "orientation_residual_rad,valid\n";
    scsv << "noise,algorithm,time_s,panel,error_relative_world_m,error_relative_root_m,terminal_"
            "error_m,physical_tip_error_m,root_attitude_error_rad\n";
    ncsv << "noise,algorithm,time_s,method,x_m,y_m,z_m,truth_x_m,truth_y_m,truth_z_m,error_m,"
            "terminal_error_m\n";
    jcsv << std::setprecision(17);
    scsv << std::setprecision(17);
    ncsv << std::setprecision(17);
    for (const auto& noise_value : c.at("noise_modes"))
    {
        std::string noise_mode = noise_value;
        ImuNoise noise(c.at("noise"), noise_mode);
        std::vector<ImuMeasurements> measurements;
        std::vector<BiasTruth> biases;
        measurements.reserve(data.size());
        biases.reserve(data.size());
        for (auto& frame : data)
        {
            auto generated = noise.apply(frame.imu, dt);
            measurements.push_back(generated.measured);
            biases.push_back(generated.truth);
        }
        std::vector<ImuMeasurements> calibration_window;
        for (auto& m : measurements)
        {
            if (m.time - measurements.front().time <=
                c.at("calibration").at("duration_s").get<double>() + 1e-10)
                calibration_window.push_back(m);
            else
                break;
        }
        auto calibration = calibrate_initial_static(calibration_window, initial, c);
        Json mode_report = {{"calibration",
                             {{"available", calibration.available},
                              {"reason", calibration.reason},
                              {"samples", calibration_window.size()}}},
                            {"algorithms", Json::object()}};
        Json active = c;
        if (noise_mode == "none")
        {
            active["noise"]["gyro_white_std_deg_s"] = 0.;
            active["noise"]["accel_white_std_m_s2"] = 0.;
            active["noise"]["gyro_white_density_deg_s_per_sqrt_hz"] = 0.;
            active["noise"]["accel_white_density_m_s2_per_sqrt_hz"] = 0.;
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
        mode_report["effective_noise_config"] = active.at("noise");
        std::vector<std::unique_ptr<Trial>> trials;
        for (const auto& algorithm : c.at("joint_modes"))
        {
            std::string name = algorithm;
            if (name == "A1" && !calibration.available)
            {
                mode_report["algorithms"][name] = {{"status", "unavailable"},
                                                   {"reason", calibration.reason}};
                continue;
            }
            trials.push_back(std::make_unique<Trial>(name, model, active, root_prior, calibration));
        }
        std::array<double, 5> truth_angles = doubles<5>(c.at("initial_joint_angles_rad"));
        std::ofstream btruth(output + "/bias_truth_" + noise_mode + ".jsonl");
        for (size_t k = 0; k < data.size(); ++k)
        {
            // Evaluation branch: the estimator never receives SimulationFrame or BiasTruth.
            const auto& truth = data[k].truth;
            const auto& observed = measurements[k];
            for (int j = 0; j < 5; ++j)
                truth_angles[j] = joint_angle_from_relative(
                    model.joints[j], truth.orientation[j].conjugate() * truth.orientation[j + 1],
                    truth_angles[j]);
            btruth << Json({{"time_s", observed.time},
                            {"gyro_bias_rad_s", vectors(biases[k].gyro)},
                            {"accel_bias_m_s2", vectors(biases[k].accel)}})
                          .dump()
                   << '\n';
            for (auto& trial_ptr : trials)
            {
                auto& trial = *trial_ptr;
                auto joint = trial.estimator.update(observed, dt);
                std::array<V3D, 6> positions;
                std::array<Matrix15d, 6> covariance;
                for (int i = 0; i < 6; ++i)
                {
                    Sample sample;
                    sample.force = observed.force[i];
                    sample.gyro = observed.gyro[i];
                    sample.stamp = std::llround(observed.time * 1e9);
                    bool valid = quality(sample, c.at("robot").at("quality"), c.at("gravity_m_s2"),
                                         sample.force);
                    auto& f = trial.filters[i];
                    f.propagate(sample.gyro, sample.force, dt, valid,
                                valid ? 1
                                      : c.at("robot")
                                            .at("quality")
                                            .at("impact_noise_multiplier")
                                            .get<double>());
                    auto detection = trial.detectors[i].update(sample, valid, dt);
                    std::string policy = c.at("navigation_zupt");
                    if (policy != "none" && policy != "glrt")
                        throw std::invalid_argument(
                            "Experiment navigation_zupt must be none or glrt");
                    bool requested =
                        policy == "glrt" && detection.stationary &&
                        observed.time - trial.attempts[i] + 1e-10 >=
                            c.at("robot").at("eskf").at("zupt_min_interval_s").get<double>();
                    if (requested)
                        trial.attempts[i] = observed.time;
                    trial.accepted += f.zupt(requested, valid).accepted;
                    trial.independent_attitude[i].push(
                        f.attitude.q.angularDistance(truth.orientation[i]));
                    positions[i] = f.p;
                    covariance[i] = f.P;
                    if (k % 1000 == 0 || k + 1 == data.size())
                        f.check_covariance();
                }
                auto& root = trial.filters[0].attitude.q;
                auto shape = forward_shape(model, root, joint.angle);
                auto local = forward_shape(model, Quaterniond::Identity(), joint.angle);
                double attitude_error = root.angularDistance(truth.orientation[0]);
                trial.attitude.push(attitude_error);
                double terminal_error =
                    (shape.terminal - (truth.markers[6] - truth.markers[0])).norm();
                trial.terminal.push(terminal_error);
                V3D physical_truth = truth.markers[5] - truth.markers[0] +
                                     truth.orientation[5] * V3D(model.physical_length_m, 0, 0);
                double physical_error = (shape.physical_tip - physical_truth).norm();
                trial.physical.push(physical_error);
                Json frame = {{"noise", noise_mode},
                              {"algorithm", trial.name},
                              {"time_s", observed.time},
                              {"estimated_joint_angles_rad", joint.angle},
                              {"evaluation_joint_truth_rad", truth_angles},
                              {"joint_valid", joint.valid},
                              {"joint_axis_residual_rad_s", joint.axis_residual},
                              {"relative_position_W_m", vectors(shape.position)},
                              {"root_quaternion_wxyz", {root.w(), root.x(), root.y(), root.z()}},
                              {"navigation", Json::object()}};
                frame["joint_variance_proxy_rad2"] = joint.variance;
                frame["independent_positions_W_m"] = vectors(positions);
                frame["estimated_gyro_bias_rad_s"] = Json::array();
                frame["estimated_accel_bias_m_s2"] = Json::array();
                for (auto& filter : trial.filters)
                {
                    frame["estimated_gyro_bias_rad_s"].push_back(array(filter.bg));
                    frame["estimated_accel_bias_m_s2"].push_back(array(filter.ba));
                }
                for (int j = 0; j < 5; ++j)
                {
                    double error = joint.angle[j] - truth_angles[j];
                    trial.joints[j].push(error);
                    trial.anomalies += !joint.valid[j];
                    jcsv << noise_mode << ',' << trial.name << ',' << observed.time << ',' << j
                         << ',' << joint.angle[j] << ',' << truth_angles[j] << ',' << error << ','
                         << joint.axis_residual[j] << ',' << joint.orientation_residual[j] << ','
                         << joint.valid[j] << '\n';
                }
                for (int i = 0; i < 6; ++i)
                {
                    V3D relative_truth = truth.markers[i] - truth.markers[0];
                    double error = (shape.position[i] - relative_truth).norm(),
                           root_error = (local.position[i] -
                                         truth.orientation[0].conjugate() * relative_truth)
                                            .norm();
                    trial.shape_errors[i].push(error);
                    trial.local_shape_errors[i].push(root_error);
                    scsv << noise_mode << ',' << trial.name << ',' << observed.time << ',' << i
                         << ',' << error << ',' << root_error << ',' << terminal_error << ','
                         << physical_error << ',' << attitude_error << '\n';
                }
                if (!shape_only)
                {
                    auto fused =
                        fuse_positions(positions, covariance, model, shape, joint, c.at("fusion"));
                    std::map<std::string, V3D> estimates = {
                        {"D0", fused.single}, {"D1", fused.uniform}, {"D2", fused.weighted}};
                    V3D oracle_terminal = shape.terminal;
                    if (c.value("oracle_shape", true))
                    {
                        auto oracle_shape = forward_shape(model, root, truth_angles);
                        oracle_terminal = oracle_shape.terminal;
                        JointEstimate oracle_joint;
                        oracle_joint.valid.fill(true);
                        auto oracle = fuse_positions(positions, covariance, model, oracle_shape,
                                                     oracle_joint, c.at("fusion"));
                        estimates["ORACLE_D1"] = oracle.uniform;
                        estimates["ORACLE_D2"] = oracle.weighted;
                    }
                    for (auto& [method, estimate] : estimates)
                    {
                        trial.navigation[method].update(estimate, truth.markers[0], lag);
                        double terminal_error_absolute =
                            (estimate +
                             (method.rfind("ORACLE_", 0) == 0 ? oracle_terminal : shape.terminal) -
                             truth.markers[6])
                                .norm();
                        trial.navigation[method].terminal.push(terminal_error_absolute);
                        frame["navigation"][method] = array(estimate);
                        ncsv << noise_mode << ',' << trial.name << ',' << observed.time << ','
                             << method;
                        for (double x : estimate)
                            ncsv << ',' << x;
                        for (double x : truth.markers[0])
                            ncsv << ',' << x;
                        ncsv << ',' << (estimate - truth.markers[0]).norm() << ','
                             << terminal_error_absolute << '\n';
                    }
                    frame["fusion_weights"] = fused.weights;
                    frame["uncertainty_proxy_m2"] = fused.uncertainty_proxy_m2;
                    frame["correlation_aware_trace_bound_proxy_m2"] = fused.weighted_trace_bound_m2;
                }
                frames << frame.dump() << '\n';
            }
        }
        for (auto& trial_ptr : trials)
        {
            auto& trial = *trial_ptr;
            Json stats = {{"status", "completed"},
                          {"joint_angle_error_rad", Json::array()},
                          {"panel_relative_position_error_m", Json::array()},
                          {"panel_root_frame_position_error_m", Json::array()},
                          {"terminal_marker_error_m", trial.terminal.json()},
                          {"physical_tip_error_m", trial.physical.json()},
                          {"reference_attitude_error_rad", trial.attitude.json()},
                          {"joint_anomaly_count", trial.anomalies},
                          {"zupt_accepted_total", trial.accepted},
                          {"navigation", Json::object()}};
            stats["independent_attitude_error_rad"] = Json::array();
            for (auto& e : trial.independent_attitude)
                stats["independent_attitude_error_rad"].push_back(e.json());
            for (auto& e : trial.joints)
                stats["joint_angle_error_rad"].push_back(e.json());
            for (auto& e : trial.shape_errors)
                stats["panel_relative_position_error_m"].push_back(e.json());
            for (auto& e : trial.local_shape_errors)
                stats["panel_root_frame_position_error_m"].push_back(e.json());
            for (auto& [name, metric] : trial.navigation)
                stats["navigation"][name] = metric.json();
            mode_report["algorithms"][trial.name] = stats;
        }
        report["noise_modes"][noise_mode] = mode_report;
        if (!btruth)
            throw std::runtime_error("Bias truth write failed");
    }
    if (!frames || !jcsv || !scsv || !ncsv)
        throw std::runtime_error("Experiment output write failed");
    std::ofstream summary(output + "/summary.json");
    summary << report.dump(2);
    if (!summary)
        throw std::runtime_error("Summary write failed");
    return report;
}
} // namespace origami
