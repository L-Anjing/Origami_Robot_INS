#include "calibration_diagnostics.h"
#include "multi_imu_eskf.h"
#include <fstream>
#include <iomanip>
namespace origami
{
namespace
{
struct Accumulator
{
    size_t n = 0;
    double e2 = 0, v2 = 0, q2 = 0, final = 0;
    void add(const V3D& error, const V3D& velocity = V3D::Zero(), double angle = 0)
    {
        ++n;
        e2 += error.squaredNorm();
        v2 += velocity.squaredNorm();
        q2 += angle * angle;
        final = error.norm();
    }
    Json json() const
    {
        return {{"ate_m", std::sqrt(e2 / n)},
                {"final_error_m", final},
                {"velocity_rmse_m_s", std::sqrt(v2 / n)},
                {"attitude_rmse_rad", std::sqrt(q2 / n)}};
    }
};
Json diagnose(const Json& config, const Model& model, const std::vector<SimulationFrame>& frames,
              const std::string& dataset, std::ofstream& trajectories, std::ofstream& candidates,
              std::ofstream& consistency)
{
    Json active = config;
    for (auto key : {"gyro_white_std_deg_s", "accel_white_std_m_s2",
                     "gyro_initial_bias_bound_deg_s", "accel_initial_bias_bound_m_s2",
                     "gyro_bias_rw_deg_s_sqrt_s", "accel_bias_rw_m_s2_sqrt_s"})
        active["noise"][key] = 0.;
    auto qv = config.at("initial_root_quaternion_wxyz");
    Quaterniond root(qv[0], qv[1], qv[2], qv[3]);
    std::array<double, 5> initial_angles{};
    for (int j = 0; j < 5; ++j)
        initial_angles[j] = config.at("initial_joint_angles_rad")[j];
    auto initial = forward_shape(model, root, initial_angles);
    std::vector<ErrorStateKalmanFilter> nodes, true_attitude, equivalent;
    std::array<V3D, 6> previous_position, previous_velocity, integrated_kinematic;
    std::array<Quaterniond, 6> previous_rotation;
    std::array<Accumulator, 6> metrics, true_attitude_metrics, equivalent_metrics,
        candidate_metrics, oracle_candidate_metrics;
    std::array<double, 6> acceleration2{}, gyro2{}, instantaneous2{}, attitude_contribution2{},
        velocity_contribution2{}, quadrature_contribution2{};
    for (int i = 0; i < 6; ++i)
    {
        nodes.emplace_back(config, false, false, false, true);
        true_attitude.emplace_back(config, false, false, true, true);
        equivalent.emplace_back(config, false, false, false, true);
        V3D position = vector3(config.at("robot").at("initial_root_W_m")) + initial.position[i];
        for (auto* x : {&nodes.back(), &true_attitude.back(), &equivalent.back()})
        {
            x->p = position;
            x->attitude.q = initial.rotation[i];
        }
        previous_position[i] = position;
        previous_velocity[i].setZero();
        integrated_kinematic[i] = position;
        previous_rotation[i] = initial.rotation[i];
    }
    Json cj = config;
    cj["joint_filter"]["enable_joint_position_constraint"] = true;
    cj["joint_filter"]["enable_joint_axis_constraint"] = false;
    cj["joint_filter"]["enable_shape_state"] = false;
    MultiImuEskf position_filter(cj, model);
    cj["joint_filter"]["enable_joint_axis_constraint"] = true;
    MultiImuEskf axis_filter(cj, model);
    JointEstimator joint(model, active, "A2", root);
    Accumulator mean_metric, oracle_metric, position_metric, axis_metric, mean_true_attitude,
        mean_attitude_contribution, mean_velocity_contribution, mean_quadrature_contribution,
        mean_geometry_contribution;
    double max_mean_error = 0, max_budget_error = 0;
    const double dt = 1 / config.at("sample_rate_hz").get<double>();
    for (size_t k = 0; k < frames.size(); ++k)
    {
        const auto& frame = frames[k];
        auto estimate = joint.update(frame.imu, dt);
        for (int i = 0; i < 6; ++i)
        {
            nodes[i].propagate(frame.imu.gyro[i], frame.imu.force[i], dt);
            true_attitude[i].propagate(frame.imu.gyro[i], frame.imu.force[i], dt, true, 1,
                                       frame.truth.orientation[i]);
        }
        std::array<V3D, 6> instantaneous;
        instantaneous[0] = true_attitude[0].v;
        for (const auto& h : model.joints)
        {
            V3D parent_arm = frame.truth.orientation[h.parent] *
                             (h.translation + h.rest_rotation * h.anchor_child);
            V3D child_arm = frame.truth.orientation[h.child] * h.anchor_child;
            V3D wp = frame.truth.orientation[h.parent] * frame.imu.gyro[h.parent];
            V3D wc = frame.truth.orientation[h.child] * frame.imu.gyro[h.child];
            instantaneous[h.child] =
                instantaneous[h.parent] + wp.cross(parent_arm) - wc.cross(child_arm);
        }
        auto shape = forward_shape(model, nodes[0].attitude.q, estimate.angle);
        V3D average_error = V3D::Zero(), oracle_average = V3D::Zero(),
            true_attitude_mean = V3D::Zero(), attitude_mean = V3D::Zero(),
            velocity_mean = V3D::Zero(), quadrature_mean = V3D::Zero();
        std::array<V3D, 6> position;
        std::array<Matrix15d, 6> covariance;
        for (int i = 0; i < 6; ++i)
        {
            const V3D& truth = frame.truth.markers[i];
            V3D veq = (truth - previous_position[i]) / dt;
            V3D aeq = (veq - previous_velocity[i]) / dt;
            V3D world_a = frame.truth.orientation[i] * frame.imu.force[i] + V3D(0, 0, -9.81);
            V3D interval_gyro =
                rotation_log(previous_rotation[i].conjugate() * frame.truth.orientation[i]) / dt;
            V3D interval_force = frame.truth.orientation[i].conjugate() * (aeq + V3D(0, 0, 9.81));
            // Truth-derived interval reconstruction is a diagnostic control, never a normal input.
            equivalent[i].propagate(interval_gyro, interval_force, dt);
            integrated_kinematic[i] += instantaneous[i] * dt;
            V3D e = nodes[i].p - truth, et = true_attitude[i].p - truth;
            V3D attitude_term = nodes[i].p - true_attitude[i].p;
            V3D velocity_term = true_attitude[i].p - integrated_kinematic[i];
            V3D quadrature_term = integrated_kinematic[i] - truth;
            max_budget_error = std::max(
                max_budget_error, (e - attitude_term - velocity_term - quadrature_term).norm());
            metrics[i].add(e, nodes[i].v - veq,
                           nodes[i].attitude.q.angularDistance(frame.truth.orientation[i]));
            true_attitude_metrics[i].add(et, true_attitude[i].v - veq);
            equivalent_metrics[i].add(
                equivalent[i].p - truth, equivalent[i].v - veq,
                equivalent[i].attitude.q.angularDistance(frame.truth.orientation[i]));
            acceleration2[i] += (world_a - aeq).squaredNorm();
            gyro2[i] += (frame.imu.gyro[i] - interval_gyro).squaredNorm();
            instantaneous2[i] += (true_attitude[i].v - instantaneous[i]).squaredNorm();
            true_attitude_mean += et / 6;
            attitude_mean += attitude_term / 6;
            velocity_mean += velocity_term / 6;
            quadrature_mean += quadrature_term / 6;
            attitude_contribution2[i] += attitude_term.squaredNorm();
            velocity_contribution2[i] += velocity_term.squaredNorm();
            quadrature_contribution2[i] += quadrature_term.squaredNorm();
            trajectories << dataset << ',' << frame.imu.time << ',' << i;
            for (const auto* v : std::array<const V3D*, 3>{&nodes[i].p, &truth, &e})
                for (double x : *v)
                    trajectories << ',' << x;
            trajectories << ',' << (nodes[i].v - veq).norm() << ','
                         << nodes[i].attitude.q.angularDistance(frame.truth.orientation[i]) << ','
                         << et.norm() << ',' << (equivalent[i].p - truth).norm() << '\n';
            consistency << dataset << ',' << frame.imu.time << ',' << i;
            V3D residual = world_a - aeq;
            for (double x : residual)
                consistency << ',' << x;
            consistency << ',' << (nodes[i].v - veq).norm() << ','
                        << (frame.imu.gyro[i] - interval_gyro).norm() << ','
                        << (true_attitude[i].v - instantaneous[i]).norm() << ','
                        << attitude_term.norm() << ',' << velocity_term.norm() << ','
                        << quadrature_term.norm() << '\n';
            V3D candidate = nodes[i].p - shape.position[i];
            V3D ec = candidate - frame.truth.markers[0];
            V3D eo = nodes[i].p - (truth - frame.truth.markers[0]) - frame.truth.markers[0];
            candidate_metrics[i].add(ec);
            oracle_candidate_metrics[i].add(eo);
            average_error += ec / 6;
            oracle_average += eo / 6;
            candidates << dataset << ',' << frame.imu.time << ',' << i;
            for (const auto* v : {&candidate, &ec, &eo})
                for (double x : *v)
                    candidates << ',' << x;
            candidates << '\n';
            position[i] = nodes[i].p;
            covariance[i] = nodes[i].P;
            previous_position[i] = truth;
            previous_velocity[i] = veq;
            previous_rotation[i] = frame.truth.orientation[i];
        }
        auto fusion =
            fuse_positions(position, covariance, model, shape, estimate, config.at("fusion"));
        V3D error = fusion.uniform - frame.truth.markers[0];
        max_mean_error = std::max(max_mean_error, (error - average_error).norm());
        mean_metric.add(error);
        oracle_metric.add(oracle_average);
        mean_true_attitude.add(true_attitude_mean);
        mean_attitude_contribution.add(attitude_mean);
        mean_velocity_contribution.add(velocity_mean);
        mean_quadrature_contribution.add(quadrature_mean);
        mean_geometry_contribution.add(error - oracle_average);
        position_filter.propagate(frame.imu, dt);
        axis_filter.propagate(frame.imu, dt);
        if ((k + 1) % 10 == 0)
        {
            position_filter.update(frame.imu.time);
            axis_filter.update(frame.imu.time);
        }
        V3D pe = position_filter.nodes[0].p - frame.truth.markers[0],
            ae = axis_filter.nodes[0].p - frame.truth.markers[0];
        position_metric.add(pe);
        axis_metric.add(ae);
        candidates << dataset << ',' << frame.imu.time << ",mean";
        for (const auto* v : {&fusion.uniform, &average_error, &oracle_average})
            for (double x : *v)
                candidates << ',' << x;
        candidates << '\n';
        for (auto item : {std::pair<std::string, V3D>{"均值融合", error},
                          {"铰链位置约束", pe},
                          {"铰链位置与轴约束", ae}})
        {
            trajectories << dataset << ',' << frame.imu.time << ',' << item.first;
            V3D p = frame.truth.markers[0] + item.second;
            for (const auto* v :
                 std::array<const V3D*, 3>{&p, &frame.truth.markers[0], &item.second})
                for (double x : *v)
                    trajectories << ',' << x;
            trajectories << ",0,0,0,0\n";
        }
    }
    Json result = {{"nodes", Json::array()},
                   {"candidate_mean_max_vector_difference_m", max_mean_error},
                   {"error_budget_max_closure_m", max_budget_error},
                   {"mean_fusion", mean_metric.json()},
                   {"mean_true_attitude_oracle_geometry_debug", mean_true_attitude.json()},
                   {"mean_attitude_position_contribution", mean_attitude_contribution.json()},
                   {"mean_velocity_model_contribution", mean_velocity_contribution.json()},
                   {"mean_quadrature_contribution", mean_quadrature_contribution.json()},
                   {"mean_geometry_contribution", mean_geometry_contribution.json()},
                   {"oracle_geometry_debug_only", oracle_metric.json()},
                   {"position_constraint", position_metric.json()},
                   {"position_axis_constraint", axis_metric.json()}};
    for (int i = 0; i < 6; ++i)
    {
        double n = frames.size();
        result["nodes"].push_back(
            {{"node", i},
             {"independent", metrics[i].json()},
             {"true_attitude_debug", true_attitude_metrics[i].json()},
             {"reconstructed_interval_debug", equivalent_metrics[i].json()},
             {"candidate", candidate_metrics[i].json()},
             {"oracle_candidate_debug", oracle_candidate_metrics[i].json()},
             {"acceleration_step_end_rmse_m_s2", std::sqrt(acceleration2[i] / n)},
             {"gyro_interval_rmse_rad_s", std::sqrt(gyro2[i] / n)},
             {"true_attitude_velocity_vs_kinematics_rmse_m_s", std::sqrt(instantaneous2[i] / n)},
             {"attitude_position_contribution_rmse_m", std::sqrt(attitude_contribution2[i] / n)},
             {"velocity_model_position_contribution_rmse_m",
              std::sqrt(velocity_contribution2[i] / n)},
             {"step_end_position_quadrature_rmse_m", std::sqrt(quadrature_contribution2[i] / n)},
             {"initial_velocity_error_m_s", 0.}});
    }
    if (max_mean_error > 1e-10 || max_budget_error > 1e-10)
        throw std::runtime_error("Zero-noise candidate identity failed");
    return result;
}
} // namespace
Json zero_noise_diagnostics(const Json& c, const Model& model,
                            const std::vector<SimulationFrame>& original,
                            const std::vector<SimulationFrame>& consistent,
                            const std::string& folder)
{
    std::ofstream trajectory(folder + "/zero_noise_diagnostics.csv"),
        candidate(folder + "/imu_candidate_errors.csv"),
        consistency(folder + "/propagation_consistency.csv");
    for (auto* out : {&trajectory, &candidate, &consistency})
        *out << std::setprecision(15);
    trajectory << "dataset,time_s,node,x_m,y_m,z_m,truth_x_m,truth_y_m,truth_z_m,error_x_m,error_y_"
                  "m,error_z_m,velocity_error_m_s,attitude_error_rad,true_attitude_position_error_"
                  "m,reconstructed_interval_position_error_m\n";
    candidate << "dataset,time_s,node,candidate_x_m,candidate_y_m,candidate_z_m,error_x_m,error_y_"
                 "m,error_z_m,oracle_error_x_m,oracle_error_y_m,oracle_error_z_m\n";
    consistency << "dataset,time_s,node,acceleration_residual_x,acceleration_residual_y,"
                   "acceleration_residual_z,velocity_eq_error_m_s,gyro_interval_error_rad_s,"
                   "instantaneous_velocity_error_m_s,attitude_position_contribution_m,velocity_"
                   "model_position_contribution_m,step_end_position_quadrature_m\n";
    return {
        {"original", diagnose(c, model, original, "original", trajectory, candidate, consistency)},
        {"consistent",
         diagnose(c, model, consistent, "consistent", trajectory, candidate, consistency)}};
}
} // namespace origami
