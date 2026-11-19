#include "multi_imu_eskf.h"
#include "multi_imu_fusion.h"
#include "preprocess.h"
#include <Eigen/Sparse>
#include <chrono>
namespace origami
{
namespace
{
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point t)
{
    return std::chrono::duration<double>(Clock::now() - t).count();
}
} // namespace
Json normalize_multi_imu_config(const Json& input)
{
    Json c = input;
    auto& options = c.at("joint_filter");
    if (options.contains("imu_count"))
    {
        const auto& value = options.at("imu_count");
        if (!value.is_number_integer() || value.get<double>() < 1 || value.get<double>() > 6)
            throw std::invalid_argument("joint_filter.imu_count must be an integer in [1, 6]");
        int count = value.get<int>();
        options["participating_imus"] = Json::array();
        for (int i = 0; i < count; ++i)
            options["participating_imus"].push_back(i);
        if (!options.contains("compact_prefix_state"))
            options["compact_prefix_state"] = true;
    }
    return c;
}
MultiImuEskf::MultiImuEskf(const Json& input, const Model& model)
    : config_(normalize_multi_imu_config(input)), model_(model),
      shape_state_(config_.at("joint_filter").value("enable_shape_state", false))
{
    const auto& c = config_;
    const auto& options = c.at("joint_filter");
    if (options.value("enable_joint_velocity_constraint", false) ||
        options.value("enable_contact_update", false))
        throw std::invalid_argument(
            "Velocity/contact update requires a separately validated correlated measurement model");
    for (int i : options.at("participating_imus").get<std::vector<int>>())
    {
        if (i < 0 || i > 5 || selected_[i])
            throw std::invalid_argument("Invalid participating IMU list");
        selected_[i] = true;
    }
    if (!selected_[0])
        throw std::invalid_argument("All comparisons require S0");
    if (shape_state_)
        for (bool selected : selected_)
            if (!selected)
                throw std::invalid_argument(
                    "D4 currently requires six IMUs; missing internal angles are not invented");
    // Optional compact prefix layout preserves the selected marginal of the full filter.
    if (options.value("compact_prefix_state", false))
    {
        node_count_ = options.at("participating_imus").size();
        for (int i = 0; i < 6; ++i)
            if (selected_[i] != (i < node_count_))
                throw std::invalid_argument("Compact layout requires contiguous S0..S(N-1)");
    }
    auto qv = c.at("initial_root_quaternion_wxyz");
    Quaterniond q(qv[0].get<double>(), qv[1].get<double>(), qv[2].get<double>(),
                  qv[3].get<double>());
    q.normalize();
    for (int j = 0; j < 5; ++j)
        angles[j] = c.at("initial_joint_angles_rad")[j];
    auto shape = forward_shape(model, q, angles);
    std::array<double, 5> rates;
    for (int j = 0; j < 5; ++j)
        rates[j] = c.at("initial_joint_rates_rad_s")[j];
    auto velocities = initial_node_velocities(
        model, shape, rates, q * vector3(c.at("initial_root_angular_velocity_rad_s")),
        vector3(c.at("initial_velocity_m_s")));
    for (int i = 0; i < node_count_; ++i)
    {
        nodes.emplace_back(c, true, false, false, !c.value("root_ahrs_feedback", false));
        nodes[i].p = vector3(c.at("robot").at("initial_root_W_m")) + shape.position[i];
        nodes[i].v = velocities[i];
        nodes[i].attitude.q = shape.rotation[i];
    }
    int n = shape_state_ ? 95 : 15 * node_count_;
    P = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, 45);
    Eigen::VectorXd variance = Eigen::VectorXd::Zero(45);
    auto s = c.at("robot").at("eskf").at("initial_std");
    double stds[] = {s.at("position_m"), s.at("velocity_m_s"),
                     s.at("attitude_deg").get<double>() * pi / 180};
    for (int j = 0; j < 9; ++j)
        variance[j] = stds[j / 3] * stds[j / 3];
    for (int i = 0; i < node_count_; ++i)
    {
        B.block<3, 3>(15 * i, 0).setIdentity();
        B.block<3, 3>(15 * i, 6) = -q.toRotationMatrix() * skew(q.conjugate() * shape.position[i]);
        B.block<3, 3>(15 * i + 3, 3).setIdentity();
        B.block<3, 3>(15 * i + 6, 6) =
            shape.rotation[i].toRotationMatrix().transpose() * q.toRotationMatrix();
        B.block<6, 6>(15 * i + 9, 9 + 6 * i).setIdentity();
        for (int k = 0; k < 6; ++k)
        {
            double sigma = s.at(k < 3 ? "gyro_bias_rad_s" : "accel_bias_m_s2");
            variance[9 + 6 * i + k] = sigma * sigma;
        }
    }
    P = B * variance.asDiagonal() * B.transpose();
    double rho = c.at("noise").value("common_bias_correlation", 0.);
    if (rho < 0 || rho > 1)
        throw std::invalid_argument("Invalid common bias correlation");
    for (int i = 0; i < node_count_; ++i)
        for (int j = 0; j < node_count_; ++j)
            if (i != j)
                for (int k = 0; k < 6; ++k)
                    P(15 * i + 9 + k, 15 * j + 9 + k) = rho * variance[9 + 6 * i + k];
    phi_ = Eigen::MatrixXd::Identity(n, n);
    process_noise_ = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < node_count_; ++i)
        nodes[i].P = P.block<15, 15>(15 * i, 15 * i);
    for (int j = 0; j < 5; ++j)
        if (selected_[model.joints[j].parent] && selected_[model.joints[j].child] &&
            hinge_position(nodes, model.joints[j], j, n).value.norm() > 1e-10)
            throw std::runtime_error("Inconsistent initial hinge geometry");
}
void MultiImuEskf::propagate(const ImuMeasurements& sample, double dt)
{
    auto start = Clock::now();
    int n = P.rows();
    process_noise_.setZero();
    std::vector<Eigen::Triplet<double>> entries;
    Eigen::Matrix<double, 5, 72> angle_noise = Eigen::Matrix<double, 5, 72>::Zero();
    for (int i = 0; i < node_count_; ++i)
    {
        if (!selected_[i])
        {
            for (int k = 0; k < 15; ++k)
                entries.emplace_back(15 * i + k, 15 * i + k, 1.);
            continue;
        }
        auto& state = nodes[i];
        state.P = P.block<15, 15>(15 * i, 15 * i);
        Sample in;
        in.force = sample.force[i];
        in.gyro = sample.gyro[i];
        bool valid =
            quality(in, config_.at("robot").at("quality"), config_.at("gravity_m_s2"), in.force);
        state.propagate(
            in.gyro, in.force, dt, valid,
            valid ? 1
                  : config_.at("robot").at("quality").at("impact_noise_multiplier").get<double>());
        for (int r = 0; r < 15; ++r)
            for (int c = 0; c < 15; ++c)
                if (state.transition(r, c) != 0)
                    entries.emplace_back(15 * i + r, 15 * i + c, state.transition(r, c));
        process_noise_.block<15, 15>(15 * i, 15 * i) = state.process_noise;
    }
    if (shape_state_)
    {
        for (int j = 0; j < 5; ++j)
        {
            auto& h = model_.joints[j];
            auto& parent = nodes[h.parent];
            auto& child = nodes[h.child];
            // Bias values are unchanged by nominal propagation; use the same sensor sample once.
            angles[j] += (h.axis_child.dot(sample.gyro[h.child] - child.bg) -
                          h.axis_parent.dot(sample.gyro[h.parent] - parent.bg)) *
                         dt;
            entries.emplace_back(90 + j, 90 + j, 1.);
            for (int k = 0; k < 3; ++k)
            {
                entries.emplace_back(90 + j, 15 * h.parent + 9 + k, h.axis_parent[k] * dt);
                entries.emplace_back(90 + j, 15 * h.child + 9 + k, -h.axis_child[k] * dt);
                angle_noise(j, 12 * h.parent + k) = h.axis_parent[k];
                angle_noise(j, 12 * h.child + k) = -h.axis_child[k];
            }
        }
        // A single physical gyro noise column drives both node attitude and adjacent joint angles.
        for (int i = 0; i < node_count_; ++i)
        {
            auto A = angle_noise.block<5, 12>(0, 12 * i);
            auto diagonal = nodes[i].noise_variance.asDiagonal();
            process_noise_.block(90, 90, 5, 5) += A * diagonal * A.transpose();
            Eigen::Matrix<double, 15, 5> cross = nodes[i].noise_mapping * diagonal * A.transpose();
            process_noise_.block(15 * i, 90, 15, 5) = cross;
            process_noise_.block(90, 15 * i, 5, 15) = cross.transpose();
        }
    }
    double rho = config_.at("noise").value("common_bias_correlation", 0.);
    if (rho > 0)
        for (int i = 0; i < node_count_; ++i)
            for (int j = 0; j < node_count_; ++j)
                if (i != j && selected_[i] && selected_[j])
                {
                    Eigen::Matrix<double, 6, 1> d;
                    for (int k = 0; k < 6; ++k)
                        d[k] = rho * std::sqrt(nodes[i].noise_variance[6 + k] *
                                               nodes[j].noise_variance[6 + k]);
                    process_noise_.block<15, 15>(15 * i, 15 * j) =
                        nodes[i].noise_mapping.rightCols<6>() * d.asDiagonal() *
                        nodes[j].noise_mapping.rightCols<6>().transpose();
                }
    Eigen::SparseMatrix<double> phi(n, n);
    phi.setFromTriplets(entries.begin(), entries.end());
    Eigen::MatrixXd left = phi * P;
    P = (phi * left.transpose()).transpose().eval() + process_noise_;
    P = (.5 * (P + P.transpose())).eval();
    phi_ = Eigen::MatrixXd(phi);
    for (int i = 0; i < node_count_; ++i)
        nodes[i].P = P.block<15, 15>(15 * i, 15 * i);
    ++sample_;
    runtime.propagation_s += seconds(start);
    ++runtime.propagation_count;
}
void MultiImuEskf::inject(const Eigen::VectorXd& dx)
{
    for (int i = 0; i < node_count_; ++i)
    {
        auto& state = nodes[i];
        state.p += dx.segment<3>(15 * i);
        state.v += dx.segment<3>(15 * i + 3);
        state.attitude.q = (state.attitude.q * increment(dx.segment<3>(15 * i + 6))).normalized();
        state.bg += dx.segment<3>(15 * i + 9);
        state.ba += dx.segment<3>(15 * i + 12);
    }
    if (shape_state_)
        for (int j = 0; j < 5; ++j)
            angles[j] += dx[90 + j];
    // Reset every orientation row and column, including all node-node and node-angle cross blocks.
    for (int i = 0; i < node_count_; ++i)
    {
        M3D J = right_jacobian(dx.segment<3>(15 * i + 6));
        P.middleRows(15 * i + 6, 3) = (J * P.middleRows(15 * i + 6, 3)).eval();
        P.middleCols(15 * i + 6, 3) = (P.middleCols(15 * i + 6, 3) * J.transpose()).eval();
    }
    P = (.5 * (P + P.transpose())).eval();
    for (int i = 0; i < node_count_; ++i)
        nodes[i].P = P.block<15, 15>(15 * i, 15 * i);
}
void MultiImuEskf::update(double time)
{
    auto start = Clock::now();
    last_innovations.clear();
    last_correction = Eigen::VectorXd::Zero(P.rows());
    const auto& option = config_.at("joint_filter");
    int n = P.rows();
    std::vector<Constraint> constraints;
    if (shape_state_)
        constraints = shape_constraints(nodes, model_, angles, n);
    else
        for (int j = 0; j < 5; ++j)
        {
            auto& h = model_.joints[j];
            if (!selected_[h.parent] || !selected_[h.child])
                continue;
            if (option.at("enable_joint_position_constraint").get<bool>())
                constraints.push_back(hinge_position(nodes, h, j, n));
            if (option.at("enable_joint_axis_constraint").get<bool>())
                constraints.push_back(hinge_axis(nodes, h, j, n));
        }
    int total = 0;
    std::vector<Constraint> accepted;
    std::vector<double> sigmas;
    for (auto& c : constraints)
    {
        bool position = c.kind == "position" || c.kind == "shape_position";
        auto tic = Clock::now();
        double sigma = option.at(position ? "position_std_m" : "axis_std_rad"),
               gate = option.at(c.value.size() == 2 ? "axis_nis_limit" : "position_nis_limit");
        if (!std::isfinite(sigma) || sigma <= 0)
            throw std::invalid_argument("Constraint standard deviations must be positive");
        Eigen::SparseMatrix<double> H = c.H.sparseView();
        Eigen::MatrixXd HP = H * P;
        Eigen::MatrixXd S = HP * H.transpose();
        S.diagonal().array() += sigma * sigma;
        Eigen::LDLT<Eigen::MatrixXd> solve(S);
        InnovationRecord record;
        record.time = time;
        record.kind = c.kind;
        record.joint = c.joint;
        record.innovation = -c.value;
        record.nis = solve.info() == Eigen::Success && solve.isPositive()
                         ? c.value.dot(solve.solve(c.value))
                         : INFINITY;
        record.accepted = std::isfinite(record.nis) && record.nis <= gate;
        if (c.kind == "axis")
        {
            auto& h = model_.joints[c.joint];
            if ((nodes[h.parent].attitude.q * h.axis_parent)
                    .dot(nodes[h.child].attitude.q * h.axis_child) < 0)
                record.accepted = false;
        }
        if (record.accepted)
        {
            total += c.value.size();
            accepted.push_back(c);
            sigmas.push_back(sigma);
        }
        last_innovations.push_back(record);
        if (position)
            runtime.position_update_s += seconds(tic);
        else
            runtime.axis_update_s += seconds(tic);
    }
    if (total)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total, n);
        Eigen::VectorXd innovation(total), variance(total);
        int row = 0;
        for (size_t i = 0; i < accepted.size(); ++i)
        {
            int m = accepted[i].value.size();
            H.middleRows(row, m) = accepted[i].H;
            innovation.segment(row, m) = -accepted[i].value;
            variance.segment(row, m).setConstant(sigmas[i] * sigmas[i]);
            row += m;
        }
        Eigen::SparseMatrix<double> sparse = H.sparseView();
        Eigen::MatrixXd HP = sparse * P;
        Eigen::MatrixXd S = HP * sparse.transpose();
        S.diagonal() += variance;
        Eigen::LDLT<Eigen::MatrixXd> solve(S);
        if (solve.info() != Eigen::Success || !solve.isPositive())
            throw std::runtime_error("Invalid joint innovation covariance");
        Eigen::MatrixXd K = solve.solve(HP).transpose();
        Eigen::VectorXd dx = K * innovation;
        last_correction = dx;
        // Factored Joseph: left=(I-KH)P; posterior=left(I-KH)^T+KRK^T.
        Eigen::MatrixXd left = P - K * HP;
        P = left - (sparse * left.transpose()).transpose() * K.transpose() +
            K * variance.asDiagonal() * K.transpose();
        if (!P.allFinite() || !dx.allFinite())
            throw std::runtime_error("Nonfinite joint filter update");
        inject(dx);
        double bias = 0;
        for (int i = 0; i < node_count_; ++i)
            bias += dx.segment<6>(15 * i + 9).squaredNorm();
        for (auto& record : last_innovations)
            if (record.accepted)
            {
                record.correction_norm = dx.norm();
                record.bias_change_norm = std::sqrt(bias);
            }
    }
    runtime.total_update_s += seconds(start);
    ++runtime.update_count;
}
Json MultiImuEskf::diagnostics() const
{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
    if (eig.info() != Eigen::Success || !P.allFinite())
        throw std::runtime_error("Invalid global covariance");
    double cross = 0, angular = 0;
    for (int i = 0; i < node_count_; ++i)
        for (int j = 0; j < node_count_; ++j)
            if (i != j)
                cross += P.block<15, 15>(15 * i, 15 * j).squaredNorm();
    if (shape_state_)
        angular = P.block(0, 90, 90, 5).norm();
    double minimum = eig.eigenvalues().minCoeff();
    if (minimum < -1e-8)
        throw std::runtime_error("Global covariance lost PSD");
    return {{"dimension", P.rows()},
            {"symmetry_error", (P - P.transpose()).cwiseAbs().maxCoeff()},
            {"min_eigenvalue", minimum},
            {"cross_node_covariance_norm", std::sqrt(cross)},
            {"node_joint_cross_covariance_norm", angular},
            {"position_std_m", std::sqrt(std::max(0., P.block<3, 3>(0, 0).trace()))}};
}
} // namespace origami
