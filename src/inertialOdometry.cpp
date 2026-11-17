#include "IMU_Processing.hpp"
#include "ros2_utils.h"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <iostream>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

class InertialOdometry : public rclcpp::Node
{
    using Steady = std::chrono::steady_clock;
    using Json = origami::Json;
    Json config_, latest_;
    std::unique_ptr<origami::ImuProcess> imu_process_;
    double initial_, timeout_, path_interval_;
    bool zupt_, failed_ = false;
    std::string imu_frame_, odom_frame_, raw_frame_, failure_;
    size_t max_points_, received_ = 0, dropped_ = 0;
    std::optional<int64_t> previous_, last_path_;
    std::optional<Steady::time_point> last_input_;
    double last_dt_ = .01;
    std::deque<origami::PoseStampedMsg> history_, raw_history_;
    std::deque<int64_t> breaks_;
    std::deque<geometry_msgs::msg::Point> zupt_points_, anomaly_points_;
    rclcpp::Subscription<origami::ImuMsg>::SharedPtr imu_sub_;
    rclcpp::Publisher<origami::OdometryMsg>::SharedPtr odom_pub_, raw_pub_;
    rclcpp::Publisher<origami::PathMsg>::SharedPtr path_pub_, full_path_pub_, raw_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_, segments_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr visual_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    template <class T> T parameter(const std::string& name, T value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<T>(name, value, descriptor);
    }
    void reset()
    {
        imu_process_ = std::make_unique<origami::ImuProcess>(config_, initial_, zupt_);
        failed_ = false;
        failure_.clear();
        latest_ = nullptr;
        previous_.reset();
        last_path_.reset();
        last_input_.reset();
        history_.clear();
        raw_history_.clear();
        breaks_.clear();
        zupt_points_.clear();
        anomaly_points_.clear();
        received_ = dropped_ = 0;
    }
    void publish_odometry(const origami::ErrorStateKalmanFilter& f, int64_t ns, bool raw,
                          const origami::V3D& gyro)
    {
        origami::OdometryMsg m;
        m.header.stamp = origami::stamp_message(ns);
        m.header.frame_id = odom_frame_;
        m.child_frame_id = raw ? raw_frame_ : imu_frame_;
        m.pose.pose = origami::pose_message(f.snapshot(), ns, odom_frame_).pose;
        origami::M3D R = f.attitude.q.toRotationMatrix();
        origami::V3D velocity = R.transpose() * f.v, rate = gyro - f.bg;
        m.twist.twist.linear.x = velocity.x();
        m.twist.twist.linear.y = velocity.y();
        m.twist.twist.linear.z = velocity.z();
        m.twist.twist.angular.x = rate.x();
        m.twist.twist.angular.y = rate.y();
        m.twist.twist.angular.z = rate.z();
        if (!raw)
        {
            Eigen::Matrix<double, 6, 15> jp = Eigen::Matrix<double, 6, 15>::Zero(), jt = jp;
            jp.block<3, 3>(0, 0).setIdentity();
            jp.block<3, 3>(3, 6) = R;
            jt.block<3, 3>(0, 3) = R.transpose();
            jt.block<3, 3>(0, 6) = origami::skew(velocity);
            jt.block<3, 3>(3, 9) = -origami::M3D::Identity();
            Eigen::Matrix<double, 6, 6> pc = jp * f.P * jp.transpose(),
                                        tc = jt * f.P * jt.transpose();
            double density = config_["robot"]["eskf"]["noise_density"]["gyro_rad_s_sqrt_hz"];
            tc.bottomRightCorner<3, 3>() +=
                origami::M3D::Identity() * density * density / std::max(last_dt_, 1e-9);
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j)
                {
                    m.pose.covariance[6 * i + j] = pc(i, j);
                    m.twist.covariance[6 * i + j] = tc(i, j);
                }
        }
        else
            for (int i = 0; i < 6; ++i)
            {
                m.pose.covariance[7 * i] = 1e6;
                m.twist.covariance[7 * i] = 1e6;
            }
        (raw ? raw_pub_ : odom_pub_)->publish(m);
        geometry_msgs::msg::TransformStamped transform;
        transform.header = m.header;
        transform.child_frame_id = m.child_frame_id;
        transform.transform.translation.x = f.p.x();
        transform.transform.translation.y = f.p.y();
        transform.transform.translation.z = f.p.z();
        transform.transform.rotation = m.pose.pose.orientation;
        tf_->sendTransform(transform);
    }
    void receive(const origami::ImuMsg& m)
    {
        if (failed_)
            return;
        origami::Sample sample;
        sample.stamp = int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
        sample.force = {m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z};
        sample.gyro = {m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z};
        if (m.header.frame_id != imu_frame_ || sample.stamp < 0 ||
            (previous_ && sample.stamp <= *previous_) || !sample.force.allFinite() ||
            !sample.gyro.allFinite())
        {
            ++dropped_;
            return;
        }
        if (previous_)
            last_dt_ = (sample.stamp - *previous_) * 1e-9;
        previous_ = sample.stamp;
        last_input_ = Steady::now();
        ++received_;
        try
        {
            auto states = imu_process_->push(sample);
            if (states.empty())
                return;
            for (auto& state : states)
            {
                int64_t ns = state.at("timestamp_ns");
                bool broken = state["health"]["position_discontinuity"];
                if (broken)
                {
                    breaks_.push_back(ns);
                    anomaly_points_.push_back(
                        origami::pose_message(state.at("eskf"), ns, odom_frame_).pose.position);
                }
                if (!last_path_ || (ns - *last_path_) * 1e-9 >= path_interval_ || broken)
                {
                    history_.push_back(origami::pose_message(state.at("eskf"), ns, odom_frame_));
                    raw_history_.push_back(origami::pose_message(state.at("raw"), ns, odom_frame_));
                    last_path_ = ns;
                }
                if (state.at("accepted").get<bool>())
                    zupt_points_.push_back(
                        origami::pose_message(state.at("eskf"), ns, odom_frame_).pose.position);
            }
            while (history_.size() > max_points_)
                history_.pop_front();
            while (raw_history_.size() > max_points_)
                raw_history_.pop_front();
            while (zupt_points_.size() > max_points_)
                zupt_points_.pop_front();
            while (anomaly_points_.size() > max_points_)
                anomaly_points_.pop_front();
            if (!history_.empty())
            {
                auto oldest = int64_t(history_.front().header.stamp.sec) * 1000000000LL +
                              history_.front().header.stamp.nanosec;
                while (!breaks_.empty() && breaks_.front() < oldest)
                    breaks_.pop_front();
            }
            latest_ = std::move(states.back());
            publish_odometry(*imu_process_->eskf, sample.stamp, false, sample.gyro);
            publish_odometry(*imu_process_->raw, sample.stamp, true, sample.gyro);
        }
        catch (const std::exception& error)
        {
            failed_ = true;
            failure_ = error.what();
            RCLCPP_ERROR(get_logger(), "%s; call ~/reset", error.what());
        }
    }
    void publish_visuals()
    {
        auto stamp = origami::stamp_message(
            latest_.is_null() ? 0 : latest_.at("timestamp_ns").get<int64_t>());
        origami::PathMsg full;
        full.header.frame_id = odom_frame_;
        full.header.stamp = stamp;
        full.poses.assign(history_.begin(), history_.end());
        if (!latest_.is_null() && (full.poses.empty() || full.poses.back().header.stamp != stamp))
            full.poses.push_back(
                origami::pose_message(latest_.at("eskf"), latest_.at("timestamp_ns"), odom_frame_));
        full_path_pub_->publish(full);
        origami::PathMsg path = full;
        visualization_msgs::msg::MarkerArray segments;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        segments.markers.push_back(clear);
        size_t start = 0;
        int id = 0;
        for (auto boundary : breaks_)
        {
            size_t finish = start;
            while (finish < full.poses.size())
            {
                auto& t = full.poses[finish].header.stamp;
                if (int64_t(t.sec) * 1000000000LL + t.nanosec >= boundary)
                    break;
                ++finish;
            }
            if (finish - start >= 2)
            {
                visualization_msgs::msg::Marker line;
                line.header = full.header;
                line.ns = "trajectory_segments";
                line.id = id++;
                line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line.pose.orientation.w = 1;
                line.scale.x = .006;
                line.color.g = .745;
                line.color.b = 1;
                line.color.a = 1;
                for (size_t k = start; k < finish; ++k)
                    line.points.push_back(full.poses[k].pose.position);
                segments.markers.push_back(line);
            }
            start = finish;
        }
        path.poses.assign(full.poses.begin() + start, full.poses.end());
        path_pub_->publish(path);
        segments_pub_->publish(segments);
        origami::PathMsg raw;
        raw.header = full.header;
        raw.poses.assign(raw_history_.begin(), raw_history_.end());
        if (!latest_.is_null() && (raw.poses.empty() || raw.poses.back().header.stamp != stamp))
            raw.poses.push_back(
                origami::pose_message(latest_.at("raw"), latest_.at("timestamp_ns"), odom_frame_));
        raw_path_pub_->publish(raw);
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(clear);
        if (!latest_.is_null())
            for (int id : {0, 1, 2, 4})
            {
                visualization_msgs::msg::Marker m;
                m.header = full.header;
                m.ns = "ins";
                m.id = id;
                m.pose.orientation.w = 1;
                m.color.a = 1;
                if (id == 0)
                {
                    m.type = m.ARROW;
                    m.pose = origami::pose_message(latest_.at("eskf"), latest_.at("timestamp_ns"),
                                                   odom_frame_)
                                 .pose;
                    m.scale.x = .12;
                    m.scale.y = .025;
                    m.scale.z = .025;
                    m.color.g = .65;
                    m.color.b = 1;
                    if (latest_["health"]["estimate_unreliable"].get<bool>())
                    {
                        m.color.r = 1;
                        m.color.g = m.color.b = .1;
                    }
                }
                else if (id == 1)
                {
                    m.type = m.SPHERE;
                    m.scale.x = m.scale.y = m.scale.z = .025;
                    m.color.g = 1;
                }
                else
                {
                    m.type = m.SPHERE_LIST;
                    m.scale.x = m.scale.y = m.scale.z = id == 2 ? .012 : .03;
                    m.color.r = id == 2 ? .1 : 1;
                    m.color.g = id == 2 ? 1 : .1;
                    auto& points = id == 2 ? zupt_points_ : anomaly_points_;
                    m.points.assign(points.begin(), points.end());
                }
                markers.markers.push_back(m);
            }
        marker_pub_->publish(markers);
        diagnostic_msgs::msg::DiagnosticArray d;
        d.header = full.header;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "origami_ins/estimator";
        status.hardware_id = imu_frame_;
        status.message = "Running; yaw unreferenced";
        if (failed_)
        {
            status.level = 2;
            status.message = failure_;
        }
        else if (last_input_ &&
                 std::chrono::duration<double>(Steady::now() - *last_input_).count() > timeout_)
        {
            status.level = 3;
            status.message = "IMU input stopped";
        }
        else if (latest_.is_null())
        {
            status.level = 1;
            status.message = "Static calibration; keep IMU still";
        }
        else if (latest_["health"]["estimate_unreliable"].get<bool>())
        {
            status.level = 2;
            status.message = "Estimate unreliable";
        }
        else if (latest_["health"]["position_uncertain"].get<bool>() ||
                 !latest_.at("valid").get<bool>())
        {
            status.level = 1;
            status.message = "Position uncertainty or invalid aiding";
        }
        auto value = [&](const std::string& key, const std::string& text)
        {
            diagnostic_msgs::msg::KeyValue v;
            v.key = key;
            v.value = text;
            status.values.push_back(v);
        };
        value("received", std::to_string(received_));
        value("dropped", std::to_string(dropped_));
        if (!latest_.is_null())
        {
            value("gyro_bias_rad_s", latest_["eskf"]["bg"].dump());
            value("accel_bias_m_s2", latest_["eskf"]["ba"].dump());
            value("zupt_nis", latest_["nis"].dump());
            value("health", latest_["health"].dump());
        }
        d.status.push_back(status);
        diagnostic_pub_->publish(d);
    }

  public:
    InertialOdometry() : Node("inertial_odometry")
    {
        std::ifstream file(parameter<std::string>("config_file", ""));
        if (!file)
            throw std::invalid_argument("config_file required");
        file >> config_;
        initial_ = parameter("initial_static_seconds", 2.);
        auto mode = parameter<std::string>("zupt_mode", "glrt");
        if (mode != "glrt" && mode != "none")
            throw std::invalid_argument("Invalid ZUPT mode");
        zupt_ = mode == "glrt";
        imu_frame_ = parameter<std::string>("imu_frame", "imu_link");
        odom_frame_ = parameter<std::string>("odom_frame", "odom");
        raw_frame_ = parameter<std::string>("raw_body_frame", "imu_raw");
        auto body = parameter<std::string>("body_frame", "imu_link");
        for (auto& frame : {imu_frame_, odom_frame_, raw_frame_})
            if (frame.empty() || frame.front() == '/')
                throw std::invalid_argument("Invalid frame ID");
        if (body != imu_frame_ || imu_frame_ == odom_frame_ || raw_frame_ == imu_frame_ ||
            raw_frame_ == odom_frame_)
            throw std::invalid_argument("Inconsistent body/odom/raw frames");
        auto max_points = parameter("max_path_points", 20000);
        if (max_points < 2)
            throw std::invalid_argument("max_path_points must be >=2");
        max_points_ = max_points;
        auto& rest = config_["robot"]["stationary"];
        rest["window_s"] = declare_parameter("stationary_window_seconds", .05);
        rest["minimum_stationary_s"] = declare_parameter("minimum_stationary_seconds", .05);
        rest["gyro_limit_rad_s"] = parameter("gyro_threshold_rad_s", .08);
        rest["force_tolerance_m_s2"] = parameter("force_tolerance_m_s2", .30);
        auto& real = config_["real_imu"];
        real["max_integration_interval_s"] = parameter("max_integration_interval_seconds", .25);
        real["protect_position_updates"] = parameter("protect_zupt_position", true);
        for (auto pair : {std::pair{"max_speed_m_s", 2.},
                          {"step_tolerance_m", .02},
                          {"max_position_correction_m", .10},
                          {"max_position_std_m", .50}})
        {
            double v = parameter(std::string("health.") + pair.first, pair.second);
            if (!std::isfinite(v) || v <= 0)
                throw std::invalid_argument("Invalid health limit");
            real["health"][pair.first] = v;
        }
        timeout_ = parameter("input_timeout_seconds", .50);
        double path_rate = parameter("path_rate_hz", 20.),
               visual_rate = parameter("visualization_rate_hz", 10.);
        if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(path_rate) ||
            path_rate <= 0 || !std::isfinite(visual_rate) || visual_rate <= 0)
            throw std::invalid_argument("Invalid publish rates or timeout");
        path_interval_ = 1 / path_rate;
        reset();
        odom_pub_ = create_publisher<origami::OdometryMsg>("/Odometry", 100);
        raw_pub_ = create_publisher<origami::OdometryMsg>("/ins/raw_odometry", 100);
        auto latched = rclcpp::QoS(1).transient_local();
        path_pub_ = create_publisher<origami::PathMsg>("/path", latched);
        full_path_pub_ = create_publisher<origami::PathMsg>("/ins/estimate_history", latched);
        raw_path_pub_ = create_publisher<origami::PathMsg>("/ins/raw_path", latched);
        marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("/ins/markers", latched);
        segments_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ins/trajectory_segments", latched);
        diagnostic_pub_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu_sub_ = create_subscription<origami::ImuMsg>(
            parameter<std::string>("imu_topic", "/imu/data"),
            rclcpp::SensorDataQoS().keep_last(2000),
            [this](origami::ImuMsg::ConstSharedPtr message) { receive(*message); });
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                reset();
                publish_visuals();
                response->success = true;
                response->message = "Reset; keep IMU static";
            });
        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                auto candidate = config_["robot"]["stationary"];
                try
                {
                    for (auto& p : parameters)
                    {
                        if (p.get_name() == "stationary_window_seconds")
                            candidate["window_s"] = p.as_double();
                        if (p.get_name() == "minimum_stationary_seconds")
                            candidate["minimum_stationary_s"] = p.as_double();
                    }
                    imu_process_->set_stationary_timing(candidate.at("window_s"),
                                                        candidate.at("minimum_stationary_s"));
                    config_["robot"]["stationary"] = candidate;
                }
                catch (const std::exception& error)
                {
                    result.successful = false;
                    result.reason = error.what();
                }
                return result;
            });
        visual_timer_ = create_wall_timer(std::chrono::duration<double>(1 / visual_rate),
                                          [this] { publish_visuals(); });
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<InertialOdometry>());
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
