#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRecord : public rclcpp::Node
{
    std::ofstream output_;
    size_t count_ = 0;
    int64_t last_stamp_ = -1;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

  public:
    ImuRecord() : Node("imu_record")
    {
        const auto path = declare_parameter<std::string>("output_file", "");
        const auto topic = declare_parameter<std::string>("imu_topic", "/imu/data");
        if (path.empty())
            throw std::invalid_argument("Set output_file to a new TXT file");
        // Refuse to overwrite an existing recording.
        if (std::filesystem::exists(path))
            throw std::runtime_error("Output file already exists: " + path);
        output_.open(path);
        if (!output_)
            throw std::runtime_error("Cannot create output: " + path);
        output_ << "# ROS 2 IMU recording (imu_record)\n# topic: " << topic
                << "\n# columns: step 0 qw qx qy qz ax ay az wx wy wz;T timestamp_ns\n"
                   "# units: body specific force m/s^2; body angular velocity rad/s\n"
                   "# quaternion: wxyz, copied from device; unavailable orientation uses identity\n"
                << std::setprecision(17);
        output_.flush();
        subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu& m)
            {
                const int64_t stamp =
                    int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
                const auto& a = m.linear_acceleration;
                const auto& w = m.angular_velocity;
                auto q = m.orientation;
                if (m.orientation_covariance[0] == -1)
                {
                    q.w = 1;
                    q.x = q.y = q.z = 0;
                }
                if (stamp < 0 || stamp <= last_stamp_ || !std::isfinite(a.x) ||
                    !std::isfinite(a.y) || !std::isfinite(a.z) || !std::isfinite(w.x) ||
                    !std::isfinite(w.y) || !std::isfinite(w.z) || !std::isfinite(q.w) ||
                    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
                    std::abs(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) - 1.) >= .01)
                {
                    RCLCPP_WARN(get_logger(), "Skipping invalid or non-increasing IMU sample");
                    return;
                }
                output_ << ++count_ << " 0 " << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' '
                        << a.x << ' ' << a.y << ' ' << a.z << ' ' << w.x << ' ' << w.y << ' ' << w.z
                        << ";T " << stamp << '\n';
                output_.flush();
                if (!output_)
                    throw std::runtime_error("Recording write failed");
                last_stamp_ = stamp;
            });
        RCLCPP_INFO(get_logger(), "Recording %s to %s; Ctrl+C to stop", topic.c_str(),
                    path.c_str());
    }
    ~ImuRecord() override
    {
        RCLCPP_INFO(get_logger(), "Saved %zu samples", count_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        rclcpp::spin(std::make_shared<ImuRecord>());
    }
    catch (const std::exception& e)
    {
        std::cerr << "imu_record: " << e.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
