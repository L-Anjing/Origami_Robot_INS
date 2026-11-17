#include "preprocess.h"
#include "ros2_utils.h"
#include <chrono>
#include <iostream>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ImuReplay : public rclcpp::Node
{
    using Steady = std::chrono::steady_clock;
    std::vector<origami::Sample> samples_;
    size_t index_ = 0;
    double rate_;
    bool paused_ = false;
    std::string frame_;
    Steady::time_point due_, pause_wall_;
    rclcpp::Publisher<origami::ImuMsg>::SharedPtr imu_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    void process()
    {
        if (paused_ || index_ == samples_.size())
            return;
        auto now = Steady::now();
        // Bound the catch-up batch so pause/reset requests remain responsive.
        for (int batch = 0; batch < 100 && index_ < samples_.size(); ++batch)
        {
            const auto& s = samples_[index_];
            auto target = due_ + std::chrono::nanoseconds(static_cast<int64_t>(
                                     (s.stamp - samples_.front().stamp) / rate_));
            if (now < target)
                break;
            origami::ImuMsg m;
            m.header.frame_id = frame_;
            m.header.stamp = origami::stamp_message(s.stamp);
            m.orientation_covariance[0] = -1.;
            m.linear_acceleration.x = s.force.x();
            m.linear_acceleration.y = s.force.y();
            m.linear_acceleration.z = s.force.z();
            m.angular_velocity.x = s.gyro.x();
            m.angular_velocity.y = s.gyro.y();
            m.angular_velocity.z = s.gyro.z();
            if (clock_pub_)
            {
                rosgraph_msgs::msg::Clock clock;
                clock.clock = m.header.stamp;
                clock_pub_->publish(clock);
            }
            imu_pub_->publish(m);
            ++index_;
        }
        if (index_ == samples_.size())
        {
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "Replay complete: %zu samples", index_);
        }
    }

  public:
    ImuReplay() : Node("imu_replay")
    {
        samples_ = origami::read_recording(declare_parameter<std::string>("data_file", ""));
        frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
        rate_ = declare_parameter("playback_rate", 1.);
        double delay = declare_parameter("start_delay_s", 2.);
        if (frame_.empty() || frame_.front() == '/' || !std::isfinite(rate_) || rate_ <= 0 ||
            !std::isfinite(delay) || delay < 0)
            throw std::invalid_argument("Invalid replay parameters");
        imu_pub_ = create_publisher<origami::ImuMsg>(
            declare_parameter<std::string>("imu_topic", "/imu/data"), 2000);
        if (declare_parameter("publish_clock", true))
            clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
                "/clock", rclcpp::QoS(1).transient_local());
        due_ = Steady::now() +
               std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(delay));
        pause_service_ = create_service<std_srvs::srv::SetBool>(
            "~/pause",
            [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            {
                auto now = Steady::now();
                if (request->data && !paused_)
                {
                    pause_wall_ = now;
                    paused_ = true;
                }
                else if (!request->data && paused_)
                {
                    due_ += now - pause_wall_;
                    paused_ = false;
                }
                response->success = true;
                response->message = paused_ ? "paused" : "playing";
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { process(); });
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<ImuReplay>());
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
