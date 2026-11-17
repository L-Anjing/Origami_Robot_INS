#include "ros2_utils.h"
#include <csignal>
#include <iostream>
#include <rclcpp/parameter_client.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

class Child
{
    pid_t pid_;

  public:
    Child(std::vector<std::string> args)
    {
        pid_ = fork();
        if (pid_ < 0)
            throw std::runtime_error("fork failed");
        if (pid_ == 0)
        {
            std::vector<char*> argv;
            for (auto& arg : args)
                argv.push_back(arg.data());
            argv.push_back(nullptr);
            execv(argv[0], argv.data());
            _exit(127);
        }
    }
    ~Child()
    {
        if (pid_ > 0)
        {
            kill(pid_, SIGTERM);
            waitpid(pid_, nullptr, 0);
        }
    }
};
int main(int argc, char** argv)
{
    if (argc != 4)
        return 2;
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<rclcpp::Node>("origami_interface_test");
        int odometry = 0, paths = 0, clocks = 0, replayed = 0;
        double z = 1;
        auto sub = node->create_subscription<origami::OdometryMsg>(
            "/Odometry", 100,
            [&](origami::OdometryMsg::ConstSharedPtr m)
            {
                ++odometry;
                z = m->pose.pose.position.z;
                if (m->child_frame_id != "imu_link")
                    throw std::runtime_error("Body frame mismatch");
            });
        auto path = node->create_subscription<origami::PathMsg>(
            "/ins/estimate_history", rclcpp::QoS(1).transient_local(),
            [&](origami::PathMsg::ConstSharedPtr) { ++paths; });
        auto clock = node->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(1).transient_local(),
            [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) { ++clocks; });
        auto replay_sub = node->create_subscription<origami::ImuMsg>(
            "/test/replay", 100, [&](origami::ImuMsg::ConstSharedPtr) { ++replayed; });
        auto pub = node->create_publisher<origami::ImuMsg>("/imu/data", 100);
        auto spin_until = [&](auto predicate, double seconds)
        {
            auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
            while (rclcpp::ok() && !predicate() && std::chrono::steady_clock::now() < end)
            {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (!predicate())
                throw std::runtime_error("ROS interface timeout");
        };
        Child estimator({std::string(argv[1]) + "/inertial_odometry", "--ros-args", "-p",
                         std::string("config_file:=") + argv[2], "-p",
                         "initial_static_seconds:=0.2"});
        spin_until([&] { return pub->get_subscription_count() > 0; }, 10);
        for (int k = 0; k < 150; ++k)
        {
            origami::ImuMsg m;
            m.header.frame_id = "imu_link";
            m.header.stamp = origami::stamp_message(1000000000LL + k * 10000000LL);
            m.linear_acceleration.z = 9.81;
            pub->publish(m);
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        spin_until([&] { return odometry > 20 && paths > 0; }, 5);
        if (std::abs(z) > 1e-8)
            throw std::runtime_error("Stationary drift");
        auto parameters =
            std::make_shared<rclcpp::AsyncParametersClient>(node, "/inertial_odometry");
        spin_until([&] { return parameters->service_is_ready(); }, 5);
        auto change =
            parameters->set_parameters({rclcpp::Parameter("stationary_window_seconds", .06)});
        spin_until(
            [&] { return change.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
            5);
        if (!change.get().at(0).successful)
            throw std::runtime_error("Timing update rejected");
        auto reset = node->create_client<std_srvs::srv::Trigger>("/inertial_odometry/reset");
        spin_until([&] { return reset->service_is_ready(); }, 5);
        auto result =
            reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        spin_until(
            [&] { return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
            5);
        if (!result.get()->success)
            throw std::runtime_error("Reset failed");
        Child replay({std::string(argv[1]) + "/imu_replay", "--ros-args", "-p",
                      std::string("data_file:=") + argv[3], "-p", "start_delay_s:=0.2", "-p",
                      "imu_topic:=/test/replay"});
        spin_until([&] { return clocks > 5 && replayed > 5; }, 10);
        auto pause = node->create_client<std_srvs::srv::SetBool>("/imu_replay/pause");
        spin_until([&] { return pause->service_is_ready(); }, 5);
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        auto paused = pause->async_send_request(request);
        spin_until(
            [&] { return paused.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
            5);
        if (!paused.get()->success)
            throw std::runtime_error("Pause failed");
        std::cout << "ROS interfaces passed: odometry=" << odometry << ", paths=" << paths
                  << ", replay=" << replayed << ", clocks=" << clocks << "\n";
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
