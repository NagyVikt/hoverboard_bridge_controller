#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp" // Include the header for String messages
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <regex>
#include <fstream> // Include the header for file operations
#include <chrono>
#include <iomanip> // for std::put_time
#include <algorithm>
#include <array>
#include <numeric>

// Assuming these namespaces are already being used in your code
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Constants
constexpr double HIGH_KP = 1.0; // Higher gain for faster response
constexpr double LOW_KP = 0.5;  // Lower gain for stable state
constexpr int FAST_SAMPLING_RATE_MS = 10;  // Fast sampling rate in milliseconds
constexpr int SLOW_SAMPLING_RATE_MS = 50;  // Slow sampling rate for stable state
constexpr double COMMAND_CHANGE_THRESHOLD = 0.05; // Threshold for significant command change

class HoverboardBridge : public rclcpp_lifecycle::LifecycleNode {
public:
    HoverboardBridge() : rclcpp_lifecycle::LifecycleNode("hoverboard_bridge") {
        // Open a log file in append mode
        log_file_.open("hoverboard_bridgelog.txt", std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            std::cerr << "Failed to open log file for writing." << std::endl;
        }

        // Parameter declarations are typically done in the constructor
        this->declare_parameter<double>("wheel_base_width", 0.53); // Wheel base width in meters
        this->declare_parameter<double>("wheel_radius", 0.085); // Wheel radius in meters
        this->declare_parameter<double>("kP", 0.1);  // Default value of 0.1
    }

    ~HoverboardBridge() {
        // Close the log file when the node is destroyed
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        // Retrieve parameters
        this->get_parameter("wheel_base_width", wheel_base_width_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("kP", kP);

        // Initialize publishers and subscribers within the configure step
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&HoverboardBridge::cmdVelCallback, this, _1));

        cmd_left_publisher_ = this->create_publisher<std_msgs::msg::Int32>("cmd_left", 10);
        cmd_right_publisher_ = this->create_publisher<std_msgs::msg::Int32>("cmd_right", 10);

        return CallbackReturn::SUCCESS;
    }

private:
    std::ofstream log_file_; // File stream for logging
    double wheel_base_width_;
    double wheel_radius_;
    double kP;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double new_linear_velocity = msg->linear.x;
        double new_angular_velocity = msg->angular.z;

        double desired_rpm_left, desired_rpm_right;
        calculateDesiredRPM(new_linear_velocity, new_angular_velocity, desired_rpm_left, desired_rpm_right);

        // Now convert RPM to PWM (this part depends on your motor controller)
        int pwm_left = convertRPMtoPWM(desired_rpm_left);
        int pwm_right = convertRPMtoPWM(desired_rpm_right);

        publishPWM(pwm_left, pwm_right);
    }

    void calculateDesiredRPM(double linear_velocity, double angular_velocity, double& desired_rpm_left, double& desired_rpm_right) {
        // This function calculates the desired RPM based on the received cmd_vel
        // Note: Implementation depends on your robot's kinematic model
    }

    int convertRPMtoPWM(double rpm) {
        // This function converts RPM to PWM
        // Note: Implementation depends on your specific hardware
        return static_cast<int>(rpm); // Placeholder conversion
    }

    void publishPWM(int pwm_left, int pwm_right) {
        std_msgs::msg::Int32 left_wheel_msg, right_wheel_msg;
        left_wheel_msg.data = pwm_left;
        right_wheel_msg.data = pwm_right;

        cmd_left_publisher_->publish(left_wheel_msg);
        cmd_right_publisher_->publish(right_wheel_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr cmd_left_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr cmd_right_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<HoverboardBridge>();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
