#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>

#include "std_msgs/msg/string.hpp" // Include the header for String messages
#include <iostream>
#include <sstream>
#include <string>

#include <regex>

#include <fstream> // Include the header for file operations
#include <chrono>
#include <iomanip> // for std::put_time

using std::placeholders::_1;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HoverboardBridge : public rclcpp_lifecycle::LifecycleNode {
public:
    HoverboardBridge() : rclcpp_lifecycle::LifecycleNode("hoverboard_bridge") {
        // Open a log file in append mode
        log_file_.open("hoverboard_bridgelog.txt", std::ios::out);
        if (log_file_.is_open()) {
            //logInfo("Node initialized");
        } else {
            std::cerr << "Failed to open log file for writing." << std::endl;
        }
        log_file2_.open("hoverboard_bridgelog2.txt", std::ios::out);
        if (log_file2_.is_open()) {
            //logInfo2("Node initialized");
        } else {
            std::cerr << "Failed to open log file for writing." << std::endl;
        }
        rclcpp::QoS customized_qos(10);


        customized_qos.best_effort(); // Use this for a reliable policy
        // Parameter declarations are typically done in the constructor
        this->declare_parameter<double>("wheel_base_width",
                                        0.53); // Wheel base width in meters
        this->declare_parameter<double>("wheel_radius",
                                        0.085); // Wheel radius in meters

        this->declare_parameter<double>("kP", 0.1);  // Default value of 0.1



    }

    ~HoverboardBridge() {

        // Close the log file when the node is destroyed
        if (log_file_.is_open()) {
            //logInfo("Node shutting down");
            log_file_.close();
        }
        if (log_file2_.is_open()) {
            //logInfo2("Node shutting down");
            log_file2_.close();

        }


    }

        // Explicit shutdown function
    void shutdownHoverboard() {
        // Make sure the publisher is still valid and active
        if (cmd_left_publisher_->is_activated() &&
            cmd_right_publisher_->is_activated()) {

            cmd_left_publisher_->publish(0);
            cmd_right_publisher_->publish(0);


            // Optionally, add a small delay to ensure the message is sent out
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }


    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        // Activate the publishers
        shutdownHoverboard();
        cmd_left_publisher_->on_activate();
        cmd_right_publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        // Deactivate the publishers
        shutdownHoverboard();
        cmd_left_publisher_->on_deactivate();
        cmd_right_publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        // Cleanup the resources

        shutdownHoverboard();
        cmd_vel_subscriber_.reset();
        feedback_subscriber_.reset();
        cmd_left_publisher_.reset();
        cmd_right_publisher_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        // Handle shutdown
        shutdownHoverboard();
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        // Retrieve parameters here to ensure they're available after configuration
        this->get_parameter("wheel_base_width", wheel_base_width_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("kP", kP);

        // Define QoS settings here, within the scope of on_configure
        rclcpp::QoS customized_qos(10);
        customized_qos.best_effort(); // Adjust this to match the publisher's QoS
                                    // settings, if necessary

        // Initialize publishers and subscribers within the configure step
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&HoverboardBridge::cmdVelCallback, this, _1));

        feedback_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "feedback", customized_qos,
            std::bind(&HoverboardBridge::feedbackCallback, this,
                    std::placeholders::_1));

        cmd_left_publisher_ =
            this->create_publisher<std_msgs::msg::Int32>("cmd_left", 10);
        cmd_right_publisher_ =
            this->create_publisher<std_msgs::msg::Int32>("cmd_right", 10);

        return CallbackReturn::SUCCESS;
    }

private:
    std::ofstream log_file_, log_file2_; // File stream for logging
    double wheelCircumference = 0.534055; // Circumference in meters
    double min_pwm_value = -500, max_pwm_value = 500;
    int actual_RPM_L = 0, actual_RPM_R = 0;
    double desired_pwmL = 0, desired_pwmR = 0;
    int febPwmR = 0, febPwmL = 0;
    double linear_velocity_ = 0; // Current lWinear velocity from cmd_vel
    double angular_velocity_ = 0;

    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0;

    int desired_rpm_left_ = 0;  // Desired RPM for the left wheel
    int desired_rpm_right_ = 0; // Desired RPM for the right wheel

    const double LINEAR_VELOCITY_THRESHOLD = 0.01; // Adjust as needed
    const double ANGULAR_VELOCITY_THRESHOLD = 0.01; // Adjust as needed
    bool initial_publish_ = true;
    bool feedback_received_ = false; // Flag to indicate feedback received
    double increment_left = 0;
    double increment_right = 0;

    double new_desired_rpm_left, new_desired_rpm_right;

    double error_left = desired_rpm_left_ - std::abs(actual_RPM_L);
    double error_right = desired_rpm_right_ - std::abs(actual_RPM_R);
    double new_linear_velocity;
    double new_angular_velocity;
    const int MIN_PWM = -300;  // Adjust this value based on your hardware limits
    const int MAX_PWM = 300;   // Adjust this value based on your hardware limits

    int intDesired_pwmL = 0;
    int intDesired_pwmR = 0;


    double last_effective_pwmL = 0;
    double last_effective_pwmR = 0;



    // Define a structure to hold all parsed feedback data
    struct FeedbackData {
        int pwmL;
        int pwmR;
        int speedL;
        int speedR;
        int cmdLed;
        int batVolt;
        int boardTemp;
    };

    void logInfo(const std::string& message) {
        if (!log_file_.is_open()) return;

        // Get the current time with high precision
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000;

        // Use put_time to format the time as you want
        log_file_ << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
        // Add milliseconds and nanoseconds
        log_file_ << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
        log_file_ << std::setfill('0') << std::setw(3) << now_ns.count();
        log_file_ << ": " << message << std::endl;
    }
    void logInfo2(const std::string& message) {
        if (!log_file_.is_open()) return;

        // Get the current time with high precision
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000;

        // Use put_time to format the time as you want
        log_file_ << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
        // Add milliseconds and nanoseconds
        log_file_ << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
        log_file_ << std::setfill('0') << std::setw(3) << now_ns.count();
        log_file_ << ": " << message << std::endl;
    }



    void parseFeedbackData(const std::string &data, FeedbackData &feedback) {
        // RCLCPP_INFO(this->get_logger(), "Starting to parse feedback data.");
        // //logInfo("Starting to parse feedback data.");
        std::regex reg("pwmL: (-?\\d+), pwmR: (-?\\d+), SpeedL: (-?\\d+), SpeedR: (-?\\d+)");
        std::smatch matches;

        if (std::regex_search(data, matches, reg) && matches.size() == 5) {
            feedback.pwmL = std::stoi(matches[1].str());
            feedback.pwmR = std::stoi(matches[2].str());
            feedback.speedL = std::stoi(matches[3].str());
            actual_RPM_L = std::stoi(matches[3].str());  
            feedback.speedR = std::stoi(matches[4].str());
            actual_RPM_R = std::stoi(matches[4].str());  

            // RCLCPP_INFO(this->get_logger(),
            //             "Successfully parsed feedback data: pwmL=%d, pwmR=%d, "
            //             "SpeedL=%d, SpeedR=%d",
            //             feedback.pwmL, feedback.pwmR, feedback.speedL,
            //             feedback.speedR);
            //logInfo("Parsed feedback data: pwmL=" + std::to_string(feedback.pwmL) + ", pwmR=" + std::to_string(feedback.pwmR) + 
                   // ", SpeedL=" + std::to_string(feedback.speedL) + ", SpeedR=" + std::to_string(feedback.speedR));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse feedback data.");
            //logInfo("Failed to parse feedback data.");
        }
    }

    bool significantChangeInCmdVel(double new_linear_velocity, double new_angular_velocity) {
        double linear_diff = std::abs(new_linear_velocity - last_linear_velocity_);
        double angular_diff = std::abs(new_angular_velocity - last_angular_velocity_);

        //logInfo("Checking for significant change in cmd_vel - Linear Diff: " + std::to_string(linear_diff) + ", Angular Diff: " + std::to_string(angular_diff));

        if (linear_diff > LINEAR_VELOCITY_THRESHOLD || angular_diff > ANGULAR_VELOCITY_THRESHOLD) {
            //logInfo("Significant change detected - Exceeds Thresholds. Linear Threshold: " + std::to_string(LINEAR_VELOCITY_THRESHOLD) + ", Angular Threshold: " + std::to_string(ANGULAR_VELOCITY_THRESHOLD));
            return true; // Significant change detected
        }

        //logInfo("No significant change in cmd_vel - Within Thresholds. Linear Threshold: " + std::to_string(LINEAR_VELOCITY_THRESHOLD) + ", Angular Threshold: " + std::to_string(ANGULAR_VELOCITY_THRESHOLD));
        return false; // No significant change
    }


    void adjustPWMForError(double error_left, double error_right) {
        double damping_factor = 0.5; // Adjust this value to control the damping effect
        //logInfo("Adjusting PWM for error - Initial Error Left: " + std::to_string(error_left) + ", Initial Error Right: " + std::to_string(error_right));

        // RCLCPP_INFO(this->get_logger(),
        //             "Adjusting PWM for error. Left error: %f, Right error: %f",
        //             error_left, error_right);

        ////logInfo("Adjusting PWM for error. Left error: " + std::to_string(error_left) + ", Right error: " + std::to_string(error_right));

        double max_adjustment = 50.0; // Max PWM adjustment to avoid abrupt changes

        // Calculate the PWM adjustment based on the error 
        double adjustment_left = kP * error_left;
        double adjustment_right = kP * error_right;

        // Limit the adjustment to prevent abrupt changes
        adjustment_left =
            std::clamp(adjustment_left, -max_adjustment, max_adjustment);
        adjustment_right =
            std::clamp(adjustment_right, -max_adjustment, max_adjustment);

        // Temporarily adjust the PWM values based on the error
        double temp_pwmL = desired_pwmL + adjustment_left;
        double temp_pwmR = desired_pwmR + adjustment_right;

        // Ensure the temporarily adjusted PWM values are within the allowed range
        temp_pwmL = std::clamp(temp_pwmL, min_pwm_value, max_pwm_value);
        temp_pwmR = std::clamp(temp_pwmR, min_pwm_value, max_pwm_value);

        // Determine the direction and set the final adjusted PWM values
        if (linear_velocity_ > 0) {
        // Moving forward
        desired_pwmL = std::abs(
            temp_pwmL); // Ensure the left wheel is set for forward movement
        desired_pwmR = -std::abs(
            temp_pwmR); // Ensure the right wheel is set for forward movement
        } else if (linear_velocity_ < 0) {
        // Moving backward
        desired_pwmL = -std::abs(
            temp_pwmL); // Ensure the left wheel is set for backward movement
        desired_pwmR = std::abs(
            temp_pwmR); // Ensure the right wheel is set for backward movement
        } else {
        // Stopping or very slow movement, maintain the current direction with
        // minimal adjustment
        desired_pwmL = temp_pwmL;
        desired_pwmR = temp_pwmR;

        }

    }


    void feedbackCallback(const std_msgs::msg::String::SharedPtr msg) {

        //RCLCPP_INFO(this->get_logger(), "Received feedback: %s", msg->data.c_str());
        //logInfo("Received feedback: " + msg->data);
        FeedbackData feedback;

        parseFeedbackData(msg->data, feedback);

        // Adjust PWM values to synchronize wheel speeds based on feedback
        adjustPWMBasedOnFeedback();

        // Flag to indicate feedback has been processed
        feedback_received_ = true;

            // Update last effective PWM values
        last_effective_pwmL = feedback.pwmL;
        last_effective_pwmR = feedback.pwmR;
    }


   void adjustPWMBasedOnFeedback() {

    //logInfo("Adjusting PWM based on feedback");


   // error_left = desired_rpm_left_ - std::abs(actual_RPM_L);
   // error_right = desired_rpm_right_ - std::abs(actual_RPM_R);

    error_left = desired_rpm_left_ - actual_RPM_L;
    error_right = desired_rpm_right_ + actual_RPM_R;

    // // Adjust PWM values based on errors
    // desired_pwmL += kP * error_left;
    // desired_pwmR += kP * error_right;


    if (new_linear_velocity > 0) {  // Forward movement
        desired_pwmL += kP * error_left;   // Increase left PWM for positive error
        desired_pwmR += kP * error_right;  // Decrease right PWM for positive error (since forward right wheel uses negative PWM)
    } else if (new_linear_velocity < 0) {  // Backward movement
        desired_pwmL -= kP * error_left;   // Decrease left PWM for positive error (since backward left wheel uses negative PWM)
        desired_pwmR -= kP * error_right;  // Increase right PWM for positive error
    } else {
        // Handle stopping or very slow movement
        desired_pwmL = 0;
        desired_pwmR = 0;
    }    


        publishPWM(desired_pwmL, desired_pwmR);


}

    // ===================================================================
    //  ========================   STEP 4   =============================
    // ===================================================================

    void publishPWM(double pwm_left, double pwm_right) {

        if (feedback_received_ || initial_publish_) {
            //logInfo("STEP 1 (publishPWM): Publishing PWM values. Left: " + std::to_string(pwm_left) + ", Right: " + std::to_string(pwm_right));
            std_msgs::msg::Int32 left_wheel_msg, right_wheel_msg;


            if (new_linear_velocity < 0) {  // Forward movement
                intDesired_pwmL = -pwm_left;
                intDesired_pwmR = pwm_right;
            } else if (new_linear_velocity > 0) {  // Backward movement
                intDesired_pwmL = pwm_left;
                intDesired_pwmR = -pwm_right;
            } 
            if (pwm_left != 0 || pwm_right != 0) {
                // Log before clamping PWM values
                std::ostringstream log_stream_before_clamp;
                log_stream_before_clamp << "Before clampPWMValues \t" << intDesired_pwmL << "  \t" << intDesired_pwmR;
                std::string log_message_before_clamp = log_stream_before_clamp.str();
                RCLCPP_INFO(this->get_logger(), "%s", log_message_before_clamp.c_str()); // ROS console logging
                logInfo(log_message_before_clamp); // File logging
            }

            // Clamping PWM values
            intDesired_pwmL = std::min(std::max(intDesired_pwmL, MIN_PWM), MAX_PWM);
            intDesired_pwmR = std::min(std::max(intDesired_pwmR, MIN_PWM), MAX_PWM);

            if (pwm_left != 0 || pwm_right != 0) {
                // Log after clamping PWM values
                std::ostringstream log_stream_after_clamp;
                log_stream_after_clamp << "After clampPWMValues \t" << intDesired_pwmL << "  \t" << intDesired_pwmR;
                std::string log_message_after_clamp = log_stream_after_clamp.str();
                RCLCPP_INFO(this->get_logger(), "%s", log_message_after_clamp.c_str()); // ROS console logging
                logInfo(log_message_after_clamp); // File logging
            }        

            if(new_linear_velocity != 0.0 || actual_RPM_L !=0 ||actual_RPM_R!=0 || pwm_left!=0 || pwm_right!=0 || increment_left !=0 ||increment_right!=0 || error_left!=0 ||error_right!=0 ) {
                // Format the log message first
                std::ostringstream log_stream;
                log_stream << std::fixed << std::setprecision(2) // Set precision for floating-point values
                        << "Li_vel " << new_linear_velocity
                        << "\tD_rpm " << desired_rpm_left_ << " " << desired_rpm_right_
                        << "\tR_rpm " << actual_RPM_L << " " << actual_RPM_R
                        << "\tpwm " << pwm_left << "\t" << pwm_right
                        << "\tinc " << increment_left << "  " << increment_right
                        << "\tErr  " << error_left << "    " << error_right;

                // Convert the stream into a string
                std::string log_message = log_stream.str();

                // Use the formatted message for both RCLCPP_INFO and logInfo
                RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str()); // ROS console logging
                logInfo(log_message); // File logging
            }

            left_wheel_msg.data = intDesired_pwmL;
            right_wheel_msg.data = intDesired_pwmR;

            if (new_linear_velocity == 0 && new_angular_velocity==0) {
                    intDesired_pwmL = 0;
                    intDesired_pwmR = 0;
                    increment_left = 0;
                    increment_right = 0;
                    error_left = 0;
                    error_right = 0;
                } 






            cmd_left_publisher_->publish(left_wheel_msg);
            cmd_right_publisher_->publish(right_wheel_msg);

            feedback_received_ = false;  // Reset feedback flag
            initial_publish_ = false;    // Reset initial publish flag

            //logInfo("STEP 2 (publishPWM): PWM values published.");
        }
    }

    // ===================================================================
    //  ========================   STEP 2   =============================
    // ===================================================================

    void calculateDesiredRPM(double linear_velocity, double angular_velocity,
                            double &desired_rpm_left, double &desired_rpm_right) {
        //logInfo("STEP 1 (calculateDesiredRPM): Calculating desired RPM. Linear vel: " + std::to_string(linear_velocity) + ", Angular vel: " + std::to_string(angular_velocity));

        double v_left = linear_velocity - (angular_velocity * wheel_base_width_ / 2.0);
        double v_right = linear_velocity + (angular_velocity * wheel_base_width_ / 2.0);

        desired_rpm_left = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
        desired_rpm_right = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;

        //logInfo("STEP 2 (calculateDesiredRPM): Desired RPMs calculated. Left: " + std::to_string(desired_rpm_left) + ", Right: " + std::to_string(desired_rpm_right));
    }
    // ===================================================================
    //  ========================   STEP 1   =============================
    // ===================================================================

    double calculateAdjustmentFactor(double new_velocity, double last_velocity) {
        // Simple proportional adjustment; you might need a more sophisticated approach
        return new_velocity / last_velocity;
    }




    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        new_linear_velocity = msg->linear.x;
        new_angular_velocity = msg->angular.z;


            // If there's angular velocity but no linear velocity, set a minimum linear velocity
        if (std::abs(new_linear_velocity) < LINEAR_VELOCITY_THRESHOLD && std::abs(new_angular_velocity) >= ANGULAR_VELOCITY_THRESHOLD) {
            new_linear_velocity = (new_angular_velocity > 0) ? 0.1 : -0.1; // Set a minimum linear velocity for rotation
        }

        // Calculate new desired RPMs based on the new cmd_vel command

        calculateDesiredRPM(new_linear_velocity, new_angular_velocity, new_desired_rpm_left, new_desired_rpm_right);

        // // Check for significant changes in cmd_vel
        // if (significantChangeInCmdVel(new_linear_velocity, new_angular_velocity)) {
        //     initial_publish_ = true;  // Allow publishing due to significant cmd_vel change
        //     //logInfo("Significant cmd_vel change detected");
        // }

        // Calculate the incremental change needed for each wheel
        increment_left = (new_desired_rpm_left - desired_rpm_left_) * kP;
        increment_right = (new_desired_rpm_right - desired_rpm_right_) * kP;

        //logInfo("STEP 3 (cmvVelCallback): Incremental Changes - Left: " + std::to_string(increment_left) + ", Right: " + std::to_string(increment_right));
        // Apply the incremental change to the current PWM values
        desired_pwmL += increment_left;
        desired_pwmR += increment_right;


        //         // Calculate the incremental change needed for each wheel
        // double adjustment_factor_left = calculateAdjustmentFactor(new_desired_rpm_left, desired_rpm_left_);
        // double adjustment_factor_right = calculateAdjustmentFactor(new_desired_rpm_right, desired_rpm_right_);

        // // Adjust based on previous effective PWM and the adjustment factor
        // desired_pwmL = last_effective_pwmL * adjustment_factor_left;
        // desired_pwmR = last_effective_pwmR * adjustment_factor_right;


        ////logInfo("STEP 4 (cmvVelCallback): Applied Incremental Changes - PWM Left: " + std::to_string(desired_pwmL) + ", PWM Right: " + std::to_string(desired_pwmR));

        // Update the desired RPMs to the new values
        desired_rpm_left_ = new_desired_rpm_left;
        desired_rpm_right_ = new_desired_rpm_right;


        // Log and publish the updated PWM values
        //logInfo("STEP 5 (cmvVelCallback): Updated PWM values after cmd_vel change: Left=" + std::to_string(desired_pwmL) + ", Right: " + std::to_string(desired_pwmR));

        // Update the last cmd_vel command values
        last_linear_velocity_ = new_linear_velocity;
        last_angular_velocity_ = new_angular_velocity;


    }


    double wheel_base_width_;
    double wheel_radius_;
    double pwm_scale_;
    double kP;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_subscriber_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
        cmd_left_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
        cmd_right_publisher_;
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
