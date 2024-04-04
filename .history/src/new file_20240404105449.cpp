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
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
        log_publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        // Deactivate the publishers
        shutdownHoverboard();
        cmd_left_publisher_->on_deactivate();
        cmd_right_publisher_->on_deactivate();
        log_publisher_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        // Cleanup the resources

        shutdownHoverboard();
        cmd_vel_subscriber_.reset();
        feedback_subscriber_.reset();
        cmd_left_publisher_.reset();
        cmd_right_publisher_.reset();
        log_publisher_.reset();

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


        log_publisher_ = this->create_publisher<std_msgs::msg::String>("bridge_feedback", 10);

        return CallbackReturn::SUCCESS;
    }

private:


    std_msgs::msg::String feedback_msg;


    std::ostringstream ss;
    size_t pwm_index; // Az aktuális index a körkörös tömbhöz

        // Variables to control dynamic adjustments
    double current_kP = LOW_KP;
    int current_sampling_rate_ms = SLOW_SAMPLING_RATE_MS;


    std::ofstream log_file_, log_file2_; // File stream for logging
    
    double min_pwm_value = -500, max_pwm_value = 500;
    double actual_RPM_L = 0, actual_RPM_R = 0;
    double desired_pwmL = 0, desired_pwmR = 0;
    int febPwmR = 0, febPwmL = 0;
    double linear_velocity_ = 0; // Current lWinear velocity from cmd_vel
    double angular_velocity_ = 0;

    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0;

    double desired_rpm_left_ = 0, desired_rpm_right_ = 0; 


    const double LINEAR_VELOCITY_THRESHOLD = 0.01; // Adjust as needed
    const double ANGULAR_VELOCITY_THRESHOLD = 0.01; // Adjust as needed
    const double Turn_on_Spot = 2.9; // Adjust as needed
    bool initial_publish_ = true;
    bool feedback_received_ = false; // Flag to indicate feedback received
    double increment_left = 0;
    double increment_right = 0;

    double new_desired_rpm_left, new_desired_rpm_right;

    double error_left_RPM = 0;
    double error_right_RPM = 0;
    double error_left_RPM_percent = 0;
    double error_right_RPM_percent = 0;

    double new_linear_velocity;
    double new_angular_velocity;
    const int MIN_PWM = -500;  // Adjust this value based on your hardware limits
    const int MAX_PWM = 500;   // Adjust this value based on your hardware limits
    const int turn_on_spot_speed_RPM = 15; // RPM

    int intDesired_pwmL = 0;
    int intDesired_pwmR = 0;


    double last_effective_pwmL = 0;
    double last_effective_pwmR = 0;

    double damping_factor = 0.9; // Adjust this value to control the damping effect

    int minimumPwmb = 45;

    double wheel_base_width_;
    double wheel_radius_;
    double pwm_scale_;
    double kP;
    const double wheelCircumference = 0.534;  // (2.0 * M_PI * wheel_radius_); // 0.534055  ; // Circumference in meters

    bool BoolTturnsOnTheSpot = false;
    bool prewTurnsOnTheSpot = false;

    bool testMode = false;

    // const int stableThreshold = 2; // RPM error threshold for stability
    // const int stableCountRequired = 5; // Number of consecutive stable readings required
    // int stableCount = 0; // Current count of consecutive stable readings



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

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse feedback data.");
            //logInfo("Failed to parse feedback data.");
        }
    }

/*
    bool isStableSpeed() {
            double error_left = std::abs(desired_rpm_left_ - actual_RPM_L);
            double error_right = std::abs(desired_rpm_right_ - actual_RPM_R);

            if (error_left <= stableThreshold && error_right <= stableThreshold) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            return stableCount >= stableCountRequired;
    }
*/

    void adjustPWMForError(double error_left, double error_right) {

        double max_adjustment  =  100.0; // Max PWM adjustment to avoid abrupt changes

        // Calculate the PWM adjustment based on the error 
        double adjustment_left = kP * error_left * damping_factor;;
        double adjustment_right = kP * error_right * damping_factor;

        // Limit the adjustment to prevent abrupt changes
        adjustment_left  =  std::clamp(adjustment_left, -max_adjustment, max_adjustment);
        adjustment_right  =  std::clamp(adjustment_right, -max_adjustment, max_adjustment);
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "adjustment_left -- adjustment_right %0.2f \t %0.2f",adjustment_left,adjustment_right);

        // Temporarily adjust the PWM values based on the error
        double temp_pwmL  =  desired_pwmL + adjustment_left;
        double temp_pwmR  =  desired_pwmR + adjustment_right;

        RCLCPP_INFO(this->get_logger(), "temp_pwmR -- temp_pwmL %0.2f \t %0.2f",temp_pwmR,temp_pwmL);
        // Ensure the temporarily adjusted PWM values are within the allowed range
        temp_pwmL = std::clamp(temp_pwmL, min_pwm_value, max_pwm_value);
        temp_pwmR = std::clamp(temp_pwmR, min_pwm_value, max_pwm_value);

        if (linear_velocity_ > 0) //&& std::abs(angular_velocity_)<Turn_on_Spot ) 
        {
        // Moving forward
            desired_pwmL  =  std::abs( temp_pwmL); // Ensure the left wheel is set for forward movement
            desired_pwmR  =  -std::abs( temp_pwmR); // Ensure the right wheel is set for forward movement
        } else if (linear_velocity_ < 0 ) {
        // Moving backward
            desired_pwmL  =  -std::abs( temp_pwmL); // Ensure the left wheel is set for backward movement
            desired_pwmR  =  std::abs( temp_pwmR); // Ensure the right wheel is set for backward movement
        } else {
            desired_pwmL  =  temp_pwmL;
            desired_pwmR  =  temp_pwmR;
        }
    }


    void feedbackCallback(const std_msgs::msg::String::SharedPtr msg) {

        FeedbackData feedback;

        parseFeedbackData(msg->data, feedback);

        adjustPWMBasedOnFeedback();

        feedback_received_ = true;

        last_effective_pwmL = feedback.pwmL;
        last_effective_pwmR = feedback.pwmR;
    }


   void adjustPWMBasedOnFeedback() {    
        // Adjust PWM based on errors and new_linear_velocity
        if (new_linear_velocity > LINEAR_VELOCITY_THRESHOLD) {
                        // Azonnali "rúgás" a mozgás elindításához, ha még nem érte el a minimális értéket                                 
           if (std::abs(desired_pwmL) < minimumPwmb && std::abs(new_angular_velocity) < Turn_on_Spot)
            {        
                if (desired_rpm_left_ < 0) 
                { 
                    desired_pwmL = -minimumPwmb;
                }else{
                    desired_pwmL = minimumPwmb;
                }
            }
            if (std::abs(desired_pwmR) < minimumPwmb && std::abs(new_angular_velocity) < Turn_on_Spot)
            {
                if (desired_rpm_right_ < 0) 
                { 
                    desired_pwmR = -minimumPwmb;
                }else{
                    desired_pwmR = minimumPwmb;
                }
            }
            // Moving forward
            // desired_pwmL = 70;
            // desired_pwmR = -70;
            desired_pwmL += kP * error_left_RPM * damping_factor;
            // Apply a 5% increase for the right wheel when moving forward
            desired_pwmR += kP * error_right_RPM * damping_factor;
           
        } else if (new_linear_velocity < -LINEAR_VELOCITY_THRESHOLD) 
        {         
            if (std::abs(desired_pwmL) < minimumPwmb && std::abs(new_angular_velocity) < Turn_on_Spot)
            {
                if (desired_rpm_left_ < 0) {
                    desired_pwmL = minimumPwmb;
                }else{
                    desired_pwmL = -minimumPwmb;
                }
            }
            if (std::abs(desired_pwmR) < minimumPwmb && std::abs(new_angular_velocity) < Turn_on_Spot)
            {  
                if (desired_rpm_right_ < 0) {
                    desired_pwmR = minimumPwmb;
                }else{
                    desired_pwmR = -minimumPwmb;
                }                
            }              
            // desired_pwmL = -70;
            // desired_pwmR = 70;
            // Moving backward

            desired_pwmL -= kP * error_left_RPM * damping_factor;
            desired_pwmR -= kP * error_right_RPM * damping_factor;          
        } else {
            // Stop
            
            desired_pwmL = 0;
            desired_pwmR = 0;
            resetWheelCompensation();
        }
        // Finally, publish the adjusted PWM values
        publishPWM(desired_pwmL, desired_pwmR);
    }


    void publishPWM(double pwm_left, double pwm_right) {

        if (feedback_received_ || initial_publish_) {
            //logInfo("STEP 1 (publishPWM): Publishing PWM values. Left: " + std::to_string(pwm_left) + ", Right: " + std::to_string(pwm_right));
            std_msgs::msg::Int32 left_wheel_msg, right_wheel_msg;

            intDesired_pwmL = pwm_left;
            intDesired_pwmR = pwm_right;

            if (pwm_left != 0 || pwm_right != 0) {
                // Log before clamping PWM values
                std::ostringstream log_stream_before_clamp;
                log_stream_before_clamp << "Before clampPWMValues \t" << intDesired_pwmL << "  \t" << intDesired_pwmR;
                std::string log_message_before_clamp = log_stream_before_clamp.str();
                //RCLCPP_INFO(this->get_logger(), "%s", log_message_before_clamp.c_str()); // ROS console logging
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
                //RCLCPP_INFO(this->get_logger(), "%s", log_message_after_clamp.c_str()); // ROS console logging
                logInfo(log_message_after_clamp); // File logging
            }     


            if(new_linear_velocity != 0.0 || actual_RPM_L !=0 ||actual_RPM_R!=0 || pwm_left!=0 || pwm_right!=0 ||  error_left_RPM!=0 ||error_right_RPM!=0 ) {
                // Format the log message first
                std::ostringstream log_stream;
                log_stream << std::fixed << std::setprecision(2) // Set precision for floating-point values
                        << "Li_vel " << new_linear_velocity
                        << "\tD_rpm " << desired_rpm_left_ << " " << desired_rpm_right_
                        << "\tR_rpm " << actual_RPM_L << " " << actual_RPM_R
                        << "\tpwm " << pwm_left << "\t" << pwm_right
                        << "\tinc " << increment_left << "  " << increment_right
                        << "\tErr  " << error_left_RPM << "    " << error_right_RPM
                        << "\tErr%  " << error_left_RPM_percent << "    " << error_right_RPM_percent;


                // Convert the stream into a string
                std::string log_message = log_stream.str();

                feedback_msg.data = log_message;


                // Use the formatted message for both RCLCPP_INFO and logInfo
                RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str()); // ROS console logging
                logInfo(log_message); // File logging
            }

       
            left_wheel_msg.data = intDesired_pwmL;
            right_wheel_msg.data = intDesired_pwmR;           
           
    
            if (new_linear_velocity == 0 && std::abs(new_angular_velocity) < Turn_on_Spot) 
            {
                resetWheelCompensation();
                left_wheel_msg.data = 0;
                right_wheel_msg.data = 0;    
            }


            cmd_left_publisher_->publish(left_wheel_msg);
            cmd_right_publisher_->publish(right_wheel_msg);

            feedback_received_ = false;  // Reset feedback flag
            initial_publish_ = false;    // Reset initial publish flag
        }
    }


    void calculateDesiredRPM(double linear_velocity, double angular_velocity, double& desired_rpm_left, double& desired_rpm_right) 
    {
        
        //logInfo("STEP 1 (calculateDesiredRPM): Calculating desired RPM. Linear vel: " + std::to_string(linear_velocity) + ", Angular vel: " + std::to_string(angular_velocity));
        
        double v_left = 0;
        double v_right = 0;     

        if (std::abs(angular_velocity) > Turn_on_Spot)       //  turn around in palcae
        {   /*
            if ( !BoolTturnsOnTheSpot )
            {
                desired_rpm_left = 0;
                desired_rpm_right = 0;

                resetWheelCompensation();

                //RCLCPP_INFO(this->get_logger(), "\033[32m Turns on the spot \033[0m");  

                BoolTturnsOnTheSpot = true;
            }

            new_linear_velocity =  0.1; 

            if ( angular_velocity > 0 )                        //  Left
            {
                desired_rpm_left = -turn_on_spot_speed_RPM;
                desired_rpm_right = -turn_on_spot_speed_RPM;

                error_left_RPM = desired_rpm_left - actual_RPM_L;
                error_right_RPM = desired_rpm_right - actual_RPM_R;
            }else                                             //  Right
            {            
                desired_rpm_left = turn_on_spot_speed_RPM;
                desired_rpm_right = turn_on_spot_speed_RPM;

                error_left_RPM = desired_rpm_left - actual_RPM_L;
                error_right_RPM = desired_rpm_right - actual_RPM_R;
            }    */    
        }
        else
        {
            if ( BoolTturnsOnTheSpot )   //  finished  turn around in palcae
            {         
                desired_rpm_left = 0;
                desired_rpm_right = 0;

                resetWheelCompensation();

                //RCLCPP_INFO(this->get_logger(), "\033[32m Turns in a circular arc \033[0m");   
                BoolTturnsOnTheSpot = false;         
            }
            
            if ( std::abs(linear_velocity) > LINEAR_VELOCITY_THRESHOLD )
            {
                //v_left = linear_velocity - ((angular_velocity /( 1 / linear_velocity)) * wheel_base_width_ / 2.0);
                //v_right = linear_velocity + ((angular_velocity /( 1 / linear_velocity)) * wheel_base_width_ / 2.0);

                v_left = linear_velocity - (angular_velocity * wheel_base_width_ / 2.0);
                v_right = linear_velocity + (angular_velocity * wheel_base_width_ / 2.0);

                if (linear_velocity > LINEAR_VELOCITY_THRESHOLD )
                {
                    desired_rpm_left = (v_left / wheelCircumference) * 60.0;
                    desired_rpm_right = -(v_right / wheelCircumference) * 60.0;

                    error_left_RPM = desired_rpm_left - actual_RPM_L;
                    error_right_RPM = desired_rpm_right - actual_RPM_R;  
                    error_left_RPM_percent = (desired_rpm_left - actual_RPM_L) / (desired_rpm_left/100) ;
                    error_right_RPM_percent = (desired_rpm_right - actual_RPM_R) / (desired_rpm_right/100) ;      

                }else if(linear_velocity < -LINEAR_VELOCITY_THRESHOLD )
                {
                    desired_rpm_left = -(v_left / wheelCircumference) * 60.0;
                    desired_rpm_right = (v_right / wheelCircumference) * 60.0;

                    error_left_RPM = desired_rpm_left + actual_RPM_L;
                    error_right_RPM = desired_rpm_right + actual_RPM_R;      
                    error_left_RPM_percent = (desired_rpm_left + actual_RPM_L) / (desired_rpm_left/100) ;
                    error_right_RPM_percent = (desired_rpm_right + actual_RPM_R) / (desired_rpm_right/100) ;               
                }
            }
            else
            {
                desired_rpm_left = 0;
                desired_rpm_right = 0;
                resetWheelCompensation();
            }
        }

        //desired_rpm_left = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
        //desired_rpm_right = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;

        // If there's angular velocity but no linear velocity, set a minimum linear velocity       

    }
    void resetWheelCompensation()
    {
        intDesired_pwmL = 0;
        intDesired_pwmR = 0;        
        error_left_RPM = 0;
        error_right_RPM = 0;
        increment_left = 0;
        increment_right = 0; 
        desired_pwmL = 0;
        desired_pwmR = 0;
        desired_rpm_left_ = 0;
        desired_rpm_right_ = 0;
       
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        new_linear_velocity = msg->linear.x;
        new_angular_velocity = msg->angular.z;
            
        // Only calculate new desired RPMs if the vehicle hasn't reached a stable speed
        calculateDesiredRPM(new_linear_velocity, new_angular_velocity, new_desired_rpm_left, new_desired_rpm_right);

        //    Calculate the incremental change needed for each wheel
        //increment_left = (new_desired_rpm_left - desired_rpm_left_) * kP;
        //increment_right = (new_desired_rpm_right - desired_rpm_right_) * kP;

        // Calculate the incremental change needed for each wheel
       // increment_left = (new_desired_rpm_left - desired_rpm_left_) * kP;
       // increment_right = (new_desired_rpm_right - desired_rpm_right_) * kP;

        // Apply the incremental change to the current PWM values

      //  desired_pwmL += increment_left;
      //  desired_pwmR += increment_right;

        // Update the desired RPMs to the new values
        desired_rpm_left_ = new_desired_rpm_left;
        desired_rpm_right_ = new_desired_rpm_right;

        // Update the last cmd_vel command values
        last_linear_velocity_ = new_linear_velocity;
        last_angular_velocity_ = new_angular_velocity;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_subscriber_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
        cmd_left_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
        cmd_right_publisher_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr log_publisher_;
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
