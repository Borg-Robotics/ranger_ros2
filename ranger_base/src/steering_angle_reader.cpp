/**
 * @file steering_angle_reader.cpp
 * @brief Node to read CAN frame 0x271 (motor angles) and publish steering angle in radians and degrees
 * @author Assistant
 * @date 2025-08-02
 */

#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "ranger_msgs/msg/steering_angle.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

using namespace westonrobot;

class SteeringAngleReader : public rclcpp::Node
{
public:
    SteeringAngleReader() : Node("steering_angle_reader")
    {
        // Declare parameters
        this->declare_parameter<std::string>("port_name", "can0");
        this->declare_parameter<std::string>("robot_model", "ranger_mini_v2");
        this->declare_parameter<int>("update_rate", 50);

        // Get parameters
        port_name_ = this->get_parameter("port_name").as_string();
        robot_model_ = this->get_parameter("robot_model").as_string();
        update_rate_ = this->get_parameter("update_rate").as_int();

        // Create robot instance based on robot model
        if (robot_model_ == "ranger_mini_v1") {
            robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
        } else if (robot_model_ == "ranger_mini_v2") {
            robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
        } else if (robot_model_ == "ranger_mini_v3") {
            robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
        } else {
            robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
        }
        
        // Setup robot
        if (!robot_->Connect(port_name_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot on port: %s", port_name_.c_str());
            return;
        }

        // Setup publisher
        steering_angle_pub_ = this->create_publisher<ranger_msgs::msg::SteeringAngle>(
            "steering_angle", 10);

        // Setup timer for periodic publishing
        auto timer_period = std::chrono::milliseconds(1000 / update_rate_);
        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&SteeringAngleReader::publish_steering_angle, this));

        RCLCPP_INFO(this->get_logger(), "Steering angle reader node started");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Robot model: %s, Update rate: %d Hz", 
                   port_name_.c_str(), robot_model_.c_str(), update_rate_);
    }

private:
    void publish_steering_angle()
    {
        // Get actuator state which contains motor angles from CAN frame 0x271
        auto actuator_state = robot_->GetActuatorState();
        
        // Create steering angle message
        auto msg = ranger_msgs::msg::SteeringAngle();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

        // Extract individual wheel angles (in radians)
        // For Ranger, angles 5-8 typically correspond to the steering motors
        msg.front_left_rad = actuator_state.motor_angles.angle_5;
        msg.front_right_rad = actuator_state.motor_angles.angle_6;
        msg.rear_left_rad = actuator_state.motor_angles.angle_7;
        msg.rear_right_rad = actuator_state.motor_angles.angle_8;

        // Convert to degrees
        msg.front_left_deg = msg.front_left_rad * 180.0 / M_PI;
        msg.front_right_deg = msg.front_right_rad * 180.0 / M_PI;
        msg.rear_left_deg = msg.rear_left_rad * 180.0 / M_PI;
        msg.rear_right_deg = msg.rear_right_rad * 180.0 / M_PI;

        // Calculate average steering angle for the vehicle
        // For a 4-wheel steering vehicle, we can use the front wheels average
        // or implement a more sophisticated calculation based on the vehicle kinematics
        msg.angle_rad = (msg.front_left_rad + msg.front_right_rad) / 2.0;
        msg.angle_deg = msg.angle_rad * 180.0 / M_PI;

        // Publish the message
        steering_angle_pub_->publish(msg);

        // Log debug info (throttled)
        if (debug_counter_++ % (update_rate_ * 2) == 0) { // Log every 2 seconds
            RCLCPP_DEBUG(this->get_logger(), 
                "Steering angles - FL: %.3f°, FR: %.3f°, RL: %.3f°, RR: %.3f°, Avg: %.3f°",
                msg.front_left_deg, msg.front_right_deg, 
                msg.rear_left_deg, msg.rear_right_deg, msg.angle_deg);
        }
    }

    // Parameters
    std::string port_name_;
    std::string robot_model_;
    int update_rate_;

    // Robot interface
    std::shared_ptr<RangerRobot> robot_;

    // ROS components
    rclcpp::Publisher<ranger_msgs::msg::SteeringAngle>::SharedPtr steering_angle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Debug counter for throttled logging
    int debug_counter_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SteeringAngleReader>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in steering angle reader: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
