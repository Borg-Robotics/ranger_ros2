// ranger_server.cpp
// Author: Ziad Ammar
// Date: 22-6-2025
// Description: ROS 2 node for ranger_base package to be server of ros2 services and actions for Ranger mobile base

#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <ranger_msgs/action/aruco_follow.hpp>

using namespace std::chrono_literals;

namespace ranger_base
{

class RangerServer : public rclcpp::Node
{
public:
    using ArucoFollow = ranger_msgs::action::ArucoFollow;
    using GoalHandleArucoFollow = rclcpp_action::ServerGoalHandle<ArucoFollow>;

    RangerServer()
    : Node("ranger_server"),
      target_marker_id(-1),
      action_active(false),
      marker_detected(false),
      min_distance(0.75),
      linear_speed(0.75),
      angular_speed(0.75)
    {
        RCLCPP_INFO(this->get_logger(), "RangerServer node has been started.");

        // Create action server
        this->action_server = rclcpp_action::create_server<ArucoFollow>(
            this,
            "follow_aruco",
            std::bind(&RangerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RangerServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&RangerServer::handle_accepted, this, std::placeholders::_1));

        // Create subscribers and publishers
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10,
            std::bind(&RangerServer::aruco_callback, this, std::placeholders::_1));

        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for control loop
        control_timer = this->create_wall_timer(
            100ms, std::bind(&RangerServer::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Action server '/follow_aruco' is ready.");
    }

private:
    // Action server
    rclcpp_action::Server<ArucoFollow>::SharedPtr action_server;
    
    // Publishers and subscribers
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer;
    
    // Current goal handle
    std::shared_ptr<GoalHandleArucoFollow> current_goal_handle;
    
    // State variables
    int64_t target_marker_id;
    bool action_active;
    geometry_msgs::msg::Pose current_marker_pose;
    bool marker_detected;
    rclcpp::Time last_marker_time;
    
    // Parameters
    double min_distance;
    double linear_speed;
    double angular_speed;

    // Goal handling
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArucoFollow::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request to follow marker ID: %ld", goal->marker_id);
        
        // Check if another action is already running
        if (action_active) {
            RCLCPP_WARN(this->get_logger(), "Another follow action is already active. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleArucoFollow> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        
        // Stop the robot
        stop_robot();
        action_active = false;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleArucoFollow> goal_handle)
    {
        // Start the action execution
        target_marker_id    = goal_handle->get_goal()->marker_id;
        current_goal_handle = goal_handle;
        action_active       = true;
        marker_detected     = false;
        
        RCLCPP_INFO(this->get_logger(), "Starting to follow marker ID: %ld", target_marker_id);
        
        // Check if aruco_markers topic is available
        check_aruco_topic_availability();
    }

    void check_aruco_topic_availability()
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        bool topic_found = false;
        
        for (const auto& topic : topic_names_and_types) {
            if (topic.first == "/aruco_markers") {
                topic_found = true;
                break;
            }
        }
        
        if (!topic_found) {
            RCLCPP_WARN(this->get_logger(), 
                "Warning: /aruco_markers topic is not available. Make sure the ArUco detection node is running.");
        } else {
            RCLCPP_INFO(this->get_logger(), "/aruco_markers topic is available.");
        }
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (!action_active) return;
        
        // Look for the target marker in the message
        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            if (msg->marker_ids[i] == target_marker_id) {
                current_marker_pose = msg->poses[i];
                marker_detected = true;
                last_marker_time = this->get_clock()->now();
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Detected target marker %ld at position (%.2f, %.2f, %.2f)", 
                    target_marker_id,
                    current_marker_pose.position.x,
                    current_marker_pose.position.y,
                    current_marker_pose.position.z);
                return;
            }
        }
    }

    void control_loop()
    {
        if (!action_active || !current_goal_handle) return;
        
        // Check if marker was detected recently (within 2 seconds)
        if (!marker_detected || 
            (this->get_clock()->now() - last_marker_time).seconds() > 2.0) {
            
            if (marker_detected) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Marker %ld not detected recently. Stopping robot.", target_marker_id);
            }
            
            stop_robot();
            return;
        }
        
        // Calculate distance to marker
        double distance = calculate_distance_to_marker();
        
        // Send feedback
        auto feedback = std::make_shared<ArucoFollow::Feedback>();
        feedback->distance_to_marker = static_cast<float>(distance);
        current_goal_handle->publish_feedback(feedback);
        
        // Check if we've reached the minimum distance
        if (distance <= min_distance) {
            RCLCPP_INFO(this->get_logger(), 
                "Reached minimum distance (%.2f m) to marker %ld", min_distance, target_marker_id);
            
            stop_robot();
            
            // Send success result
            auto result = std::make_shared<ArucoFollow::Result>();
            result->message = "Successfully reached marker " + std::to_string(target_marker_id);
            result->success = true;
            
            current_goal_handle->succeed(result);
            action_active = false;
            return;
        }
        
        // Calculate and publish velocity commands
        move_towards_marker();
    }

    double calculate_distance_to_marker()
    {
        return std::sqrt(
            std::pow(current_marker_pose.position.x, 2) +
            std::pow(current_marker_pose.position.y, 2)
        );
    }

    void move_towards_marker()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Calculate angle to marker
        double angle_to_marker = std::atan2(
            current_marker_pose.position.y,
            current_marker_pose.position.x
        );
        
        // Calculate distance
        double distance = calculate_distance_to_marker();
        
        // Angular velocity (turn towards marker)
        if (std::abs(angle_to_marker) > 0.1) { // 0.1 radian threshold
            twist_msg.angular.z = std::copysign(angular_speed, angle_to_marker);
            
            // Reduce linear speed when turning
            twist_msg.linear.x = linear_speed * 0.3;
        } else {
            // Move forward when aligned
            twist_msg.angular.z = 0.0;
            
            // Reduce speed as we get closer
            double speed_factor = std::min(1.0, (distance - min_distance) / 1.0);
            speed_factor = std::max(0.1, speed_factor); // Minimum speed factor
            
            twist_msg.linear.x = linear_speed * speed_factor;
        }
        
        cmd_vel_publisher->publish(twist_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Moving: linear=%.2f, angular=%.2f, distance=%.2f, angle=%.2f",
            twist_msg.linear.x, twist_msg.angular.z, distance, angle_to_marker);
    }

    void stop_robot()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        
        cmd_vel_publisher->publish(twist_msg);
    }
};

} // namespace ranger_base

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ranger_base::RangerServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}