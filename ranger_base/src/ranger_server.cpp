// ranger_server.cpp
// Author: Ziad Ammar
// Date: 22-6-2025
// Description: ROS 2 node for ranger_base package to be server of ros2 services and actions for Ranger mobile base
// TODO LIST:
// [ ] Use ROS2 Parameters for default values and dynamic reconfiguration
// [ ] Parking Mode Service Implementation
// [ ] Search For Marker Action Implementation
// [ ] Integrate diagnostics and status publishing
// [ ] Check /aruco_markers topic test since it will exist with the topic being subscribed to (check publisher node instead of topic)
// [ ] Update documentation 

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
      follow_marker_id(-1),
      follow_active(false),
      marker_detected(false),
      min_distance(0.75),
      linear_vel(0.5),
      angular_vel(0.75),
      distance_to_marker(-1.0),
      curr_linear_vel(0.0),
      curr_angular_vel(0.0),
      min_linear_vel(0.35)
    {
        RCLCPP_INFO(this->get_logger(), "RangerServer node has been started.");

        // Create action server
        this->follow_action_server = rclcpp_action::create_server<ArucoFollow>(
            this,
            "follow_aruco",
            std::bind(&RangerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RangerServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&RangerServer::handle_accepted, this, std::placeholders::_1));

        // Initialize action follow_result and follow_feedback
        follow_result   = std::make_shared<ArucoFollow::Result>();
        follow_feedback = std::make_shared<ArucoFollow::Feedback>();
        
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
    rclcpp_action::Server<ArucoFollow>::SharedPtr follow_action_server;
    
    // Publishers and subscribers
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer;
    
    // Action Result
    std::shared_ptr<ArucoFollow::Result> follow_result;

    // Action Feedback 
    std::shared_ptr<ArucoFollow::Feedback> follow_feedback;
    
    // Current goal handle
    std::shared_ptr<GoalHandleArucoFollow> current_goal_handle;
    
    // State variables
    int64_t follow_marker_id;
    bool follow_active;
    geometry_msgs::msg::Pose current_marker_pose;
    bool marker_detected;
    rclcpp::Time last_marker_time;
    
    // Parameters
    float min_distance;
    float linear_vel;
    float angular_vel;
    // Feedback variables
    float distance_to_marker;
    float curr_linear_vel;
    float curr_angular_vel;

    float min_linear_vel;

    // Goal handling
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArucoFollow::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request to follow marker ID: %ld", goal->marker_id);
        
        // Check if another action is already running
        if (follow_active) {
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
        follow_active = false;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleArucoFollow> goal_handle)
    {
        // Get goal request parameters
        auto goal = goal_handle->get_goal();
        // Mandatory request parameters
        follow_marker_id = goal->marker_id;
        min_distance     = goal->min_distance;
        // Optional parameters with defaults
        linear_vel  = (goal->linear_vel > 0.0) ? goal->linear_vel : linear_vel;
        angular_vel = (goal->angular_vel > 0.0) ? goal->angular_vel : angular_vel;
        
        // Check validity of goal parameters
        bool valid = true;
        std::string missing_fields;

        // Check if marker_id is specified
        if (follow_marker_id <= 0) {
            valid = false;
            missing_fields += "marker_id ";
        }
        // Check if min_distance is specified
        if (min_distance <= 0.0) {
            valid = false;
            missing_fields += "min_distance ";
        }

        if (valid) {
            current_goal_handle = goal_handle;
            follow_active       = true;
            marker_detected     = false;

            RCLCPP_INFO(this->get_logger(), 
                "Starting to follow marker ID: %ld | min_distance: %.2f | linear_vel: %.2f | angular_vel: %.2f",
                follow_marker_id, min_distance, linear_vel, angular_vel);

            // Check if aruco_markers topic is available
            check_aruco_topic_availability();
        } else {
            follow_active = false;
            RCLCPP_WARN(this->get_logger(),
                "Action request missing required field(s) or incorrect values: %s. Please specify them in the action request.",
                missing_fields.c_str());
            follow_result->success = false;
            follow_result->message = "Action request failed due to missing or invalid parameters: " + missing_fields;
            goal_handle->abort(follow_result);
            return;
        }
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
        if (!follow_active) return;
        
        // Look for the target marker in the message
        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            if (msg->marker_ids[i] == follow_marker_id) {
                current_marker_pose = msg->poses[i];
                marker_detected = true;
                last_marker_time = this->get_clock()->now();
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Detected target marker %ld at position (%.2f, %.2f, %.2f)", 
                    follow_marker_id,
                    current_marker_pose.position.x,
                    current_marker_pose.position.y,
                    current_marker_pose.position.z);
                return;
            }
        }
    }

    void control_loop()
    {
        if (!follow_active || !current_goal_handle) return;
        
        // Check if marker was detected recently (within 2 seconds)
        if (!marker_detected || 
            (this->get_clock()->now() - last_marker_time).seconds() > 2.0) {
            
            if (marker_detected) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Marker %ld not detected recently. Stopping robot.", follow_marker_id);
            }
            
            stop_robot();
            return;
        }
        
        // Calculate distance to marker 
        distance_to_marker = calculate_distance_to_marker();
        
        // Send follow_feedback
        follow_feedback->distance_to_marker = std::round(distance_to_marker * 1000.0f) / 1000.0f;
        follow_feedback->curr_linear_vel    = std::round(curr_linear_vel * 1000.0f) / 1000.0f;
        follow_feedback->curr_angular_vel   = std::round(curr_angular_vel * 1000.0f) / 1000.0f;
        current_goal_handle->publish_feedback(follow_feedback);
        
        // Check if we've reached the minimum distance
        if (distance_to_marker <= min_distance) {
            RCLCPP_INFO(this->get_logger(), 
                "Reached minimum distance (%.2f m) to marker %ld", min_distance, follow_marker_id);
            
            stop_robot();
            
            // Send success follow_result
            follow_result->message = "Successfully reached marker " + std::to_string(follow_marker_id);
            follow_result->success = true;
            
            current_goal_handle->succeed(follow_result);
            follow_active = false;
            return;
        }
        
        // Calculate and publish velocity commands
        move_towards_marker();
    }

    float calculate_distance_to_marker()
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
        float angle_to_marker = std::atan2(
            current_marker_pose.position.y,
            current_marker_pose.position.x
        );
        
        // Angular velocity (turn towards marker)
        if (std::abs(angle_to_marker) > 0.1) { // 0.1 radian threshold
            twist_msg.angular.z = std::copysign(angular_vel, angle_to_marker);
            
            // No linear speed when turning
            twist_msg.linear.x = 0.0;
        } else {
            // Move forward when aligned
            twist_msg.angular.z = 0.0;
            
            // use minimum linear vel 0.5 m from the minimum distance
            if (distance_to_marker < min_distance + 0.5) {
                twist_msg.linear.x = std::min(min_linear_vel, linear_vel);
            } else {
                twist_msg.linear.x = linear_vel;
            }                     
        }
        
        // Update current velocities
        curr_linear_vel  = twist_msg.linear.x;
        curr_angular_vel = twist_msg.angular.z;
        // Publish the velocity command
        cmd_vel_publisher->publish(twist_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Moving: linear=%.2f, angular=%.2f, distance=%.2f, angle=%.2f",
            twist_msg.linear.x, twist_msg.angular.z, distance_to_marker, angle_to_marker);
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