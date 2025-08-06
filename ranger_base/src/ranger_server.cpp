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
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <ranger_msgs/action/aruco_follow.hpp>
#include <ranger_msgs/action/aruco_search.hpp>
#include <ranger_msgs/action/aruco_strafe.hpp>

using namespace std::chrono_literals;

namespace ranger_base
{

class RangerServer : public rclcpp::Node
{
public:
    using ArucoFollow = ranger_msgs::action::ArucoFollow;
    using GoalHandleArucoFollow = rclcpp_action::ServerGoalHandle<ArucoFollow>;

    using ArucoSearch = ranger_msgs::action::ArucoSearch;
    using GoalHandleArucoSearch = rclcpp_action::ServerGoalHandle<ArucoSearch>;

    using ArucoStrafe = ranger_msgs::action::ArucoStrafe;
    using GoalHandleArucoStrafe = rclcpp_action::ServerGoalHandle<ArucoStrafe>;

    RangerServer()
    : Node("ranger_server"),
      // Follow Action Variables
      follow_marker_id(-1),
      follow_active(false),
      min_distance(0.75),
      linear_vel(0.5),
      angular_vel(0.75),
      // Search Action Variables
      search_marker_id(-1),
      search_active(false),
      search_start_time(this->get_clock()->now()),
      // Search velocity profile variables
      current_search_angular_velocity(0.0),
      last_search_control_time(this->get_clock()->now()),
      search_velocity_profile_initialized(false),
      search_deceleration_phase(false),
      // IMU variables
      initial_quaternion_set(false),
      current_rotation_angle(0.0),
      cumulative_rotation(0.0),
      // Strafe Action Variables
      strafe_marker_id(-1),
      strafe_active(false),
      strafe_direction(1),
      strafe_start_time(this->get_clock()->now()),
      strafe_marker_detected(false),
      current_strafe_velocity(0.0),
      last_strafe_control_time(this->get_clock()->now()),
      strafe_velocity_profile_initialized(false),
      strafe_deceleration_phase(false),
      // Helper Variables
      marker_detected(false),
      distance_to_marker(-1.0),
      curr_linear_vel(0.0),
      curr_angular_vel(0.0),
      last_follow_feedback_time(this->get_clock()->now()),
      last_search_feedback_time(this->get_clock()->now()),
      last_strafe_feedback_time(this->get_clock()->now())
    {
        RCLCPP_INFO(this->get_logger(), "RangerServer node has been started.");

        // Initialize ROS2 parameters
        initialize_parameters();

        // Create action servers
        this->follow_action_server = rclcpp_action::create_server<ArucoFollow>(
            this,
            "follow_aruco",
            std::bind(&RangerServer::handle_follow_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RangerServer::handle_follow_cancel, this, std::placeholders::_1),
            std::bind(&RangerServer::handle_follow_accepted, this, std::placeholders::_1));

        this->search_action_server = rclcpp_action::create_server<ArucoSearch>(
            this,
            "search_aruco",
            std::bind(&RangerServer::handle_search_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RangerServer::handle_search_cancel, this, std::placeholders::_1),
            std::bind(&RangerServer::handle_search_accepted, this, std::placeholders::_1));

        this->strafe_action_server = rclcpp_action::create_server<ArucoStrafe>(
            this,
            "strafe_aruco",
            std::bind(&RangerServer::handle_strafe_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RangerServer::handle_strafe_cancel, this, std::placeholders::_1),
            std::bind(&RangerServer::handle_strafe_accepted, this, std::placeholders::_1));

        // Initialize actions result and feedback
        follow_result   = std::make_shared<ArucoFollow::Result>();
        follow_feedback = std::make_shared<ArucoFollow::Feedback>();
        search_result   = std::make_shared<ArucoSearch::Result>();
        search_feedback = std::make_shared<ArucoSearch::Feedback>();
        strafe_result   = std::make_shared<ArucoStrafe::Result>();
        strafe_feedback = std::make_shared<ArucoStrafe::Feedback>();

        // Create subscribers and publishers using parameters
        std::string aruco_topic   = this->get_parameter("topics.aruco_markers").as_string();
        std::string cmd_vel_topic = this->get_parameter("topics.cmd_vel").as_string();
        std::string imu_topic     = this->get_parameter("topics.imu_data").as_string();
        
        aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            aruco_topic, 10,
            std::bind(&RangerServer::aruco_callback, this, std::placeholders::_1));

        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        
        imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&RangerServer::imu_callback, this, std::placeholders::_1));

        // Create debug publisher for rotation angle after parameters are loaded
        if (debug_enabled) {
            rotation_angle_debug_publisher = this->create_publisher<std_msgs::msg::Float64>(
                "/ranger_server/debug/rotation_angle", 10);
            RCLCPP_INFO(this->get_logger(), "Debug publisher for rotation angle created on topic: /ranger_server/debug/rotation_angle");
        }

        // Timer for control loop using parameter
        auto loop_period = std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate));
        control_timer = this->create_wall_timer(
            loop_period, std::bind(&RangerServer::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Action server '/follow_aruco' is ready.");
        RCLCPP_INFO(this->get_logger(), "Action server '/search_aruco' is ready.");
        RCLCPP_INFO(this->get_logger(), "Action server '/strafe_aruco' is ready.");
    }

private:
    // Actions server
    rclcpp_action::Server<ArucoFollow>::SharedPtr follow_action_server;
    rclcpp_action::Server<ArucoSearch>::SharedPtr search_action_server;
    rclcpp_action::Server<ArucoStrafe>::SharedPtr strafe_action_server;

    // Publishers and subscribers
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotation_angle_debug_publisher;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer;
    
    // Actions Result & Feedback
    std::shared_ptr<ArucoFollow::Result> follow_result;
    std::shared_ptr<ArucoFollow::Feedback> follow_feedback;
    std::shared_ptr<ArucoSearch::Result> search_result;
    std::shared_ptr<ArucoSearch::Feedback> search_feedback;
    std::shared_ptr<ArucoStrafe::Result> strafe_result;
    std::shared_ptr<ArucoStrafe::Feedback> strafe_feedback;

    // Current goal handle
    std::shared_ptr<GoalHandleArucoFollow> current_follow_goal_handle;
    std::shared_ptr<GoalHandleArucoSearch> current_search_goal_handle;
    std::shared_ptr<GoalHandleArucoStrafe> current_strafe_goal_handle;

    // Follow Action variables
    int64_t follow_marker_id;
    bool follow_active;
    float min_distance;
    float linear_vel;
    float angular_vel;

    // Search Action state variables
    int64_t search_marker_id;
    bool search_active;
    int64_t search_direction; 
    double search_angular_vel;
    rclcpp::Time search_start_time;
    bool alignment_marker_detected;
    geometry_msgs::msg::Pose alignment_marker_pose;

    // Search velocity profile variables
    double current_search_angular_velocity;
    rclcpp::Time last_search_control_time;
    bool search_velocity_profile_initialized;
    bool search_deceleration_phase;

    // IMU rotation tracking variables
    bool initial_quaternion_set;
    std::array<double, 4> initial_quaternion; // [x, y, z, w]
    double initial_yaw;
    double current_rotation_angle; // Current rotation angle in degrees
    double cumulative_rotation; // Cumulative rotation tracking (unbounded)

    // Strafe Action state variables
    int64_t strafe_marker_id;
    bool strafe_active;
    int64_t strafe_direction;
    rclcpp::Time strafe_start_time;
    bool strafe_marker_detected;
    geometry_msgs::msg::Pose strafe_marker_pose;

    // Strafe velocity profile variables
    double current_strafe_velocity;
    rclcpp::Time last_strafe_control_time;
    bool strafe_velocity_profile_initialized;
    bool strafe_deceleration_phase;

    // Marker state
    bool marker_detected;
    geometry_msgs::msg::Pose current_marker_pose;
    rclcpp::Time last_marker_time;

    // Feedback variables
    float distance_to_marker;
    float curr_linear_vel;
    float curr_angular_vel;
    
    // Maximum velocity limits (safety parameters)
    double max_linear_vel_follow;
    double max_angular_vel_follow;
    double max_angular_vel_search;
    
    // Parameter values
    double default_linear_vel;
    double default_angular_vel;
    double marker_timeout;
    double angular_gain;
    double max_acceleration;
    double max_deceleration;
    double min_linear_vel_threshold;
    double min_angular_vel_threshold;
    double deceleration_step_size;
    double deceleration_distance_threshold;
    double min_deceleration_velocity;
    double follow_alignment_tolerance;
    double search_timeout;
    double default_search_angular_vel;
    int default_search_direction;
    double search_alignment_tolerance;
    double search_accel;
    double search_decel;
    double search_min_angular_vel;
    double search_min_degree_for_decel;
    double strafe_vel;
    int default_strafe_direction;
    double strafe_max_vel;
    double strafe_timeout;
    double strafe_alignment_tolerance;
    double strafe_accel;
    double strafe_decel;
    double strafe_min_vel;
    double strafe_min_offset_for_decel;
    double loop_rate;
    double feedback_rate;
    bool debug_enabled;
    int throttle_duration;

    // Feedback throttling variables
    rclcpp::Time last_follow_feedback_time;
    rclcpp::Time last_search_feedback_time;
    rclcpp::Time last_strafe_feedback_time;

    // Trapezoidal Velocity Profile Variables
    bool velocity_profile_initialized;
    rclcpp::Time last_control_time;
    double current_target_velocity;

    double prev_linear_vel;
    double prev_angular_vel;
    // Parameter initialization method
    void initialize_parameters()
    {
        // Follow action parameters
        this->declare_parameter("follow_action.default_linear_vel", 0.5);
        this->declare_parameter("follow_action.default_angular_vel", 0.75);
        this->declare_parameter("follow_action.max_linear_vel", 1.5);
        this->declare_parameter("follow_action.max_angular_vel", 2.0);
        this->declare_parameter("follow_action.marker_timeout", 2.0);
        this->declare_parameter("follow_action.angular_gain", 2.0);
        this->declare_parameter("follow_action.max_acceleration", 0.2);
        this->declare_parameter("follow_action.max_deceleration", 0.2);
        this->declare_parameter("follow_action.min_linear_vel_threshold", 0.05);
        this->declare_parameter("follow_action.min_angular_vel_threshold", 0.05);
        this->declare_parameter("follow_action.deceleration_step_size", 0.05);
        this->declare_parameter("follow_action.deceleration_distance_threshold", 0.3);
        this->declare_parameter("follow_action.min_deceleration_velocity", 0.1);
        this->declare_parameter("follow_action.alignment_tolerance", 0.1); 
        // Search action parameters
        this->declare_parameter("search_action.default_angular_vel", 0.75);
        this->declare_parameter("search_action.default_direction", 1);
        this->declare_parameter("search_action.max_angular_vel", 2.0);
        this->declare_parameter("search_action.search_timeout", 60.0);
        this->declare_parameter("search_action.alignment_tolerance", 0.1); //radians
        this->declare_parameter("search_action.accel", 0.2);
        this->declare_parameter("search_action.decel", 0.2);
        this->declare_parameter("search_action.min_angular_vel", 0.05);
        this->declare_parameter("search_action.min_degree_for_decel", 30.0); //degrees
        // Strafe action parameters
        this->declare_parameter("strafe_action.default_vel", 0.5);
        this->declare_parameter("strafe_action.default_direction", 1);
        this->declare_parameter("strafe_action.max_vel", 1.0);
        this->declare_parameter("strafe_action.strafe_timeout", 30.0);
        this->declare_parameter("strafe_action.accel", 0.2);
        this->declare_parameter("strafe_action.decel", 0.2);
        this->declare_parameter("strafe_action.min_vel", 0.05);
        this->declare_parameter("strafe_action.alignment_tolerance", 0.05); //m
        this->declare_parameter("strafe_action.min_offset_for_decel", 0.2); //m
        // Topic parameters
        this->declare_parameter("topics.aruco_markers", "/aruco_markers");
        this->declare_parameter("topics.cmd_vel", "/cmd_vel");
        this->declare_parameter("topics.imu_data", "/oak/imu/data");
        
        // Control parameters
        this->declare_parameter("control.loop_rate", 10.0);
        this->declare_parameter("control.feedback_rate", 5.0);
        
        // Logging parameters
        this->declare_parameter("logging.debug_enabled", false);
        this->declare_parameter("logging.throttle_duration", 2000);
        
        // Load parameters
        load_parameters();
        
        RCLCPP_INFO(this->get_logger(), "Parameters initialized successfully");
        log_parameter_summary();
    }
    
    void load_parameters()
    {
        // Follow action parameters
        default_linear_vel      = this->get_parameter("follow_action.default_linear_vel").as_double();
        default_angular_vel     = this->get_parameter("follow_action.default_angular_vel").as_double();
        max_linear_vel_follow   = this->get_parameter("follow_action.max_linear_vel").as_double();
        max_angular_vel_follow  = this->get_parameter("follow_action.max_angular_vel").as_double();
        marker_timeout          = this->get_parameter("follow_action.marker_timeout").as_double();
        angular_gain            = this->get_parameter("follow_action.angular_gain").as_double();
        max_acceleration        = this->get_parameter("follow_action.max_acceleration").as_double();
        max_deceleration        = this->get_parameter("follow_action.max_deceleration").as_double();
        min_linear_vel_threshold  = this->get_parameter("follow_action.min_linear_vel_threshold").as_double();
        min_angular_vel_threshold = this->get_parameter("follow_action.min_angular_vel_threshold").as_double();
        deceleration_step_size    = this->get_parameter("follow_action.deceleration_step_size").as_double();
        deceleration_distance_threshold = this->get_parameter("follow_action.deceleration_distance_threshold").as_double();
        min_deceleration_velocity = this->get_parameter("follow_action.min_deceleration_velocity").as_double();
        follow_alignment_tolerance = this->get_parameter("follow_action.alignment_tolerance").as_double();
        // Search action parameters
        default_search_angular_vel = this->get_parameter("search_action.default_angular_vel").as_double();
        default_search_direction   = this->get_parameter("search_action.default_direction").as_int();
        max_angular_vel_search     = this->get_parameter("search_action.max_angular_vel").as_double();
        search_timeout             = this->get_parameter("search_action.search_timeout").as_double();
        search_alignment_tolerance = this->get_parameter("search_action.alignment_tolerance").as_double();
        search_accel               = this->get_parameter("search_action.accel").as_double();
        search_decel               = this->get_parameter("search_action.decel").as_double();
        search_min_angular_vel     = this->get_parameter("search_action.min_angular_vel").as_double();
        search_min_degree_for_decel = this->get_parameter("search_action.min_degree_for_decel").as_double();
        // Strafe action parameters
        strafe_vel                 = this->get_parameter("strafe_action.default_vel").as_double();
        default_strafe_direction   = this->get_parameter("strafe_action.default_direction").as_int();
        strafe_max_vel             = this->get_parameter("strafe_action.max_vel").as_double();
        strafe_timeout             = this->get_parameter("strafe_action.strafe_timeout").as_double();
        strafe_accel               = this->get_parameter("strafe_action.accel").as_double();
        strafe_decel               = this->get_parameter("strafe_action.decel").as_double();
        strafe_min_vel             = this->get_parameter("strafe_action.min_vel").as_double();
        strafe_alignment_tolerance  = this->get_parameter("strafe_action.alignment_tolerance").as_double();
        strafe_min_offset_for_decel = this->get_parameter("strafe_action.min_offset_for_decel").as_double();
        // Control Loop rate
        loop_rate         = this->get_parameter("control.loop_rate").as_double();
        feedback_rate     = this->get_parameter("control.feedback_rate").as_double();
        // Logging parameters
        debug_enabled     = this->get_parameter("logging.debug_enabled").as_bool();
        throttle_duration = this->get_parameter("logging.throttle_duration").as_int();
        
        // Initialize default values
        linear_vel   = default_linear_vel;
        angular_vel  = default_angular_vel;
    }
    
    void log_parameter_summary()
    {
        RCLCPP_INFO(this->get_logger(), "=== Ranger Server Parameter Summary ===");
        RCLCPP_INFO(this->get_logger(), " CONTROL LOOP RATE: %.2f Hz", loop_rate);
        RCLCPP_INFO(this->get_logger(), " FEEDBACK PUBLISH RATE: %.2f Hz", feedback_rate);
        RCLCPP_INFO(this->get_logger(), "Follow Action:");
        RCLCPP_INFO(this->get_logger(), "  Default linear vel: %.2f m/s (max: %.2f m/s)", 
                   default_linear_vel, max_linear_vel_follow);
        RCLCPP_INFO(this->get_logger(), "  Default angular vel: %.2f rad/s (max: %.2f rad/s)", 
                   default_angular_vel, max_angular_vel_follow);
        RCLCPP_INFO(this->get_logger(), "  Marker timeout: %.1f s", marker_timeout);
        RCLCPP_INFO(this->get_logger(), "  Angular gain: %.2f", angular_gain);
        RCLCPP_INFO(this->get_logger(), "  Max acceleration: %.2f m/s^2", max_acceleration);
        RCLCPP_INFO(this->get_logger(), "  Max deceleration: %.2f m/s^2", max_deceleration);
        RCLCPP_INFO(this->get_logger(), "  Min linear vel threshold: %.2f m/s", min_linear_vel_threshold);
        RCLCPP_INFO(this->get_logger(), "  Min angular vel threshold: %.2f rad/s", min_angular_vel_threshold);
        RCLCPP_INFO(this->get_logger(), "  Deceleration step size: %.2f m/s^2", deceleration_step_size);
        RCLCPP_INFO(this->get_logger(), "  Deceleration distance threshold: %.2f m", deceleration_distance_threshold);
        RCLCPP_INFO(this->get_logger(), "  Min deceleration velocity: %.2f m/s", min_deceleration_velocity);
        RCLCPP_INFO(this->get_logger(), "  Follow alignment tolerance: %.2f radians", follow_alignment_tolerance);
        RCLCPP_INFO(this->get_logger(), "Search Action:");
        RCLCPP_INFO(this->get_logger(), "  Default angular vel: %.2f rad/s (max: %.2f rad/s)", 
                   default_search_angular_vel, max_angular_vel_search);
        RCLCPP_INFO(this->get_logger(), "  Search timeout: %.1f s", search_timeout);
        RCLCPP_INFO(this->get_logger(), "  Alignment tolerance: %.2f radians", search_alignment_tolerance);
        RCLCPP_INFO(this->get_logger(), "Strafe Action:");
        RCLCPP_INFO(this->get_logger(), "  Default velocity: %.2f m/s (max: %.2f m/s)", 
                   strafe_vel, strafe_max_vel);
        RCLCPP_INFO(this->get_logger(), "  Acceleration: %.2f m/s^2", strafe_accel);
        RCLCPP_INFO(this->get_logger(), "  Deceleration: %.2f m/s^2", strafe_decel);
        RCLCPP_INFO(this->get_logger(), "  Min velocity: %.2f m/s", strafe_min_vel);
        RCLCPP_INFO(this->get_logger(), "  Min offset for deceleration: %.2f m", strafe_min_offset_for_decel);
        RCLCPP_INFO(this->get_logger(), "  Strafe timeout: %.1f s", strafe_timeout);
        RCLCPP_INFO(this->get_logger(), "  Alignment tolerance: %.2f m", strafe_alignment_tolerance);
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }
    
    // Safety function to enforce velocity limits
    double enforce_linear_velocity_limit(double requested_vel, const std::string& action_name)
    {
        if (requested_vel > max_linear_vel_follow) {
            RCLCPP_WARN(this->get_logger(), 
                       "[%s] Requested linear velocity %.2f m/s exceeds maximum limit %.2f m/s. Using maximum value.",
                       action_name.c_str(), requested_vel, max_linear_vel_follow);
            return max_linear_vel_follow;
        }
        return requested_vel;
    }
    
    double enforce_angular_velocity_limit(double requested_vel, const std::string& action_name, bool is_search = false)
    {
        double max_limit = is_search ? max_angular_vel_search : max_angular_vel_follow;
        
        if (std::abs(requested_vel) > max_limit) {
            RCLCPP_WARN(this->get_logger(), 
                       "[%s] Requested angular velocity %.2f rad/s exceeds maximum limit %.2f rad/s. Using maximum value.",
                       action_name.c_str(), std::abs(requested_vel), max_limit);
            return std::copysign(max_limit, requested_vel);
        }
        return requested_vel;
    }

    double enforce_lateral_velocity_limit(double requested_vel)
    {
        if (std::abs(requested_vel) > strafe_max_vel) {
            RCLCPP_WARN(this->get_logger(), 
                       "Requested lateral velocity %.2f m/s exceeds maximum limit %.2f m/s. Using maximum value.",
                       std::abs(requested_vel), strafe_max_vel);
            return std::copysign(strafe_max_vel, requested_vel);
        }
        return requested_vel;
    }

    //  FOLLOW ACTION HANDLERS
    rclcpp_action::GoalResponse handle_follow_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArucoFollow::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request to follow marker ID: %ld", goal->marker_id);
        
        // Check if another action is already running
        if (follow_active || search_active || strafe_active) {
            RCLCPP_WARN(this->get_logger(), "Another ranger action is already active. Rejecting new follow goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_follow_cancel(
        const std::shared_ptr<GoalHandleArucoFollow> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        
        // Stop the robot
        stop_robot();
        follow_active = false;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_follow_accepted(const std::shared_ptr<GoalHandleArucoFollow> goal_handle)
    {
        // Update parameters
        load_parameters();
        // Get goal request parameters
        auto goal = goal_handle->get_goal();
        // Mandatory request parameters
        follow_marker_id = goal->marker_id;
        min_distance     = goal->min_distance;
        
        // Optional parameters with defaults and safety limits
        double requested_linear_vel  = (goal->linear_vel > 0.0) ? goal->linear_vel : default_linear_vel;
        double requested_angular_vel = (goal->angular_vel > 0.0) ? goal->angular_vel : default_angular_vel;
        
        // Enforce velocity limits with safety warnings
        linear_vel  = enforce_linear_velocity_limit(requested_linear_vel, "follow_aruco");
        angular_vel = enforce_angular_velocity_limit(requested_angular_vel, "follow_aruco", false);
        
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
            current_follow_goal_handle = goal_handle;
            if(check_aruco_topic_availability()){
                follow_active   = true;
                marker_detected = false;
                // Initialize velocity profile parameters
                velocity_profile_initialized = false;
                last_control_time            = this->get_clock()->now();
                current_target_velocity      = 0.0;
                RCLCPP_INFO(this->get_logger(), 
                    "Starting to follow marker ID: %ld | min_distance: %.2f | linear_vel: %.2f | angular_vel: %.2f",
                    follow_marker_id, min_distance, linear_vel, angular_vel);
            } else {
                follow_active = false;
                follow_result->success = false;
                follow_result->message = "ERR: No publisher for /aruco_markers topic. Please check the aruco recognition node first.";
                goal_handle->abort(follow_result);
                return;
            }
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

    // SEARCH ACTION HANDLERS
    rclcpp_action::GoalResponse handle_search_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArucoSearch::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request to search marker ID: %ld", goal->marker_id);
        
        // Check if another action is already running
        if (follow_active || search_active || strafe_active) {
            RCLCPP_WARN(this->get_logger(), "Another rangeraction is already active. Rejecting new search goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_search_cancel(
        const std::shared_ptr<GoalHandleArucoSearch> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel search goal");
        (void)goal_handle;

        // Stop the robot
        stop_robot();
        search_active             = false;
        alignment_marker_detected = false;

        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_search_accepted(const std::shared_ptr<GoalHandleArucoSearch> goal_handle)
    {
        // Update parameters
        load_parameters();
        // Get goal request parameters
        auto goal = goal_handle->get_goal();
        search_marker_id = goal->marker_id;
        
        // Optional parameters with defaults and safety limits
        double requested_angular_vel = (goal->angular_vel > 0.0) ? goal->angular_vel : default_search_angular_vel;
        search_angular_vel = enforce_angular_velocity_limit(requested_angular_vel, "search_aruco", true);
        
        search_direction = (goal->direction != 0) ? goal->direction : default_search_direction;

        // Check validity of goal parameters
        bool valid = true;
        std::string missing_fields;

        // Check if marker_id is specified
        if(search_marker_id <= 0) {
            valid = false;
            missing_fields += "marker_id ";
        }

        // Validate search direction
        if (search_direction != 1 && search_direction != -1) {
            search_direction = default_search_direction;
            RCLCPP_WARN(this->get_logger(), "Invalid direction specified, must use [-1 || 1] ,using default value: %s", default_search_direction ? "counter-clockwise" : "clockwise");
        }

        if(valid) {
            current_search_goal_handle = goal_handle;
            if(check_aruco_topic_availability()){
                search_active             = true;
                marker_detected           = false;
                alignment_marker_detected = false;
                search_start_time  = this->get_clock()->now();
                
                // Reset search velocity profile
                search_velocity_profile_initialized = false;
                current_search_angular_velocity = 0.0;
                search_deceleration_phase = false;
                
                // Reset IMU tracking for this search action
                initial_quaternion_set = false;
                current_rotation_angle = 0.0;
                cumulative_rotation = 0.0;
                
                RCLCPP_INFO(this->get_logger(), 
                    "Starting to search for marker ID: %ld | angular_vel: %.2f | direction: %s",
                    search_marker_id, search_angular_vel, (search_direction == 1) ? "counter-clockwise" : "clockwise");
            } else {
                search_active = false;
                search_result->success = false;
                search_result->message = "ERR: No publisher for /aruco_markers topic. Please check the aruco recognition node first.";
                goal_handle->abort(search_result);
                return;
            }
            
        } else {
            search_active = false;
            RCLCPP_WARN(this->get_logger(),
                "Action request missing required field(s) or incorrect values: %s. Please specify them in the action request.",
                missing_fields.c_str());
            search_result->success = false;
            search_result->message = "Action request failed due to missing or invalid parameters: " + missing_fields;
            goal_handle->abort(search_result);
            return;
        }
    }

    // STRAFE ACTION HANDLERS
    rclcpp_action::GoalResponse handle_strafe_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArucoStrafe::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request to strafe marker ID: %ld", goal->marker_id);
    
    // Check if another action is already running
        if (follow_active || search_active || strafe_active) {
            RCLCPP_WARN(this->get_logger(), "Another ranger action is already active. Rejecting new strafe goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_strafe_cancel(
        const std::shared_ptr<GoalHandleArucoStrafe> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel strafe goal");
        (void)goal_handle;

        // Stop the robot
        stop_robot();
        strafe_active = false;
        
        // Reset velocity profile
        current_strafe_velocity = 0.0;
        strafe_velocity_profile_initialized = false;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_strafe_accepted(const std::shared_ptr<GoalHandleArucoStrafe> goal_handle)
    {
        // Update parameters
        load_parameters();
        // Get goal request parameters
        auto goal = goal_handle->get_goal();
        // Mandatory request parameters
        strafe_marker_id = goal->marker_id;
        
        // Optional parameters with defaults and safety limits
        double requested_lateral_vel = (goal->lateral_vel > 0.0) ? goal->lateral_vel : strafe_vel;
        strafe_vel = enforce_lateral_velocity_limit(requested_lateral_vel);

        strafe_direction   = (goal->direction != 0) ? goal->direction : default_strafe_direction;

        // Check validity of goal parameters
        bool valid = true;
        std::string missing_fields;

        // Check if marker_id is specified
        if(strafe_marker_id <= 0) {
            valid = false;
            missing_fields += "marker_id ";
        }
        // Validate strafe direction
        if (strafe_direction != 1 && strafe_direction != -1) {
            strafe_direction = default_strafe_direction;
            RCLCPP_WARN(this->get_logger(), "Invalid direction specified, must use [-1 || 1] ,using default value: %s", default_strafe_direction ? "left" : "right");
        }

        if(valid) {
            current_strafe_goal_handle = goal_handle;
            if(check_aruco_topic_availability()){
                strafe_active = true;
                marker_detected = false;
                strafe_marker_detected = false;
                strafe_start_time = this->get_clock()->now();
                
                // Initialize velocity profile
                current_strafe_velocity = 0.0;
                last_strafe_control_time = this->get_clock()->now();
                strafe_velocity_profile_initialized = true;
                strafe_deceleration_phase = false;
                
                RCLCPP_INFO(this->get_logger(), "Strafe action started for marker ID: %ld | lateral_vel: %.2f | direction: %s",
                    strafe_marker_id, strafe_vel, (strafe_direction == 1) ? "left" : "right");
            } else {
                strafe_active = false;
                strafe_result->success = false;
                strafe_result->message = "ERR: No publisher for /aruco_markers topic. Please check the aruco recognition node first.";
                goal_handle->abort(strafe_result);
                return;
            }
        } else {
        
            strafe_active = false;
            RCLCPP_WARN(this->get_logger(),
                "Action request missing required field(s) or incorrect values: %s. Please specify them in the action request.",
                missing_fields.c_str());
            strafe_result->success = false;
            strafe_result->message = "Action request failed due to missing or invalid parameters: " + missing_fields;
            goal_handle->abort(strafe_result);
            return;
        }
    }
    

    bool check_aruco_topic_availability()
    {
        // Get publisher count for the topic
        auto publishers_info = this->get_publishers_info_by_topic("/aruco_markers");
        
        if (publishers_info.empty()) {
            RCLCPP_WARN(this->get_logger(), 
                "Warning: No publishers found for /aruco_markers topic. Make sure the Aruco recognition node is running.");
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "Found %zu publisher(s) for /aruco_markers topic.", publishers_info.size());
            
            // Optionally, you can check the publisher node names
            for (const auto& pub_info : publishers_info) {
                RCLCPP_INFO(this->get_logger(), 
                    "Publisher node: %s", pub_info.node_name().c_str());
            }
            return true;
        }
    }

    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (!follow_active && !search_active && !strafe_active) return;
        
        // Look for the target marker in the message
        for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
            // Check for follow action
            if (follow_active && msg->marker_ids[i] == follow_marker_id) {
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
            
            // Check for search action
            if (search_active && msg->marker_ids[i] == search_marker_id) {
                // Marker found - update detection status and pose for alignment check
                marker_detected = true;
                alignment_marker_detected = true;
                alignment_marker_pose = msg->poses[i];
                last_marker_time = this->get_clock()->now();
                
                RCLCPP_INFO(this->get_logger(), 
                    "Found target marker %ld during search at position (%.2f, %.2f, %.2f)", 
                    search_marker_id,
                    alignment_marker_pose.position.x,
                    alignment_marker_pose.position.y,
                    alignment_marker_pose.position.z);
                return;
            }
            // Check for strafe action
            if (strafe_active && msg->marker_ids[i] == strafe_marker_id) {
                // Update marker pose and detection status
                marker_detected = true;
                strafe_marker_detected = true;
                strafe_marker_pose = msg->poses[i];
                last_marker_time = this->get_clock()->now();
                
                // Calculate lateral offset for debugging
                float lateral_offset = std::abs(strafe_marker_pose.position.y);
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Marker %ld detected at lateral offset %.3fm. Velocity profile will handle movement.", 
                    strafe_marker_id, lateral_offset);
                    
                return;
            }
        }
    }

    // IMU callback for rotation tracking
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract quaternion
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        
        std::array<double, 4> current_quaternion = {qx, qy, qz, qw};
        
        // Convert to Euler angles to get current yaw
        double roll, pitch, yaw;
        quaternion_to_euler(qx, qy, qz, qw, roll, pitch, yaw);
        
        // Store initial orientation on first message
        if (!initial_quaternion_set) {
            initial_quaternion = current_quaternion;
            initial_yaw = yaw;
            initial_quaternion_set = true;
            current_rotation_angle = 0.0;
            cumulative_rotation = 0.0;
            RCLCPP_INFO(this->get_logger(), "Initial IMU orientation set - Yaw: %.2f°", 
                        yaw * 180.0 / M_PI);
            return;
        }
        
        // Calculate relative rotation using quaternion method
        std::array<double, 4> initial_conj = quaternion_conjugate(initial_quaternion);
        std::array<double, 4> relative_q = quaternion_multiply(current_quaternion, initial_conj);
        
        // Convert relative quaternion to Euler to get relative yaw
        double rel_roll, rel_pitch, rel_yaw;
        quaternion_to_euler(relative_q[0], relative_q[1], relative_q[2], relative_q[3], 
                           rel_roll, rel_pitch, rel_yaw);
        
        // Normalize relative yaw and convert to degrees
        rel_yaw = normalize_angle(rel_yaw);
        current_rotation_angle = rel_yaw * 180.0 / M_PI;
        
        // Publish debug information if enabled and publisher exists
        if (debug_enabled && rotation_angle_debug_publisher) {
            auto debug_msg = std_msgs::msg::Float64();
            debug_msg.data = current_rotation_angle;
            rotation_angle_debug_publisher->publish(debug_msg);
        }
    }

    // Utility methods for quaternion and angle operations
    void quaternion_to_euler(double x, double y, double z, double w, 
                            double& roll, double& pitch, double& yaw)
    {
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    // Quaternion multiplication
    std::array<double, 4> quaternion_multiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2)
    {
        double x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
        double x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];
        
        double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        double y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
        double z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
        
        return {x, y, z, w};
    }

    // Quaternion conjugate
    std::array<double, 4> quaternion_conjugate(const std::array<double, 4>& q)
    {
        return {-q[0], -q[1], -q[2], q[3]};
    }

    void control_loop()
    {
        if (follow_active && current_follow_goal_handle) {
            control_follow_action();
        } else if (search_active && current_search_goal_handle) {
            control_search_action();
        } else if (strafe_active && current_strafe_goal_handle) {
            control_strafe_action();
        }
    }

    void control_follow_action()
    {
        if (!follow_active || !current_follow_goal_handle) return;
        // Reset Velocity Profile
        if (!velocity_profile_initialized) {
            velocity_profile_initialized = true;
            current_target_velocity     = 0.0;
            prev_linear_vel             = 0.0;
            prev_angular_vel            = 0.0;
            last_control_time           = this->get_clock()->now();
        }
        // Check if marker was detected recently using parameter-based timeout
        if (!marker_detected || 
            (this->get_clock()->now() - last_marker_time).seconds() > marker_timeout) {
            
            if (marker_detected) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_duration,
                    "Marker %ld not detected recently.", follow_marker_id);
            }
            
            return;
        }
        
        // Calculate distance to marker 
        distance_to_marker = calculate_distance_to_marker();
        
        // Send follow_feedback (throttled)
        if (should_publish_follow_feedback()) {
            follow_feedback->distance_to_marker = std::round(distance_to_marker * 1000.0f) / 1000.0f;
            follow_feedback->curr_linear_vel    = std::round(curr_linear_vel * 1000.0f) / 1000.0f;
            follow_feedback->curr_angular_vel   = std::round(curr_angular_vel * 1000.0f) / 1000.0f;
            current_follow_goal_handle->publish_feedback(follow_feedback);
        }
        
        // Check if we've reached the minimum distance
        if (distance_to_marker <= min_distance) {
            RCLCPP_INFO(this->get_logger(), 
                "Reached minimum distance (%.2f m) to marker %ld", min_distance, follow_marker_id);
            
            stop_robot();
            
            // Send success follow_result
            follow_result->message = "Successfully reached marker " + std::to_string(follow_marker_id);
            follow_result->success = true;
            
            current_follow_goal_handle->succeed(follow_result);
            follow_active = false;
            return;
        }
        
        // Calculate and publish velocity commands
        move_towards_marker();
    }

    void control_search_action()
    { 
        // Calculate elapsed time since search started
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - search_start_time).seconds();
        
        // Send search feedback with elapsed time (throttled)
        if (should_publish_search_feedback()) {
            search_feedback->elapsed_time = elapsed_time;  // This field now represents elapsed time in seconds
            search_feedback->curr_angular_vel = std::round(current_search_angular_velocity * search_direction * 1000.0f) / 1000.0f;
            current_search_goal_handle->publish_feedback(search_feedback);
        }
        
        // Check if we've exceeded the timeout using parameter value
        if (elapsed_time >= search_timeout) {
            RCLCPP_INFO(this->get_logger(), 
                "Search timeout (%.1f seconds) reached without finding marker %ld", 
                search_timeout, search_marker_id);
            
            stop_robot();
            
            search_result->success = false;
            search_result->message = "Marker " + std::to_string(search_marker_id) + " not found within timeout period";
            
            current_search_goal_handle->succeed(search_result);
            search_active = false;
            return;
        }
        
        // Apply trapezoidal velocity profile for rotation (includes alignment checking)
        apply_search_velocity_profile();
    }

    void control_strafe_action()
    {
        if (!strafe_active || !current_strafe_goal_handle) return;
        
        // Calculate elapsed time since strafe started
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - strafe_start_time).seconds();
        
        // Check if we've exceeded the timeout using parameter value
        if (elapsed_time >= strafe_timeout) {
            RCLCPP_WARN(this->get_logger(), 
                "Strafe action timed out after %.1f seconds. Marker %ld not found.", 
                strafe_timeout, strafe_marker_id);
            
            stop_robot();
            
            strafe_result->success = false;
            strafe_result->message = "Marker " + std::to_string(strafe_marker_id) + " not found within timeout period";
            
            current_strafe_goal_handle->succeed(strafe_result);
            strafe_active = false;
            return;
        }
        
        // Apply trapezoidal velocity profile
        apply_strafe_velocity_profile();
        
        // Check if we've reached alignment tolerance
        if (strafe_marker_detected) {
            float lateral_offset = std::abs(strafe_marker_pose.position.y);
            if (lateral_offset <= strafe_alignment_tolerance) {
                RCLCPP_INFO(this->get_logger(), 
                    "Aligned with marker %ld within tolerance (%.3fm). Action completed.", 
                    strafe_marker_id, strafe_alignment_tolerance);
                
                stop_robot();
                
                // Send success result
                strafe_result->success = true;
                strafe_result->message = "Successfully aligned with marker " + std::to_string(strafe_marker_id);
                current_strafe_goal_handle->succeed(strafe_result);
                strafe_active = false;
                return;
            }
        }
        
        // Send strafe feedback with elapsed time (throttled)
        if (should_publish_strafe_feedback()) {
            strafe_feedback->current_lateral_vel = std::round(current_strafe_velocity * strafe_direction * 1000.0f) / 1000.0f;
            strafe_feedback->elapsed_time = elapsed_time;
            current_strafe_goal_handle->publish_feedback(strafe_feedback);
        }
    }

    void apply_strafe_velocity_profile()
    {
        auto current_time = this->get_clock()->now();
        
        // Initialize time tracking if this is the first call
        if (!strafe_velocity_profile_initialized) {
            last_strafe_control_time = current_time;
            current_strafe_velocity = 0.0;
            strafe_deceleration_phase = false;
            strafe_velocity_profile_initialized = true;
        }
        
        // Calculate time delta
        double dt = (current_time - last_strafe_control_time).seconds();
        last_strafe_control_time = current_time;
        
        // Skip if dt is too large (likely first iteration or system pause)
        if (dt > 0.1) {
            dt = 0.01; // Use small default dt
        }
        
        // Determine target velocity based on trapezoidal profile
        double target_velocity = 0.0;
        
        // Check if marker is detected and get lateral offset
        bool marker_found = strafe_marker_detected;
        double lateral_offset = 0.0;
        if (marker_found) {
            lateral_offset = std::abs(strafe_marker_pose.position.y);
        }
        
        if (!marker_found) {
            // Phase 1 & 2: No marker detected - accelerate to or maintain default strafe velocity
            target_velocity = strafe_vel;
            strafe_deceleration_phase = false;
        } else {
            // Marker detected - check which phase we're in
            if (lateral_offset <= strafe_min_offset_for_decel) {
                // Phase 3: Deceleration phase - linearly reduce velocity based on remaining distance
                strafe_deceleration_phase = true;
                
                // Calculate remaining distance to tolerance
                double remaining_distance = lateral_offset - strafe_alignment_tolerance;
                double decel_range = strafe_min_offset_for_decel - strafe_alignment_tolerance;
                
                if (remaining_distance <= 0) {
                    // Within tolerance - stop
                    target_velocity = 0.0;
                } else if (decel_range > 0.0001) {
                    // Linear interpolation from min_alignment_vel to strafe_vel
                    double velocity_range = strafe_vel - strafe_min_vel;
                    target_velocity = strafe_min_vel + 
                                    (velocity_range * remaining_distance / decel_range);
                    
                    // Ensure target velocity is within bounds
                    target_velocity = std::max(target_velocity, strafe_min_vel);
                    target_velocity = std::min(target_velocity, strafe_vel);
                } else {
                    target_velocity = strafe_min_vel;
                }
            } else {
                // Phase 1 & 2: Marker found but not yet in deceleration zone
                target_velocity = strafe_vel;
                strafe_deceleration_phase = false;
            }
        }
        
        // Apply acceleration/deceleration towards target velocity
        double velocity_error = target_velocity - current_strafe_velocity;
        
        if (std::abs(velocity_error) > 0.001) { // Small threshold to avoid oscillation
            if (velocity_error > 0) {
                // Accelerate
                current_strafe_velocity += strafe_accel * dt;
                current_strafe_velocity = std::min(current_strafe_velocity, target_velocity);
            } else {
                // Decelerate
                current_strafe_velocity -= strafe_decel * dt;
                current_strafe_velocity = std::max(current_strafe_velocity, target_velocity);
            }
        }
        
        // Ensure velocity doesn't exceed limits
        current_strafe_velocity = std::min(current_strafe_velocity, strafe_max_vel);
        current_strafe_velocity = std::max(current_strafe_velocity, 0.0);
        
        // Apply movement
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.y = current_strafe_velocity * strafe_direction;
        
        // No other movement during strafe
        twist_msg.linear.x = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        
        cmd_vel_publisher->publish(twist_msg);
        
        if (debug_enabled) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Trapezoidal velocity profile: current_vel=%.3f, target_vel=%.3f, lateral_offset=%.3f, decel_phase=%s, direction=%s", 
                current_strafe_velocity, target_velocity, lateral_offset, 
                strafe_deceleration_phase ? "true" : "false", 
                (strafe_direction == 1) ? "left" : "right");
        }
    }

    void apply_search_velocity_profile()
    {
        auto current_time = this->get_clock()->now();
        
        // Initialize time tracking if this is the first call
        if (!search_velocity_profile_initialized) {
            last_search_control_time = current_time;
            current_search_angular_velocity = 0.0;
            search_deceleration_phase = false;
            search_velocity_profile_initialized = true;
        }
        
        // Calculate time delta
        double dt = (current_time - last_search_control_time).seconds();
        last_search_control_time = current_time;
        
        // Skip if dt is too large (likely first iteration or system pause)
        if (dt > 0.1) {
            dt = 0.01; // Use small default dt
        }
        
        // Check if marker is detected and we're aligned within tolerance
        if (marker_detected && alignment_marker_detected) {
            // Calculate angle to marker for alignment check
            float angle_to_marker = std::atan2(
                alignment_marker_pose.position.y, 
                alignment_marker_pose.position.x
            );
            
            // Check if we are aligned within tolerance
            if (std::abs(angle_to_marker) <= search_alignment_tolerance) {
                RCLCPP_INFO(this->get_logger(), 
                    "Aligned with marker %ld within tolerance (%.3f radians). Stopping search action.", 
                    search_marker_id, search_alignment_tolerance);
                
                stop_robot();
                
                // Send success result
                search_result->success = true;
                search_result->message = "Successfully aligned with marker " + std::to_string(search_marker_id);
                current_search_goal_handle->succeed(search_result);
                search_active = false;
                return;
            }
        }
        
        // Determine target velocity based on trapezoidal profile
        double target_velocity = 0.0;
        
        // Get current rotation angle (absolute value for comparison)
        double abs_rotation_angle = std::abs(current_rotation_angle);
        
        if (abs_rotation_angle < search_min_degree_for_decel) {
            // Phase 1 & 2: Acceleration/Constant velocity phase
            // Accelerate to or maintain default angular velocity
            target_velocity = search_angular_vel;
            search_deceleration_phase = false;
        } else {
            // Phase 3: Deceleration phase
            // Once we reach the deceleration threshold, start decelerating to minimum velocity
            search_deceleration_phase = true;
            target_velocity = search_min_angular_vel;
        }
        
        // Apply acceleration/deceleration towards target velocity
        double velocity_error = target_velocity - current_search_angular_velocity;
        
        if (std::abs(velocity_error) > 0.001) { // Small threshold to avoid oscillation
            if (velocity_error > 0) {
                // Accelerate
                current_search_angular_velocity += search_accel * dt;
                current_search_angular_velocity = std::min(current_search_angular_velocity, target_velocity);
            } else {
                // Decelerate
                current_search_angular_velocity -= search_decel * dt;
                current_search_angular_velocity = std::max(current_search_angular_velocity, target_velocity);
            }
        }
        
        // Ensure velocity doesn't exceed limits
        current_search_angular_velocity = std::min(current_search_angular_velocity, search_angular_vel);
        current_search_angular_velocity = std::max(current_search_angular_velocity, 0.0);
        
        // Apply rotation
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = current_search_angular_velocity * search_direction;
        
        cmd_vel_publisher->publish(twist_msg);
        
        if (debug_enabled) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Search trapezoidal profile: current_vel=%.3f, target_vel=%.3f, rotation=%.1f°, decel_threshold=%.1f°, phase=%s, direction=%s", 
                current_search_angular_velocity, target_velocity, current_rotation_angle, search_min_degree_for_decel,
                search_deceleration_phase ? "decel" : "accel/const", 
                (search_direction == 1) ? "ccw" : "cw");
        }
    }

    // Feedback throttling helper methods
    bool should_publish_follow_feedback()
    {
        auto current_time = this->get_clock()->now();
        double feedback_period = 1.0 / feedback_rate;  // Convert Hz to seconds
        
        if ((current_time - last_follow_feedback_time).seconds() >= feedback_period) {
            last_follow_feedback_time = current_time;
            return true;
        }
        return false;
    }
    
    bool should_publish_search_feedback()
    {
        auto current_time = this->get_clock()->now();
        double feedback_period = 1.0 / feedback_rate;  // Convert Hz to seconds
        
        if ((current_time - last_search_feedback_time).seconds() >= feedback_period) {
            last_search_feedback_time = current_time;
            return true;
        }
        return false;
    }

    bool should_publish_strafe_feedback()
    {
        auto current_time = this->get_clock()->now();
        double feedback_period = 1.0 / feedback_rate;  // Convert Hz to seconds
        
        if ((current_time - last_strafe_feedback_time).seconds() >= feedback_period) {
            last_strafe_feedback_time = current_time;
            return true;
        }
        return false;
    }

    // Helper functions
    float calculate_distance_to_marker()
    {
        return std::sqrt(
            std::pow(current_marker_pose.position.x, 2) +
            std::pow(current_marker_pose.position.y, 2)
        );
    }

    double round_to_precision(double value, int decimal_places = 3)
    {
        double multiplier = std::pow(10.0, decimal_places);
        return std::round(value * multiplier) / multiplier;
    }
    
    double filter_angular_velocity(double angular_vel)
    {
        // Filter out very small angular velocities
        if (std::abs(angular_vel) < min_angular_vel_threshold) {
            return min_angular_vel_threshold;
        }
        return round_to_precision(angular_vel, 3);
    }
    
    double filter_linear_velocity(double linear_vel)
    {
        // Filter out very small linear velocities
        if (std::abs(linear_vel) < min_linear_vel_threshold) {
            return min_linear_vel_threshold;
        }
        return round_to_precision(linear_vel, 3);
    }

    double calculate_target_velocity()
    {
        auto current_time = this->get_clock()->now();
        double dt = velocity_profile_initialized ? (current_time - last_control_time).seconds() : 0.1;
        last_control_time = current_time;

        if (!velocity_profile_initialized) {
            current_target_velocity = 0.0;
            velocity_profile_initialized = true;
            return current_target_velocity;
        }

        dt = std::max(0.01, std::min(dt, 0.2));
        double remaining_distance = distance_to_marker - min_distance;
        
        double desired_velocity = 0.0;
        
        if (remaining_distance <= deceleration_distance_threshold) {
            // STEPPED DECELERATION PHASE with minimum velocity limit
            // Calculate number of steps based on distance
            int steps_from_max = static_cast<int>((linear_vel - min_deceleration_velocity) / deceleration_step_size);
            double distance_per_step = deceleration_distance_threshold / steps_from_max;
            int current_step = static_cast<int>(remaining_distance / distance_per_step);
            
            // Calculate stepped velocity but ensure it doesn't go below minimum
            double stepped_velocity = min_deceleration_velocity + (current_step * deceleration_step_size);
            desired_velocity = std::max(min_deceleration_velocity, 
                                      std::min(stepped_velocity, static_cast<double>(linear_vel)));
        } else {
            // ACCELERATION/CRUISE PHASE
            desired_velocity = linear_vel;
        }

        // Smooth velocity changes
        double velocity_error = desired_velocity - current_target_velocity;
        double max_change = (velocity_error > 0) ? max_acceleration * dt : max_deceleration * dt;
        
        if (std::abs(velocity_error) > max_change) {
            current_target_velocity += std::copysign(max_change, velocity_error);
        } else {
            current_target_velocity = desired_velocity;
        }

        // Ensure bounds - maintain minimum velocity during motion unless stopping
        if (remaining_distance > 0.02) {
            current_target_velocity = std::max(min_deceleration_velocity, 
                                              std::min(current_target_velocity, static_cast<double>(linear_vel)));
        } else {
            current_target_velocity = std::max(0.0, std::min(current_target_velocity, static_cast<double>(linear_vel)));
        }
        
        return filter_linear_velocity(current_target_velocity);
    }

    void move_towards_marker()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Calculate angle to marker
        float angle_to_marker = std::atan2(
            current_marker_pose.position.y,
            current_marker_pose.position.x
        );
        
        // Angular correction withen alignment tolerance [dead-zone](proportional control)
        double raw_angular_vel = 0.0;
        if( std::abs(angle_to_marker) > follow_alignment_tolerance) {
            raw_angular_vel = std::max(-static_cast<double>(angular_vel),
                                         std::min(static_cast<double>(angular_vel), static_cast<double>(angle_to_marker * angular_gain)));
        }
        
        // Apply angular velocity filtering
        double filtered_angular = filter_angular_velocity(raw_angular_vel);
        
        // Improved alignment factor
        float alignment_factor = std::max(0.5f, 1.0f - std::abs(angle_to_marker) / static_cast<float>(M_PI));

        // Apply Trapezoidal velocity profile with filtering
        double target_velocity = calculate_target_velocity();
        double raw_linear_vel = target_velocity * alignment_factor;
        double filtered_linear = filter_linear_velocity(raw_linear_vel);
        
        // Continuity check - avoid sudden drops to zero
        if (prev_linear_vel > 0.05 && filtered_linear == 0.0 && target_velocity > min_deceleration_velocity) {
            filtered_linear = std::max(0.02, prev_linear_vel * 0.5); // Gradual reduction instead of sudden stop
        }
        
        // Modified continuity check for angular velocity - only apply if outside tolerance
        if (std::abs(angle_to_marker) > follow_alignment_tolerance && 
            prev_angular_vel > 0.02 && filtered_angular == 0.0 && std::abs(raw_angular_vel) > 0.01) {
            filtered_angular = std::max(0.01, prev_angular_vel * 0.5); // Gradual reduction instead of sudden stop
        }
        
        twist_msg.angular.z = filtered_angular;
        twist_msg.linear.x = filtered_linear;
        
        // Update previous velocities
        prev_linear_vel = filtered_linear;
        prev_angular_vel = filtered_angular;
        
        // Update current velocities with filtered values
        curr_linear_vel  = twist_msg.linear.x;
        curr_angular_vel = twist_msg.angular.z;
        
        cmd_vel_publisher->publish(twist_msg);
        
        if (debug_enabled) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Moving: linear=%.3f, angular=%.3f, distance=%.3f, angle=%.3f (tol=%.3f), alignment=%.3f",
                twist_msg.linear.x, twist_msg.angular.z, distance_to_marker, 
                angle_to_marker, follow_alignment_tolerance, alignment_factor);
        }
    }

    void rotate_robot()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Set angular velocity based on direction using search_angular_vel
        twist_msg.angular.z = search_angular_vel * search_direction;
        
        // No linear movement during search
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        
        cmd_vel_publisher->publish(twist_msg);
        
        if (debug_enabled) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Searching: angular=%.2f, elapsed_time=%.1f seconds",
                twist_msg.angular.z, (this->get_clock()->now() - search_start_time).seconds());
        }
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