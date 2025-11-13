// ranger_vr_teleop.cpp
// VR Joystick Teleoperation Node for Ranger Mini Mobile Base
// Subscribes to VR controller topics and publishes velocity commands

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unity_ros_interfaces/msg/controller.hpp>
#include <memory>

class RangerVRTeleop : public rclcpp::Node
{
public:
  RangerVRTeleop()
    : Node("ranger_vr_teleop"),
      right_teleop_enabled_(false),
      left_teleop_enabled_(false),
      right_button_prev_state_(false),
      left_button_prev_state_(false)
  {
    // Declare and get parameters
    this->declare_parameter("max_linear_vel", 0.7);
    this->declare_parameter("max_angular_vel", 0.5);
    this->declare_parameter("max_strafe_vel", 0.5);
    this->declare_parameter("dead_band", 0.05);
    this->declare_parameter("publish_rate", 50.0);  // Hz
    
    max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    max_strafe_vel_  = this->get_parameter("max_strafe_vel").as_double();
    dead_band_       = this->get_parameter("dead_band").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Starting Ranger VR Teleop Node");
    RCLCPP_INFO(this->get_logger(), "Max Linear Velocity: %.2f m/s", max_linear_vel_);
    RCLCPP_INFO(this->get_logger(), "Max Angular Velocity: %.2f rad/s", max_angular_vel_);
    RCLCPP_INFO(this->get_logger(), "Max Strafe Velocity: %.2f m/s", max_strafe_vel_);
    RCLCPP_INFO(this->get_logger(), "Dead Band: %.2f", dead_band_);
    RCLCPP_INFO(this->get_logger(), "Press SECONDARY BUTTON on RIGHT controller (B) to enable/disable base teleoperation");
    RCLCPP_INFO(this->get_logger(), "Press SECONDARY BUTTON on LEFT controller (Y) to enable/disable special motions");
    
    // Initialize velocity commands to zero
    current_linear_vel_ = 0.0;
    current_angular_vel_ = 0.0;
    current_strafe_vel_ = 0.0;
    
    // Create subscribers for VR controllers
    right_controller_sub_ = this->create_subscription<unity_ros_interfaces::msg::Controller>(
      "/vr/right_controller_ros",
      10,
      std::bind(&RangerVRTeleop::rightControllerCallback, this, std::placeholders::_1));
    
    left_controller_sub_ = this->create_subscription<unity_ros_interfaces::msg::Controller>(
      "/vr/left_controller_ros",
      10,
      std::bind(&RangerVRTeleop::leftControllerCallback, this, std::placeholders::_1));
    
    // Create publisher for cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create timer for publishing cmd_vel at fixed rate
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&RangerVRTeleop::publishCmdVel, this));
    
    RCLCPP_INFO(this->get_logger(), "Ranger VR Teleop initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Use right controller joystick to control the base");
  }

private:
  // Apply dead band to joystick value
  double applyDeadBand(double value)
  {
    if (std::abs(value) < dead_band_) {
      return 0.0;
    }
    
    // Scale the value to maintain smoothness after dead band
    // Map from [dead_band, 1.0] to [0.0, 1.0]
    double sign = (value > 0) ? 1.0 : -1.0;
    double abs_value = std::abs(value);
    double scaled_value = (abs_value - dead_band_) / (1.0 - dead_band_);
    
    return sign * scaled_value;
  }
  
  // Right controller callback - used for base movement
  void rightControllerCallback(const unity_ros_interfaces::msg::Controller::SharedPtr msg)
  {
    // Detect secondary button press (rising edge) for toggle
    if (msg->secondary_button_pressed && !right_button_prev_state_) {
      // Button just pressed - toggle teleop state
      right_teleop_enabled_ = !right_teleop_enabled_;
      
      if (right_teleop_enabled_) {
        RCLCPP_INFO(this->get_logger(), "RIGHT Controller Teleoperation ENABLED");
      } else {
        RCLCPP_INFO(this->get_logger(), "RIGHT Controller Teleoperation DISABLED");
        // Stop the robot when disabling
        current_linear_vel_  = 0.0;
        current_angular_vel_ = 0.0;
      }
    }
    
    // Update previous button state
    right_button_prev_state_ = msg->secondary_button_pressed;
    
    // Only process joystick input if teleoperation is enabled
    if (!right_teleop_enabled_) {
      // Make sure velocities are zero when disabled
      current_linear_vel_ = 0.0;
      current_angular_vel_ = 0.0;
      return;
    }
    
    // Apply dead band to joystick values
    double joystick_x = applyDeadBand(msg->joystick_x);
    double joystick_y = applyDeadBand(msg->joystick_y);
    
    // Map joystick_y to linear velocity (forward/backward)
    // Positive Y = forward
    current_linear_vel_ = joystick_y * max_linear_vel_;
    
    // Map joystick_x to angular velocity (left/right rotation)
    // Positive X = turn right (negative angular velocity in ROS convention)
    current_angular_vel_ = -joystick_x * max_angular_vel_;
    
    // Log velocity commands for debugging (only when non-zero)
    if (std::abs(current_linear_vel_) > 0.01 || std::abs(current_angular_vel_) > 0.01) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Right Controller - Linear: %.2f, Angular: %.2f", 
                   current_linear_vel_, current_angular_vel_);
    }
  }
  
  // Left controller callback - reserved for special motions
  void leftControllerCallback(const unity_ros_interfaces::msg::Controller::SharedPtr msg)
  {
    // Detect secondary button press (rising edge) for toggle
    if (msg->secondary_button_pressed && !left_button_prev_state_) {
      // Button just pressed - toggle special motion state
      left_teleop_enabled_ = !left_teleop_enabled_;
      
      if (left_teleop_enabled_) {
        RCLCPP_INFO(this->get_logger(), "LEFT Controller Special Motions ENABLED");
      } else {
        RCLCPP_INFO(this->get_logger(), "LEFT Controller Special Motions DISABLED");
      }
    }
    
    // Update previous button state
    left_button_prev_state_ = msg->secondary_button_pressed;
    
    // Only process joystick input if special motions are enabled
    if (!left_teleop_enabled_) {
      left_joystick_x_ = 0.0;
      left_joystick_y_ = 0.0;
      current_strafe_vel_ = 0.0;
      return;
    }
    
    // Apply dead band to left joystick X axis for strafe movement
    double strafe_input = applyDeadBand(msg->joystick_x);
    
    // Map joystick_x to strafe velocity (left/right strafing)
    // Positive X = turn right = strafe right = negative linear.y
    // Negative X = turn left = strafe left = positive linear.y
    current_strafe_vel_ = -strafe_input * max_strafe_vel_;
    
    // Store the values for future use if needed
    left_joystick_x_ = strafe_input;
    left_joystick_y_ = applyDeadBand(msg->joystick_y);
    
    // Log strafe velocity for debugging (only when non-zero)
    if (std::abs(current_strafe_vel_) > 0.01) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Left Controller - Strafe: %.2f", 
                   current_strafe_vel_);
    }
  }
  
  // Publish cmd_vel at fixed rate
  void publishCmdVel()
  {
    auto twist_msg = geometry_msgs::msg::Twist();
    
    twist_msg.linear.x = current_linear_vel_;
    twist_msg.linear.y = current_strafe_vel_;
    twist_msg.linear.z = 0.0;
    
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = current_angular_vel_;
    
    cmd_vel_pub_->publish(twist_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<unity_ros_interfaces::msg::Controller>::SharedPtr right_controller_sub_;
  rclcpp::Subscription<unity_ros_interfaces::msg::Controller>::SharedPtr left_controller_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  double max_linear_vel_;
  double max_angular_vel_;
  double max_strafe_vel_;
  double dead_band_;
  
  // Current velocity commands
  double current_linear_vel_;
  double current_angular_vel_;
  double current_strafe_vel_;
  
  // Left controller joystick values (for future use)
  double left_joystick_x_;
  double left_joystick_y_;
  
  // Toggle states for teleoperation
  bool right_teleop_enabled_;
  bool left_teleop_enabled_;
  
  // Previous button states for edge detection
  bool right_button_prev_state_;
  bool left_button_prev_state_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RangerVRTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}