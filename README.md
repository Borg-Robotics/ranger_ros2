# ROS2 Packages for Ranger Robot

This repository is a fork of [agilexrobotics/ranger_ros2](https://github.com/agilexrobotics/ranger_ros2) that contains ROS2 support packages for the Ranger robot bases to provide a ROS interface to the robot.

## Setup CAN-To-USB adapter
> **Note:** This setup is done automatically when running [`{workspace}/docker/run_hw.bash`](../../../docker/run_hw.bash).

* first time use ranger-ros package
   * enable kernel module: gs_usb

   ```shell
   $ sudo modprobe gs_usb
   ```
   
* if not the first time use ranger-ros package(Run this command every time you turn off the power) 
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

## To bring up real ranger hardware: Launch Base Node for ranger

* Start the base node for ranger with default parameters

    ```shell
    $ ros2 launch ranger_base ranger_bringup.launch.py
    ```
> **Note:** To change the default parameters you can change [`config\ranger_base_params.yaml`](../ranger_ros2/ranger_base/config/ranger_base_params.yaml) before launching

## Use Services/Actions
* For `/ranger_base_node/parking_service `, you need the **ranger_base_node** running with *ranger_bringup.launch*
  * Request Service
    ```shell
    ros2 service call /ranger_base_node/parking_service ranger_msgs/srv/TriggerParkMode "{trigger_parked_mode: true}"
    ```
* For ArUco Marker Actions, you need to launch **aruco_recognition** and **ranger_server** node.
    ```shell
        ros2 launch ros2_aruco aruco_recognition.launch.py
    ```
    ```shell
        ros2 launch ranger_base ranger_server.launch.py
    ```
    
    ### ArUco Action Features
    
    The ranger_server provides advanced ArUco marker tracking with the following enhanced capabilities:
    - **Automatic alignment** with markers for both search and follow actions
    - **Trapezoidal velocity profile** for smooth acceleration/deceleration in follow action
    - **Combined angular and linear movement** for fluid motion (no more stop-and-turn behavior)
    - **Robust marker timeout handling** with graceful recovery
    - **Safety velocity limits** with parameter validation
    
    #### Available Actions:
    
    1. **`/search_aruco`**: The robot rotates to search for the specified ArUco marker ID. Upon detection, it automatically aligns with the marker before completing the action.
          ```shell
            ros2 action send_goal /search_aruco ranger_msgs/action/ArucoSearch "{marker_id: 1, angular_vel: 0.2, direction: -1}" -f
          ``` 
        - **Parameters:**
          - `marker_id` (required): ArUco marker ID to search for
          - `angular_vel` (optional): Rotation velocity during search (default: 0.2 rad/s)
          - `direction` (optional): Search direction
            - `1` → counter-clockwise rotation [default]
            - `-1` → clockwise rotation
        - **Behavior:** 
          - Robot rotates until marker is found or timeout occurs
          - Upon detection, enters alignment phase for precise orientation
          - Completes successfully when aligned within tolerance
    
    2. **`/follow_aruco`**: The robot smoothly approaches and follows the ArUco marker with specified minimum distance, featuring smooth trapezoidal acceleration/deceleration and combined angular-linear movement.
        ```shell
            ros2 action send_goal /follow_aruco ranger_msgs/action/ArucoFollow "{marker_id: 1, min_distance: 0.5, linear_vel: 0.3, angular_vel: 0.2}" -f
        ``` 
        - **Parameters:**
          - `marker_id` (required): ArUco marker ID to follow
          - `min_distance` (required): Minimum distance to maintain from marker (meters)
          - `linear_vel` (optional): Maximum linear velocity (default: 0.25 m/s)
          - `angular_vel` (optional): Maximum angular velocity (default: 0.15 rad/s)
        - **Behavior:**
          - Smooth acceleration using trapezoidal velocity profile
          - Simultaneous linear and angular movement for fluid motion
          - Automatic deceleration when approaching target distance
          - Continuous alignment correction during movement
          - Maintains specified distance while tracking marker movement

## Configuration Tips

### 🔧 **Tuning for Different Environments**

**For High-Precision Applications:**
```yaml
follow_action:
  alignment_tolerance: 0.005    # ~0.29° - very precise
  angular_gain: 750.0           # Higher gain for faster correction
  max_acceleration: 0.15        # Gentler acceleration
```

**For Fast Response Applications:**
```yaml
follow_action:
  max_linear_vel: 1.0          # Higher speed limit
  max_acceleration: 0.4        # Faster acceleration
  alignment_tolerance: 0.02    # ~1.15° - looser tolerance
```

**For Small/Confined Spaces:**
```yaml
follow_action:
  default_linear_vel: 0.15     # Slower default speed
  deceleration_distance_threshold: 0.5  # Start slowing earlier
  min_deceleration_velocity: 0.05       # Slower final approach
```

### **Safety Considerations**

- **Velocity Limits**: Always set appropriate `max_linear_vel` and `max_angular_vel` for your environment
- **Timeout Settings**: Adjust `marker_timeout` based on your marker detection reliability
- **Distance Settings**: Ensure `min_distance` provides safe clearance for your application
- **Feedback Rate**: Set `feedback_rate` independent of `loop_rate` to prevent message flooding (recommended max: 10 Hz)
- **Debug Mode**: Enable `debug_enabled: true` during initial testing and tuning
## ROS interface

### Parameters
#### ranger_base_params.yaml
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port_name` | string | "can0" | CAN port name for robot communication |
| `robot_model` | string | "ranger_mini_v2" | Robot model (ranger_mini_v1, ranger_mini_v2, ranger) |
| `update_rate` | int | 50 | Update rate in Hz for state publishing |
| `odom_frame` | string | "odom" | Odometry frame name |
| `base_frame` | string | "robot_footprint" | Base frame name |
| `odom_topic_name` | string | "odom" | Odometry topic name |
| `publish_odom_tf` | bool | true | Whether to publish odometry transform |
#### ranger_server_params.yaml

The ranger_server parameters have been significantly enhanced to support advanced ArUco tracking features including automatic alignment, trapezoidal velocity profiles, and smooth combined movement.

##### Follow Action Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `follow_action.default_linear_vel` | double | 0.25 | Default linear velocity for follow action (m/s) |
| `follow_action.default_angular_vel` | double | 0.15 | Default angular velocity for follow action (rad/s) |
| `follow_action.max_linear_vel` | double | 1.0 | Maximum allowed linear velocity (m/s) |
| `follow_action.max_angular_vel` | double | 0.3 | Maximum allowed angular velocity (rad/s) |
| `follow_action.marker_timeout` | double | 2.0 | Timeout for marker detection loss (s) |
| `follow_action.angular_gain` | double | 500.0 | Proportional gain for angular control |
| `follow_action.alignment_tolerance` | double | 0.01 | Tolerance for alignment dead-zone (rad, ~0.57°) |

##### Trapezoidal Velocity Profile Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `follow_action.max_acceleration` | double | 0.25 | Maximum acceleration (m/s²) |
| `follow_action.max_deceleration` | double | 0.25 | Maximum deceleration (m/s²) |
| `follow_action.min_linear_vel_threshold` | double | 0.05 | Minimum linear velocity threshold (m/s) |
| `follow_action.min_angular_vel_threshold` | double | 0.05 | Minimum angular velocity threshold (rad/s) |
| `follow_action.deceleration_step_size` | double | 0.05 | Step size for smooth deceleration (m/s) |
| `follow_action.deceleration_distance_threshold` | double | 0.3 | Distance threshold for minimum deceleration velocity (m) |
| `follow_action.min_deceleration_velocity` | double | 0.1 | Minimum velocity during deceleration phase (m/s) |

##### Search Action Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `search_action.default_angular_vel` | double | 0.2 | Default angular velocity for search action (rad/s) |
| `search_action.default_direction` | int | 1 | Default search direction (1: counter-clockwise, -1: clockwise) |
| `search_action.max_angular_vel` | double | 0.3 | Maximum allowed angular velocity for search (rad/s) |
| `search_action.search_timeout` | double | 60.0 | Maximum time to search for marker (s) |
| `search_action.alignment_tolerance` | double | 0.01 | Tolerance for alignment phase (rad, ~0.57°) |

##### System Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topics.aruco_markers` | string | "/aruco_markers" | ArUco markers topic |
| `topics.cmd_vel` | string | "/cmd_vel" | Velocity command topic |
| `control.loop_rate` | double | 10.0 | Control loop frequency (Hz) |
| `control.feedback_rate` | double | 5.0 | Feedback publishing frequency (Hz) - Independent of control loop rate |
| `logging.debug_enabled` | bool | false | Enable debug logging |
| `logging.throttle_duration` | int | 2000 | Duration for throttled warnings (ms) |

### Published topics

* /system_state (`ranger_msgs::SystemState`)
* /motion_state (`ranger_msgs::MotionState`)
* /actuator_state (`ranger_msgs::ActuatorStateArray`)
* /odom (`nav_msgs::Odometry`)
* /battery_state (`sensor_msgs::BatteryState`)

### Subscribed topics

* /cmd_vel (`geometry_msgs::Twist`)

### Services

* /ranger_base_node/parking_service (`ranger_msgs::srv:TriggerParkMode`)

### Actions

#### Enhanced ArUco Actions with Advanced Features

* **`/follow_aruco`** (`ranger_msgs::action::ArucoFollow`)
  - **Goal Fields:**
    - `int64 marker_id` - Target ArUco marker ID (required)
    - `float32 min_distance` - Minimum distance to maintain from marker in meters (required)
    - `float32 linear_vel` - Maximum linear velocity (optional, uses default if ≤ 0)
    - `float32 angular_vel` - Maximum angular velocity (optional, uses default if ≤ 0)
  - **Features:**
    - Trapezoidal velocity profile for smooth acceleration/deceleration
    - Combined angular and linear movement for fluid motion
    - Automatic alignment with sub-degree precision
    - Safety velocity limiting with parameter validation
  - **Feedback:**
    - Current distance to marker
    - Current linear and angular velocities
    - Alignment status and marker detection state

* **`/search_aruco`** (`ranger_msgs::action::ArucoSearch`)
  - **Goal Fields:**
    - `int64 marker_id` - Target ArUco marker ID to search for (required)
    - `float32 angular_vel` - Rotation velocity during search (optional, uses default if ≤ 0)
    - `int64 direction` - Search direction: 1 for counter-clockwise, -1 for clockwise (optional)
  - **Features:**
    - Intelligent search pattern with configurable direction
    - Automatic alignment phase upon marker detection
    - Timeout protection with configurable duration
    - Precise final orientation within tolerance
  - **Feedback:**
    - Search progress and current rotation angle
    - Marker detection status
    - Alignment phase progress
### Debug Mode

Enable detailed logging for troubleshooting:
```yaml
logging:
  debug_enabled: true
  throttle_duration: 1000  # Reduce for more frequent debug output
```

This will provide detailed information about:
- Control loop execution
- Velocity calculations and limits
- Marker detection and pose estimation
- Action state transitions