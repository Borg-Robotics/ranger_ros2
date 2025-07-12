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
* For Aruco Marker Actions, you need to launch **aruco_recognition** and **ranger_server** node.
    ```shell
        ros2 launch ros2_aruco aruco_recognition.launch.py
    ```
    ```shell
        ros2 launch ranger_base ranger_server.launch.py
    ```
    1. `/search_aruco`: The robot rotates in search for the specified aurco marker ID until found or full rotation.
          ```shell
            ros2 action send_goal /search_aruco ranger_msgs/action/ArucoSearch "{marker_id: 1, angular_vel: 0.2, direction: -1}" -f
          ``` 
        - Optional: direction: 
          - 1  -> counter-clockwise rotation [default]
          - -1 -> clockwise rotation
    2. `/follow_aruco`: The robot orient itself to face the aruco marker with the **ID** specified and the **minimum distance** and follows the aruco marker.
        ```shell
            ros2 action send_goal /follow_aruco ranger_msgs/action/ArucoFollow "{marker_id: 1, min_distance: 0.5, linear_vel: 1.0, angular_vel: 0.5}" -f
        ``` 
        - Optional: linear_vel & angular_vel
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

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `follow_action.default_linear_vel` | double | 0.25 | Default linear velocity for follow action (m/s) |
| `follow_action.default_angular_vel` | double | 0.15 | Default angular velocity for follow action (rad/s) |
| `follow_action.default_min_distance` | double | 1.0 | Default minimum distance to marker (m) |
| `follow_action.min_linear_vel` | double | 0.15 | Minimum linear velocity when approaching target (m/s) |
| `follow_action.max_linear_vel` | double | 0.5 | Maximum allowed linear velocity (m/s) |
| `follow_action.max_angular_vel` | double | 0.3 | Maximum allowed angular velocity (rad/s) |
| `follow_action.angular_threshold` | double | 0.01 | Angular threshold for alignment (rad) |
| `follow_action.marker_timeout` | double | 2.0 | Timeout for marker detection loss (s) |
| `search_action.default_angular_vel` | double | 0.2 | Default angular velocity for search action (rad/s) |
| `search_action.default_direction` | int | 1 | Default search direction (1: counter-clockwise, -1: clockwise) |
| `search_action.max_angular_vel` | double | 0.3 | Maximum allowed angular velocity for search (rad/s) |
| `search_action.search_timeout` | double | 60.0 | Maximum time to search for marker (s) |
| `topics.aruco_markers` | string | "/aruco_markers" | ArUco markers topic |
| `topics.cmd_vel` | string | "/cmd_vel" | Velocity command topic |
| `control.loop_rate` | double | 10.0 | Control loop frequency (Hz) |
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
* /follow_aruco (`ranger_msgs::action::FollowAruco`)
* /search_aruco (`ranger_msgs::action::SearchAruco`)