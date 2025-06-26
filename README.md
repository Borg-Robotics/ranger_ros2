# ROS2 Packages for Ranger Robot

This repository is a fork of [agilexrobotics/ranger_ros2](https://github.com/agilexrobotics/ranger_ros2) that contains ROS2 support packages for the Ranger robot bases to provide a ROS interface to the robot.

## Setup CAN-To-USB adapter
    
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
    
## To brinh up real ranger hardware: Launch Base Node for ranger

* Start the base node for ranger with default parameters

    ```shell
    $ ros2 launch ranger_base ranger_bringup.launch.py
    ```
* Launch with custom parameters:

    ```shell
    $ ros2 launch ranger_base ranger_bringup.launch.py port_name:=can1 update_rate:=100
    ```

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
        ros2 run ranger_base ranger_server
    ```
    1. `/search_aruco`: The robot rotates in search for the specified aurco marker ID until found or full rotation.
          ```shell
            ros2 action send_goal /search_aruco ranger_msgs/action/ArucoSearch "{marker_id: 1, direction: -1}" -f
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

* **can_device** (*string*): `can0`
* **robot_model** (*string*): Model of the robot
    - `"ranger"`: Standard Ranger model
    - `"ranger_mini_v1"`: Ranger Mini V1 model
    - `"ranger_mini_v2"`: Ranger Mini V2 model
    - `"ranger_mini_v3"`: Ranger Mini V3 model
* **update_rate** (int): `50`
* **base_frame** (string): `"robot_footprint"` - `"ranger_base_link"`
* **odom_frame** (string): `odom`
* **publish_odom_tf** (bool): `true`
* **odom_topic_name** (string): `odom`

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