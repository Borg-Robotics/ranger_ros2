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
    
## Launch Base Node for ranger

* Start the base node for ranger with default parameters

    ```shell
    $ ros2 launch ranger_base ranger_bringup.launch.py
    ```
* Launch with custom parameters:

    ```shell
    $ ros2 launch ranger_base ranger_bringup.launch.py port_name:=can1 update_rate:=100
    ```
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
