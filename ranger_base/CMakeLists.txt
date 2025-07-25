cmake_minimum_required(VERSION 3.5)
project(ranger_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(ranger_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  # ${EIGEN3_INCLUDE_DIR}
)

# ranger_base_node
add_executable(ranger_base_node
    src/ranger_base_node.cpp
    src/ranger_messenger.cpp)
    
target_link_libraries(ranger_base_node ugv_sdk)
ament_target_dependencies(ranger_base_node rclcpp tf2 tf2_ros std_msgs nav_msgs sensor_msgs ranger_msgs tf2_geometry_msgs)

install(TARGETS ranger_base_node
  DESTINATION lib/${PROJECT_NAME})

# ranger_server node
add_executable(ranger_server
    src/ranger_server.cpp)
ament_target_dependencies(ranger_server rclcpp rclcpp_action ranger_msgs ros2_aruco_interfaces geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS ranger_server
  DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )
