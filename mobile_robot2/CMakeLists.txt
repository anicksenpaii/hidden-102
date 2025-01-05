cmake_minimum_required(VERSION 3.8)
project(mobile_robot2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# Add Python scripts to the install directory
install(PROGRAMS
  src/cords.py
  src/synchroniser.py
  src/gazebo.py
  src/slam.py
  config/slam_toolbox_params.yaml
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files and models
install(DIRECTORY 
  launch
  model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
