cmake_minimum_required(VERSION 3.5)
project(blimp_description)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) 
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
# ADDED: Additional dependencies that might be needed for URDF processing
find_package(urdf REQUIRED)
find_package(urdf_parser_plugin REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Install Python scripts 
install(PROGRAMS
  script/px4_blimp_control.py
  script/mavros_sensor_bridge.py
  script/px4_sitl_launcher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  urdf
  meshes
  worlds
  rviz
  resource
  script
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()