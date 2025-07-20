cmake_minimum_required(VERSION 3.8)
project(blimp_gazebo_plugin)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Find Gazebo packages through ROS2 vendor packages
find_package(gz_cmake_vendor REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)

# REVERT: Try original versions that might be correct for your system
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-sim8 REQUIRED)

# NormWind Plugin
add_library(normwind_plugin SHARED src/normwind_plugin.cpp)
target_link_libraries(normwind_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
)
target_include_directories(normwind_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# FinLiftDrag Plugin
add_library(finliftdrag_plugin SHARED src/finliftdrag_plugin.cpp)
target_link_libraries(finliftdrag_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
)
target_include_directories(finliftdrag_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# DynamicVolume Plugin
add_library(dynamicvolume_plugin SHARED src/dynamicvolume_plugin.cpp)
target_link_libraries(dynamicvolume_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
)
target_include_directories(dynamicvolume_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Installation directives
install(TARGETS
  normwind_plugin
  finliftdrag_plugin
  dynamicvolume_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch OPTIONAL)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models OPTIONAL)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Environment hook for auto-registering plugins
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/blimp_gazebo_plugin.dsv.in")

ament_package()