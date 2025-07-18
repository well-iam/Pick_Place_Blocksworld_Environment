cmake_minimum_required(VERSION 3.8)
project(my_sim_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- message generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlockPose.msg"
  "msg/BlockPoseArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# export rosidl runtime so that other packages can consume your msgs
ament_export_dependencies(rosidl_default_runtime)

# --- plugin library ---
add_library(pose_relay_plugin SHARED
  src/pose_relay_plugin.cpp
)

# include generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(pose_relay_plugin ${cpp_typesupport_target})


ament_target_dependencies(pose_relay_plugin
  rclcpp
  std_msgs
  geometry_msgs
  ignition-gazebo6
  ignition-transport11
)

# install the plugin
install(TARGETS pose_relay_plugin
  LIBRARY DESTINATION lib
)

# install msg definitions so that they can be found by colcon test/install
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
