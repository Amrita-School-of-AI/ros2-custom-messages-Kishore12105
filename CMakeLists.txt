cmake_minimum_required(VERSION 3.8)
project(ros2_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)          # usually good to have

# Generate interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)

# Create executable
add_executable(status_publisher src/status_publisher.cpp)

# Link dependencies
ament_target_dependencies(status_publisher
  rclcpp
  ros2_custom_msgs          # ← this is important after interface generation
)

# Install targets
install(TARGETS
  status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Required for message generation to work correctly
ament_export_dependencies(rosidl_default_runtime)

ament_package()