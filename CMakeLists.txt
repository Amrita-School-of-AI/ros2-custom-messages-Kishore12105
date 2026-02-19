cmake_minimum_required(VERSION 3.8)
project(ros2_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate the interface (message)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)

# === Build the node ===
add_executable(status_publisher src/status_publisher.cpp)

# Link dependencies from other packages
ament_target_dependencies(status_publisher
  rclcpp
)

# Link the generated interface (THIS WAS MISSING / WRONG BEFORE)
rosidl_target_interfaces(status_publisher
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

# Install the executable
install(TARGETS
  status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Export runtime dependency so downstream packages know about the messages
ament_export_dependencies(rosidl_default_runtime)

ament_package()