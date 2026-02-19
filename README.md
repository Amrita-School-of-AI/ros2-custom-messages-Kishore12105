[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/gttpdjHD)
# Assignment: ROS2 Custom Messages

**Course:** ROS2 Fundamentals  
**Build System:** ament_cmake

## Objective

The goal of this assignment is to verify your understanding of:

1. How to define custom ROS2 message types.
2. How to generate message code using `rosidl`.
3. How to use custom messages in publishers and subscribers.

## Problem Statement

You must complete the provided skeleton code to create a custom message and use it in a publisher node.

### Requirements

1. **Message Definition (`msg/RobotStatus.msg`)**:
   Create a custom message with the following fields:
   - `string robot_name`
   - `float64 battery_level`
   - `bool is_active`
   - `int32 mission_count`

2. **Source Code (`src/status_publisher.cpp`)**:
   - Implement a class `StatusPublisher` that inherits from `rclcpp::Node`.
   - Initialize the node with the name `"status_publisher"`.
   - Create a publisher to topic `"/robot_status"` with your custom `RobotStatus` message.
   - Create a timer that fires every **1000ms**.
   - In the timer callback:
     - Create a RobotStatus message with:
       - `robot_name`: "Explorer1"
       - `battery_level`: decreasing from 100.0 by 0.5 each tick
       - `is_active`: true
       - `mission_count`: incrementing from 0
     - Publish the message.
     - Log the status.

3. **Build Configuration (`CMakeLists.txt`)**:
   - Use `rosidl_generate_interfaces` to generate message code.
   - Add executable and link dependencies.

4. **Package Metadata (`package.xml`)**:
   - Add required dependencies for message generation.

## How to Test Locally

```bash
colcon build --packages-select ros2_custom_msgs
source install/setup.bash
ros2 run ros2_custom_msgs status_publisher

# In another terminal
ros2 topic echo /robot_status
```

#### Expected Output:

```shell
[INFO] [1700000000.123456789] [status_publisher]: Publishing: robot=Explorer1, battery=100.0, active=true, missions=0
[INFO] [1700000001.123456789] [status_publisher]: Publishing: robot=Explorer1, battery=99.5, active=true, missions=1
...
```

#### Expected Topic Echo:

```
robot_name: Explorer1
battery_level: 99.5
is_active: true
mission_count: 1
---
```
