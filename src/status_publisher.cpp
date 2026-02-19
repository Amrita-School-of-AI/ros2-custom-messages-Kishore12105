#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include "ros2_custom_msgs/msg/robot_status.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
  StatusPublisher() : Node("status_publisher"),
                      battery_(100.0),
                      mission_count_(0)
  {
    publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>(
      "/robot_status", 10);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StatusPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Status publisher node started");
  }

private:
  void timer_callback()
  {
    auto message = ros2_custom_msgs::msg::RobotStatus();

    message.robot_name    = "Explorer1";
    message.battery_level = battery_;
    message.is_active     = true;
    message.mission_count = mission_count_;

    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(),
                "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
                message.robot_name.c_str(),
                message.battery_level,
                message.is_active ? "true" : "false",
                message.mission_count);

    // Update state for next tick
    battery_ -= 0.5;
    mission_count_ += 1;

    // Optional: prevent negative battery (not required, but nice)
    if (battery_ < 0.0) {
      battery_ = 0.0;
    }
  }

  rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double battery_;
  int mission_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisher>());
  rclcpp::shutdown();
  return 0;
}