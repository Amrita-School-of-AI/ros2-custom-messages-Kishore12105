#include <rclcpp/rclcpp.hpp>
#include "ros2_custom_msgs/msg/robot_status.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

class StatusPublisher : public rclcpp::Node
{
public:
  StatusPublisher()
  : Node("status_publisher"),
    battery_level_(100.0),
    mission_count_(0)
  {
    publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>(
      "/robot_status", 10);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StatusPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Status publisher node has started");
  }

private:
  void timer_callback()
  {
    auto msg = ros2_custom_msgs::msg::RobotStatus();

    msg.robot_name     = "Explorer1";
    msg.battery_level  = battery_level_;
    msg.is_active      = true;
    msg.mission_count  = mission_count_;

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
                msg.robot_name.c_str(),
                msg.battery_level,
                msg.is_active ? "true" : "false",
                msg.mission_count);

    // Update for next cycle
    battery_level_ -= 0.5;
    mission_count_ += 1;

    if (battery_level_ < 0.0) {
      battery_level_ = 0.0;
    }
  }

  rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double battery_level_;
  int mission_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisher>());
  rclcpp::shutdown();
  return 0;
}