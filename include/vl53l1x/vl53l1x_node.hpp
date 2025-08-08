#ifndef VL53L1X_VL53L1X_NODE_HPP_
#define VL53L1X_VL53L1X_NODE_HPP_

#include "tca9548a/tca9548a.hpp"
#include "vl53l1x/vl53l1x.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

namespace vl53l1x {
class Vl53l1xNode : public rclcpp::Node {
public:
  explicit Vl53l1xNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // ROS2 Objects
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<tca9548a::srv::SelectChannel>::SharedPtr select_channel_client_;

  // Driver instances
  Vl53l1x sensor;

  // Parameter values
  std::string i2c_bus_param_;
  std::string tca_manager_node_name_param_;
  uint8_t tca_address_param_;
  uint8_t vl53l1x_channel_param_;
  uint8_t vl53l1x_address_param_;
  unsigned int timeout_;
  unsigned int timing_budget_;
  double freq_;

  // Callback function for timer
  void timer_callback();
};
}; // namespace vl53l1x
#endif