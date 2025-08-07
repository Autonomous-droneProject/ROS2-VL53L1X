#include "vl53l1x/vl53l1x_node.hpp"

namespace vl53l1x {
Vl53l1xNode::Vl53l1xNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("vl53l1x_node", options), tca_driver_("/dev/i2c-1", 0x70),
      sensor("dev/i2c-1") {
  // --- Declare and get params ---
  RCLCPP_INFO(this->get_logger(), "Starting VL53L1X Ros node...");
  i2c_bus_param_ =
      this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
  int tca_address_int = this->declare_parameter<int>("tca_address", 0x70);
  tca_address_param_ = static_cast<uint8_t>(tca_address_int);
  int vl53l1x_channel_int = this->declare_parameter<int>("vl53l1x_channel", 0);
  vl53l1x_channel_param_ = static_cast<uint8_t>(vl53l1x_channel_int);
  timeout_ = this->declare_parameter<unsigned int>("timeout", 500);
  timing_budget_ =
      this->declare_parameter<unsigned int>("timing_budget", 50000);
  freq_ = this->declare_parameter<double>("publish_rate", 10.0);

  // --- Iniitalize drivers ---
  sensor = Vl53l1x(i2c_bus_param_);
  if (!sensor.init()) {
    RCLCPP_ERROR(this->get_logger(), "VL53L1X offline!");
  }

  sensor.setDistanceMode(Vl53l1x::Short);
  sensor.setTimeout(timeout_);
  sensor.setMeasurementTimingBudget(timing_budget_);

  // --- Begin measuring ---
  sensor.startContinuous(static_cast<int>(1000.0 / freq_));

  // --- Setup publisher ---
  publisher_ =
      this->create_publisher<sensor_msgs::msg::Range>("vl53l1x/range", 5);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
      std::bind(&Vl53l1xNode::timer_callback, this));
}
void Vl53l1xNode::timer_callback() {
  int distance = sensor.read_range();
  if (sensor.timeoutOccurred()) {
    RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
    distance = 0;
  }

  rclcpp::Time now = this->get_clock()->now();
  auto message = sensor_msgs::msg::Range();
  message.header.frame_id = "vl53l1x";
  message.header.stamp = now;
  message.radiation_type = sensor_msgs::msg::Range::INFRARED;
  message.field_of_view = 0.47; // Typically 27 degrees or 0,471239 radians
  message.min_range = 0.14; // 140 mm.  (It is actully much less, but this makes
                            // sense in the context
  message.max_range = 3.00; // 3.6 m. in the dark, down to 73cm in bright light

  message.range = (float)distance / 1000.0; // range in meters

  // from
  // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg
  // # (Note: values < range_min or > range_max should be discarded)
  if ((message.range >= message.min_range) &&
      (message.range <= message.max_range)) {
    publisher_->publish(message);
  }
}
} // namespace vl53l1x