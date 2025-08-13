#include "vl53l1x/vl53l1x_device.hpp"

namespace vl53l1x {
Vl53l1xDevice::Vl53l1xDevice() {}
Vl53l1xDevice::Vl53l1xDevice(std::string i2c_bus, const std::map<std::string, std::any>& parameters)
    : sensor_(i2c_bus), parameters_(parameters){
    }
bool Vl53l1xDevice::initialize() { return sensor_.init(); }

bool Vl53l1xDevice::configure() {
  config_.timeout = static_cast<int>(tca9548a::get_param<int64_t>(parameters_, std::string("timeout")));
  config_.timing_budget = static_cast<int>(tca9548a::get_param<int64_t>(parameters_, std::string("timing_budget")));
  config_.freq = tca9548a::get_param<double>(parameters_, std::string("freq"));
  sensor_.setDistanceMode(vl53l1x::Vl53l1x::DistanceMode::Short);
  sensor_.setMeasurementTimingBudget(config_.timing_budget);
  sensor_.setTimeout(config_.timeout);
  sensor_.startContinuous(static_cast<uint32_t>(1000.0 / config_.freq));
  return true;
}

tca9548a::msg::SensorData Vl53l1xDevice::read() {
  tca9548a::msg::SensorData data;

  // Get the distance reading from the low-level driver
  uint16_t reading = sensor_.read_range();
  bool success = !sensor_.timeoutOccurred();

  if (success) {
    // Populate the header with a timestamp
    data.header.stamp = rclcpp::Clock().now();
    data.device_name = "VL53L1X";

    // Add the distance reading as a key-value pair
    diagnostic_msgs::msg::KeyValue distance_kv;
    distance_kv.key = "distance_mm";
    distance_kv.value = std::to_string(reading);
    data.values.push_back(distance_kv);
    
    // You can add more data here if needed (e.g., status codes)
  } else {
    // On failure, return an empty message with a null timestamp
    data.header.stamp = rclcpp::Time(0, 0);
  }

  return data;
}
} // namespace vl53l1x

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(vl53l1x::Vl53l1xDevice, tca9548a::I2CDevice)