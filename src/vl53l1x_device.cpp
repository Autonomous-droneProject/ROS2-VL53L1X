#include "vl53l1x/vl53l1x_device.hpp"

namespace vl53l1x {
Vl53l1xDevice::Vl53l1xDevice(std::string i2c_bus, uint8_t sensor_address,
                             Config config)
    : sensor_(i2c_bus), config_(config) {}
bool Vl53l1xDevice::initialize() { return sensor_.init(); }

bool Vl53l1xDevice::configure() {
  sensor_.setDistanceMode(vl53l1x::Vl53l1x::DistanceMode::Short);
  sensor_.setMeasurementTimingBudget(config_.timing_budget);
  sensor_.setTimeout(config_.timeout);
  sensor_.startContinuous(static_cast<uint32_t>(1000.0 / config_.freq));
}

tca9548a::ReadResult Vl53l1xDevice::read() {
  tca9548a::ReadResult result;
  result.reading = sensor_.read_range();
  result.success = !sensor_.timeoutOccurred();
  return result;
}
} // namespace vl53l1x