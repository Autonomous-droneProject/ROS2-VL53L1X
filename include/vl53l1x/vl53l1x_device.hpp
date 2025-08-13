#ifndef VL53L1X_VL53L1X_DEVICE_HPP_
#define VL53L1X_Vl53L1X_DEVICE_HPP_

#include "tca9548a/i2c_device.hpp"
#include "tca9548a/tca9548a.hpp"
#include "tca9548a/msg/sensor_data.hpp"
#include "vl53l1x/vl53l1x.hpp"
#include <any>

namespace vl53l1x {
struct Config {
  int timeout;
  int timing_budget;
  double freq;
};
class Vl53l1xDevice : public tca9548a::I2CDevice {
public:
  Vl53l1xDevice();
  Vl53l1xDevice(std::string i2c_bus, const std::map<std::string, std::any>& parmaters);
  virtual ~Vl53l1xDevice() = default;
  bool initialize() override;
  bool configure() override;
  tca9548a::msg::SensorData read() override;

private:
  vl53l1x::Vl53l1x sensor_;
  const std::map<std::string, std::any> parameters_;
  Config config_;
};
}; // namespace vl53l1x

#endif