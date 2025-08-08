#ifndef VL53L1X_VL53L1X_DEVICE_HPP_
#define VL53L1X_Vl53L1X_DEVICE_HPP_

#include "tca9548a/I2CDevice.hpp"
#include "tca9548a/tca9548a.hpp"
#include "vl53l1x/vl53l1x.hpp"

namespace vl53l1x {
struct Config {
  uint16_t timeout;
  uint16_t timing_budget;
  float freq;
};
class Vl53l1xDevice : public tca9548a::I2CDevice {
public:
  Vl53l1xDevice(std::string i2c_bus, uint8_t sensor_address, Config config_);
  virtual ~Vl53l1xDevice() = default;
  bool initialize() override;
  bool configure() override;
  tca9548a::ReadResult read() override;

private:
  uint8_t sensor_address_;
  vl53l1x::Vl53l1x sensor_;

  Config config_;
};
}; // namespace vl53l1x

#endif