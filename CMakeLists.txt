cmake_minimum_required(VERSION 3.8)
project(vl53l1x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tca9548a REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(vl53l1x_driver SHARED
  src/vl53l1x.cpp
  src/vl53l1x_device.cpp
)

# Use ament_target_dependencies for external ROS packages
ament_target_dependencies(vl53l1x_driver
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
  "tca9548a"
)

pluginlib_export_plugin_description_file(vl53l1x_driver vl53l1x_plugins.xml)

install(TARGETS vl53l1x_driver
  EXPORT vl53l1x_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(vl53l1x_driver_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  pluginlib
  tca9548a
  sensor_msgs
)

ament_package()