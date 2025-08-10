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

add_library(vl53l1x_driver src/vl53l1x.cpp)
target_include_directories(vl53l1x_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vl53l1x_driver PUBLIC
  tca9548a::tca9548a_driver
)

add_executable(vl53l1x_manager src/vl53l1x_manager.cpp)

# Use ament_target_dependencies for external ROS packages
ament_target_dependencies(vl53l1x_manager
  "rclcpp"
  "sensor_msgs"
  "tca9548a"
)

# Use target_link_libraries for local libraries in the same package
target_link_libraries(vl53l1x_manager
  vl53l1x_driver
)

# --- Modern Installation & Export Rules for Jazzy ---
install(TARGETS
  vl53l1x_manager
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_export_dependencies(tca9548a)

ament_package()