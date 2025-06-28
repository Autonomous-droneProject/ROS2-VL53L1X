cmake_minimum_required(VERSION 3.8)
project(vl53l1x)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ===================================================================
# == 1. CREATE THE REUSABLE DRIVER LIBRARY                         ==
# ===================================================================
# This creates a library from the core sensor C++ file so it can be
# shared with the node in this package and with other ROS 2 packages.
add_library(vl53l1x_driver src/vl53l1x.cpp)

# Specify the include directories for the library.
# PUBLIC makes these directories available to any target that links to this library.
target_include_directories(vl53l1x_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ===================================================================
# == 2. DEFINE THE EXECUTABLE NODE                                 ==
# ===================================================================
# This defines the main node that runs the sensor.
add_executable(vl53l1x_node src/vl53l1x_node.cpp)

# Specify ROS dependencies for the executable.
ament_target_dependencies(vl53l1x_node
  "rclcpp"
  "sensor_msgs"
)

# Link the executable against the driver library created above.
target_link_libraries(vl53l1x_node vl53l1x_driver)


# ===================================================================
# == 3. SET UP INSTALLATION AND EXPORT RULES                       ==
# ===================================================================
# This section ensures all necessary files are placed in the `install`
# directory after building, and that other packages can find our library.

# Install the executable target.
install(TARGETS
  vl53l1x_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library target and define an export name.
install(TARGETS vl53l1x_driver
  EXPORT export_vl53l1x_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the public header files.
install(DIRECTORY
  include/
  DESTINATION include
)

# Export the library target. This is crucial for other packages to be able
# to find and link against `vl53l1x_driver` using find_package().
ament_export_targets(export_vl53l1x_driver
  HAS_LIBRARY_TARGET
)

# This command must be at the end of the file.
ament_package()