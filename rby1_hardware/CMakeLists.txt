cmake_minimum_required(VERSION 3.10)
project(rby1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find PythonInterp policy warning suppression
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# SDK Path Resolution
if(DEFINED ENV{RBY1_SDK_PATH})
  set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
else()
  set(RBY1_SDK_PATH "/home/jsm/sdk/rby1-sdk")
endif()

# Include directories
include_directories(
  include
  ${RBY1_SDK_PATH}/include
  ${RBY1_SDK_PATH}/build/private
  /usr/include/eigen3
)

# Export plugins
pluginlib_export_plugin_description_file(hardware_interface rby1_hardware.xml)

# Create hardware interface shared library
add_library(rby1_hardware SHARED
  src/rby1_system_hardware.cpp
)

# Set dependencies
ament_target_dependencies(rby1_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Link SDK library
target_link_directories(rby1_hardware PRIVATE ${RBY1_SDK_PATH}/build/src)
target_link_libraries(rby1_hardware rby1-sdk)

target_include_directories(rby1_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rby1_hardware
  EXPORT export_rby1_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_rby1_hardware)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
