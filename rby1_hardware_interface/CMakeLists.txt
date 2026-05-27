cmake_minimum_required(VERSION 3.8)
project(rby1_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

if(NOT DEFINED ENV{RBY1_SDK_PATH})
  message(FATAL_ERROR "RBY1_SDK_PATH must be set to build rby1_hardware_interface")
endif()

set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})

add_library(rby1_robot_client SHARED
  src/rby1_robot_client.cpp
)
target_include_directories(rby1_robot_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${RBY1_SDK_PATH}/include
    ${RBY1_SDK_PATH}/build/private
    /usr/include/eigen3
)
target_link_directories(rby1_robot_client PRIVATE ${RBY1_SDK_PATH}/build/src)
target_link_libraries(rby1_robot_client PRIVATE rby1-sdk)

add_library(rby1_system_interface SHARED
  src/rby1_system_interface.cpp
)
target_include_directories(rby1_system_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rby1_system_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(rby1_system_interface PUBLIC rby1_robot_client)

pluginlib_export_plugin_description_file(hardware_interface rby1_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rby1_robot_client rby1_system_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(rby1_robot_client rby1_system_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
