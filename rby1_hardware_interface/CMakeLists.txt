cmake_minimum_required(VERSION 3.16)
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
find_package(Eigen3 REQUIRED)

# ---- Locate rby1-sdk ---------------------------------------------------------
# Resolution order:
#   1) -DRBY1_SDK_ROOT=/path (user-provided install prefix)
#   2) find_package(rby1-sdk CONFIG) if SDK ships CMake config files
#   3) Python (pip-installed) fallback via importlib
set(RBY1_SDK_ROOT "" CACHE PATH "Path to rby1-sdk install prefix (contains include/ and lib/)")

find_package(rby1-sdk CONFIG QUIET)

if(NOT RBY1_SDK_INCLUDE_DIR OR NOT RBY1_SDK_LIBRARY)
  if(RBY1_SDK_ROOT)
    find_path(RBY1_SDK_INCLUDE_DIR
      NAMES "rby1-sdk/robot.h"
      HINTS "${RBY1_SDK_ROOT}/include"
      NO_DEFAULT_PATH
    )
    find_library(RBY1_SDK_LIBRARY
      NAMES rby1-sdk rby1_sdk
      HINTS "${RBY1_SDK_ROOT}/lib" "${RBY1_SDK_ROOT}/lib64"
      NO_DEFAULT_PATH
    )
  endif()
endif()

if(NOT RBY1_SDK_INCLUDE_DIR OR NOT RBY1_SDK_LIBRARY)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import importlib.util, pathlib; \
spec = importlib.util.find_spec('rby1_sdk'); \
print(pathlib.Path(spec.origin).parent if spec else '')"
    OUTPUT_VARIABLE _rby1_sdk_py_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _rby1_sdk_py_result
  )
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.prefix)"
    OUTPUT_VARIABLE _python_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(_rby1_sdk_py_result EQUAL 0 AND _rby1_sdk_py_dir)
    find_path(RBY1_SDK_INCLUDE_DIR
      NAMES "rby1-sdk/robot.h"
      HINTS "${_rby1_sdk_py_dir}/include" "${_python_prefix}/include"
    )
    find_library(RBY1_SDK_LIBRARY
      NAMES rby1-sdk rby1_sdk
      HINTS
        "${_rby1_sdk_py_dir}/lib"
        "${_rby1_sdk_py_dir}/libs"
        "${_python_prefix}/lib"
        "${_python_prefix}/lib64"
    )
  endif()
endif()

if(NOT RBY1_SDK_INCLUDE_DIR)
  message(FATAL_ERROR
    "rby1-sdk headers not found.\n"
    "Pass -DRBY1_SDK_ROOT=<prefix>, install rby1-sdk CMake config, or 'pip install rby1-sdk'.")
endif()
if(NOT RBY1_SDK_LIBRARY)
  message(FATAL_ERROR
    "rby1-sdk shared library not found.\n"
    "Pass -DRBY1_SDK_ROOT=<prefix>, install rby1-sdk CMake config, or 'pip install rby1-sdk'.")
endif()
message(STATUS "rby1-sdk include: ${RBY1_SDK_INCLUDE_DIR}")
message(STATUS "rby1-sdk library: ${RBY1_SDK_LIBRARY}")
# -----------------------------------------------------------------------------

add_library(rby1_robot_client SHARED
  src/rby1_robot_client.cpp
)
target_include_directories(rby1_robot_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${RBY1_SDK_INCLUDE_DIR}"
)
target_link_libraries(rby1_robot_client
  PRIVATE
    "${RBY1_SDK_LIBRARY}"
    Eigen3::Eigen
)

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
target_link_libraries(rby1_system_interface rby1_robot_client)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
