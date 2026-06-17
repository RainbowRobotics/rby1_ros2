cmake_minimum_required(VERSION 3.22)
project(rby1_moveit_m_1_2)

find_package(ament_cmake REQUIRED)

include_directories(
  ${RBY1_SDK_PATH}/build/private
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
