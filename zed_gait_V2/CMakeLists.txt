cmake_minimum_required(VERSION 3.8)
project(zed_gait_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(ZED 5 REQUIRED)

# NOTA: src/backends/, src/recording_session.cpp y sus headers se conservan
# en disco SIN compilar: son de la fase 2 (procesado offline SVO -> CSV con
# body tracking). Recuperables tambien en el paquete zed_gait_analysis_recorder.
add_executable(gait_recorder_node
  src/main.cpp
  src/gait_recorder_node.cpp
)

include_directories(
  include
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
)

ament_target_dependencies(gait_recorder_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
)

target_link_libraries(gait_recorder_node
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

install(TARGETS
  gait_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
