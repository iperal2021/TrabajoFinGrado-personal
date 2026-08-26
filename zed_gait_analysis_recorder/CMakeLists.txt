cmake_minimum_required(VERSION 3.8)
project(zed_gait_analysis_recorder)

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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
# ZED SDK 5.2.2 (version fijada por el plan; en zed_node_cpp se usaba 4)
find_package(ZED 5 REQUIRED)

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

add_executable(gait_recorder_node
  src/main.cpp
  src/gait_recorder_node.cpp
  src/recording_session.cpp
  src/backends/zed_body_tracking_backend.cpp
)

ament_target_dependencies(gait_recorder_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
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
