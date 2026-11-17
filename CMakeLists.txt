cmake_minimum_required(VERSION 3.16)
project(origami_ins VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
option(BUILD_ROS2 "Build the ROS 2 interfaces" ON)
include(CTest)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY NAMES tinyxml2 REQUIRED)

add_library(origami_core
  src/common_lib.cpp
  src/so3_math.cpp
  src/eskf.cpp
  src/preprocess.cpp
  src/IMU_Processing.cpp
  src/robot_model.cpp
  src/robot_odometry.cpp
  src/mat_export.cpp
)
target_include_directories(origami_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(origami_core PRIVATE ${TINYXML2_INCLUDE_DIR})
target_link_libraries(origami_core PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json ${TINYXML2_LIBRARY})
target_compile_options(origami_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(origami_ins src/offline_ins.cpp)
target_link_libraries(origami_ins PRIVATE origami_core)

if(BUILD_ROS2)
  find_package(ament_cmake REQUIRED)
  foreach(dependency rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros std_srvs diagnostic_msgs visualization_msgs rosgraph_msgs)
    find_package(${dependency} REQUIRED)
  endforeach()
  add_executable(inertial_odometry src/inertialOdometry.cpp)
  target_link_libraries(inertial_odometry origami_core)
  ament_target_dependencies(inertial_odometry rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros std_srvs diagnostic_msgs visualization_msgs)
  add_executable(imu_replay src/imu_replay.cpp)
  target_link_libraries(imu_replay origami_core)
  ament_target_dependencies(imu_replay rclcpp sensor_msgs geometry_msgs nav_msgs rosgraph_msgs std_srvs)
  if(BUILD_TESTING AND UNIX)
    add_executable(test_ros_interfaces tests/test_ros_interfaces.cpp)
    target_link_libraries(test_ros_interfaces origami_core)
    ament_target_dependencies(test_ros_interfaces rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs rosgraph_msgs)
  endif()
  install(TARGETS inertial_odometry imu_replay DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config rviz_cfg DESTINATION share/${PROJECT_NAME})
  install(FILES data/imu_100hz.txt data/imu_500hz.txt DESTINATION share/${PROJECT_NAME}/data)
endif()

if(BUILD_TESTING)
  add_executable(test_eskf tests/test_eskf.cpp)
  target_link_libraries(test_eskf PRIVATE origami_core)
  add_test(NAME eskf COMMAND test_eskf ${CMAKE_CURRENT_SOURCE_DIR}/config/simulation.json)
  add_executable(test_navigation tests/test_navigation.cpp)
  target_link_libraries(test_navigation PRIVATE origami_core)
  add_test(NAME navigation_regression COMMAND test_navigation ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(navigation_regression PROPERTIES TIMEOUT 180)
endif()

install(TARGETS origami_core origami_ins ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES src/preprocess.h src/IMU_Processing.hpp src/robot_odometry.h DESTINATION include/${PROJECT_NAME})
if(BUILD_ROS2)
  ament_export_include_directories(include/${PROJECT_NAME})
  ament_export_libraries(origami_core)
  ament_export_dependencies(Eigen3 nlohmann_json)
  ament_package()
endif()
