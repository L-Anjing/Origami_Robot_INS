cmake_minimum_required(VERSION 3.16)
project(origami_ins)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the existing NumPy core, so installed nodes also work outside this checkout.
ament_python_install_package(ins PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ins")
ament_python_install_package(origami_ins)
install(PROGRAMS scripts/imu_replay scripts/inertial_odometry
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES ../../config.json DESTINATION share/${PROJECT_NAME}/config)
install(FILES ../../data/imu_100hz.txt ../../data/imu_500hz.txt
  DESTINATION share/${PROJECT_NAME}/data)
ament_package()
