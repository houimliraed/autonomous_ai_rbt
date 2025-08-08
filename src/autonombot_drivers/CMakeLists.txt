cmake_minimum_required(VERSION 3.5)
project(autonombot_drivers)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Python packages only - no C++ executables

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/noisy_controller.py
  ${PROJECT_NAME}/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()