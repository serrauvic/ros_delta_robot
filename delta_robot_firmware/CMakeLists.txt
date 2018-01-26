cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_firmware)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY src
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(src delta_arduino ALL)
rosserial_add_client_target(src delta_arduino-upload)
