cmake_minimum_required(VERSION 2.8.3)

include_directories(
	${ROS_LIB_DIR}
)

# add_definitions(-DUSB_CON)

print_board_list()
print_programmer_list()

# Something is wrong that it requires the time.cpp for time conversions
generate_arduino_firmware(delta_arduino
  SRCS firmware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
  PROGRAMMER avrispmkii
)
