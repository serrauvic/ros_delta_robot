cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_support)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  urdf
  std_msgs
#  joint_trajectory_action
  trajectory_msgs
#  pr2_controllers_msgs
  position_controllers
  joint_state_controller
  roscpp
  urdf
  rosserial_arduino
  rosserial_client
  std_msgs)

catkin_package(
)

###########
## Build ##
###########
include_directories(include
	${catkin_INCLUDE_DIRS}
)
