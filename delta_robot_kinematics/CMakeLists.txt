cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_kinematics)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS
      roscpp
      rospy)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      INCLUDE_DIRS
            include
      LIBRARIES
            delta_kinematics
      CATKIN_DEPENDS
            roscpp
            rospy)

## Specify additional locations of header files
include_directories(
  include
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(delta_kinematics
 src/delta_kinematics.cpp
)
