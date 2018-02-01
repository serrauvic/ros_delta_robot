## ROS delta robot.

* delta_robot_support
Project containing global launch files,URDFs and meshes.

* delta_robot_kinematics
Package kinematics library

* delta_robot_img_processor
ROS image processor for detecting circles.

* delta_robot_firmware
Arduino src code

* delta_robot_drivers
Delta robot driver and kinematic solver.


* How to launch

* Load firmware to arduiono UNO

catkin_make delta_robot_firmware_src_delta_arduino-upload

roslaunch delta_robot_support delta_robot_sim.launch

roslaunch delta_robot_support delta_robot_real.launch

[![Assembly test one](https://img.youtube.com/vi/qOeS8mAfZcc/0.jpg)](https://www.youtube.com/watch?v=qOeS8mAfZcc)
[![Assembly test two](https://img.youtube.com/vi/pI4uqf1SEYo/0.jpg)](https://www.youtube.com/watch?v=pI4uqf1SEYo)

