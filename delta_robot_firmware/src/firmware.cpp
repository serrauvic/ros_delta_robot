/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 *
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/UInt16MultiArray.h"

ros::NodeHandle  nh;

Servo servo_j1;
Servo servo_j2;
Servo servo_j3;

std_msgs::UInt16MultiArray angles_;

void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){

  servo_j1.write(cmd_msg.data[0]); //set servo angle, should be from 0-180
  servo_j2.write(cmd_msg.data[1]); //set servo angle, should be from 0-180
  servo_j3.write(cmd_msg.data[2]); //set servo angle, should be from 0-180

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("/delta_robot/arduino_cmd", servo_cb);
//ros::Publisher pub_angle_("/arduino/angles", &angles_);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub_angle_);

  servo_j1.attach(9); //attach it to pin 9
  servo_j2.attach(10); //attach it to pin 10
  servo_j3.attach(11); //attach it to pin 11
}

void loop(){
  //pub_angle_.publish(&angles_);
  nh.spinOnce();
  delay(1);
}
