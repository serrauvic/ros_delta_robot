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


void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  //Serial.print("FROM SUPPORT PROJECT");
  servo_j1.write(cmd_msg.data[0]); //set servo angle, should be from 0-180

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("trajectory_angles", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo_j1.attach(9); //attach it to pin 9
  servo_j2.attach(10); //attach it to pin 10
  servo_j3.attach(11); //attach it to pin 11
}

void loop(){
  nh.spinOnce();
  delay(1);
}
