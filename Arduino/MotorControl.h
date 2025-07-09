#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

void setupMotors();
void roverCallBack(const geometry_msgs::Twist& cmd_vel);

#endif
