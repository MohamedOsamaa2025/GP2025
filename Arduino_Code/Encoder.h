#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <geometry_msgs/Point32.h>

void setupEncoders();
void updateEncoderMsg(geometry_msgs::Point32& msg);

#endif
