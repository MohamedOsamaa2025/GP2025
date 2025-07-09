#include "Encoder.h"

// Encoder pins
#define ENCRA 19
#define ENCRB 22
#define ENCLA 3
#define ENCLB 2

volatile long right_encoderValue = 0;
volatile long left_encoderValue = 0;

// void readEncoder_left() {
//   int b = digitalRead(ENCLB);
//   left_encoderValue += (b > 0) ? 1 : -1;
// }

// void readEncoder_right() {
//   int b = digitalRead(ENCRB);
//   right_encoderValue += (b > 0) ? 1 : -1;
// }

void readEncoder_left() {

  int b = digitalRead(ENCLB);
 
  if (b > 0) {
    left_encoderValue ++;
 
  }
  else {
    left_encoderValue --;
 
  }
 
 }
 
 void readEncoder_right() {
 
   int b = digitalRead(ENCRB);
 
     if (b > 0) {
       right_encoderValue ++;
 
     }
     else {
       right_encoderValue --;
 
     }
    
  
 }
 

void setupEncoders() {
  pinMode(ENCLA, INPUT_PULLUP);
  pinMode(ENCLB, INPUT_PULLUP);
  pinMode(ENCRA, INPUT_PULLUP);
  pinMode(ENCRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCLA), readEncoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRA), readEncoder_right, RISING);
}

void updateEncoderMsg(geometry_msgs::Point32& msg) {
  msg.x = right_encoderValue;
  msg.y = left_encoderValue;
}
