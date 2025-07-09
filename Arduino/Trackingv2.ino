#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// Motor pin definitions (Cytron)
const int L_PWM = 10;
const int L_DIR = 11;
const int R_PWM = 8;
const int R_DIR = 9;

// ROS Node Handle
ros::NodeHandle nh;

// Global flag from target_present topic
bool target_present = true;

// Function declarations
void stopMotors();
void leftReverseRightForward();
void leftForwardRightReverse();
void moveMotors();

// Callback for /person_tracking/direction
void directionCallback(const std_msgs::String &msg) {
  if (!target_present) {
    stopMotors();
    return;
  }

  String input = msg.data;

  if (input == "C") {
    stopMotors();  // CENTER behaves like AWAY
  } else if (input == "L") {
    leftReverseRightForward();
  } else if (input == "R") {
    leftForwardRightReverse();
  } else if (input == "A") {
    moveMotors();
  }
}

// Callback for /person_tracking/target_present
void presenceCallback(const std_msgs::Bool &msg) {
  target_present = msg.data;

  if (!target_present) {
    stopMotors();  // Immediately stop if target lost
  }
}

// ROS Subscribers
ros::Subscriber<std_msgs::String> direction_sub("person_tracking/direction", directionCallback);
ros::Subscriber<std_msgs::Bool> presence_sub("person_tracking/target_present", presenceCallback);

void setup() {
  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(direction_sub);
  nh.subscribe(presence_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

// Movement functions
void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void moveMotors() {
  analogWrite(L_PWM, 240); digitalWrite(L_DIR, HIGH);
  analogWrite(R_PWM, 240); digitalWrite(R_DIR, HIGH);
}

void leftReverseRightForward() {
  analogWrite(L_PWM, 240); digitalWrite(L_DIR, LOW);
  analogWrite(R_PWM, 240); digitalWrite(R_DIR, HIGH);
}

void leftForwardRightReverse() {
  analogWrite(L_PWM, 240); digitalWrite(L_DIR, HIGH);
  analogWrite(R_PWM, 240); digitalWrite(R_DIR, LOW);
}
