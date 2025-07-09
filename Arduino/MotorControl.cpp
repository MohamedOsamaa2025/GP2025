#include "MotorControl.h"

// Motor pins
#define PWM_LEFT 10
#define DIR_LEFT 11
#define PWM_RIGHT 8
#define DIR_RIGHT 9

// Constants
#define L 0.33
#define R 0.0325
#define MAX_SPEED 16
#define MAX_PWM 255
#define LED 13

int VelocityToPWM(float velocity) {
  float abs_velocity = abs(velocity);
  int pwm = (int)(abs_velocity / MAX_SPEED * MAX_PWM);
  return constrain(pwm, 0, 255);
}

void setMotor(int pwmPin, int dirPin, float speed) {
  bool direction = (speed >= 0);
  int pwm_value = VelocityToPWM(speed);
  digitalWrite(dirPin, direction ? HIGH : LOW);
  analogWrite(pwmPin, pwm_value);
}

void roverCallBack(const geometry_msgs::Twist& cmd_vel) {
  float vel = constrain(cmd_vel.linear.x, -0.3, 0.3);
  float omega = constrain(cmd_vel.angular.z, -1.5, 1.5);

  float left_speed = (2 * vel - omega * L) / (2 * R);
  float right_speed = (2 * vel + omega * L) / (2 * R);

  setMotor(PWM_LEFT, DIR_LEFT, left_speed);
  setMotor(PWM_RIGHT, DIR_RIGHT, right_speed);

  digitalWrite(LED, (vel != 0 || omega != 0) ? HIGH : LOW);
}

void setupMotors() {
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(LED, OUTPUT);
}
