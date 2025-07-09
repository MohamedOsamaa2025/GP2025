#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <MPU6050.h>
#include <sensor_msgs/Range.h>

// Include your motor and encoder headers
#include "MotorControl.h"
#include "Encoder.h"

// --- ROS NodeHandle ---
ros::NodeHandle nh;

// --- IMU Publisher ---
std_msgs::Float32MultiArray imu_raw_msg;
ros::Publisher imu_raw_pub("imu_raw", &imu_raw_msg);

// --- Encoder Publisher ---
geometry_msgs::Point32 enc_msg;
ros::Publisher enc_pub("/encoder", &enc_msg);

// --- Cmd_vel Subscriber ---
geometry_msgs::Twist cmd_msg;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", roverCallBack);

// --- MPU6050 instance ---
MPU6050 mpu;

// Timing for IMU
unsigned long last_imu_time = 0;
float dt = 0;

// Orientation estimation variables for IMU complementary filter
float gyroXangle = 0, gyroYangle = 0, gyroZangle = 0;
float compAngleX = 0, compAngleY = 0;
float yaw = 0.0, pitch = 0.0, roll = 0.0;
const float alpha = 0.96;

// Ultrasonic Pins
const int trigPin = 12;
const int echoPin = 13;

// ROS Range Message
sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasonic", &range_msg);


void setup() {
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    // Handle error: stay stuck here
    while (1);
  }

  // Setup motors and encoders
  setupMotors();
  setupEncoders();

  // ROS init and advertise/subscribe
  nh.initNode();

  imu_raw_msg.data_length = 3;  // yaw, pitch, roll
  imu_raw_msg.data = new float[3];
  nh.advertise(imu_raw_pub);
  
  nh.advertise(enc_pub);
  nh.subscribe(sub);

  last_imu_time = millis();

  nh.advertise(pub_range);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up the Range message
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = "ultrasonic_link";  // Must match TF frame
  range_msg.field_of_view = 0.26;  // ~15 degrees in radians (adjust as needed)
  range_msg.min_range = 0.02;      // 2 cm (HC-SR04 min range)
  range_msg.max_range = 4.0;       // 4 meters (HC-SR04 max range)
  
}

void loop() {
  nh.spinOnce();

  unsigned long now = millis();
  dt = (now - last_imu_time) / 1000.0f;
  last_imu_time = now;

  // --- Read MPU6050 data ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = (float)ax / 16384.0f;
  float accelY = (float)ay / 16384.0f;
  float accelZ = (float)az / 16384.0f;

  float gyroX = (float)gx / 131.0f;  // deg/s
  float gyroY = (float)gy / 131.0f;
  float gyroZ = (float)gz / 131.0f;

  // Calculate pitch and roll from accelerometer (deg)
  float pitchAcc = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float rollAcc = atan2(-accelX, accelZ) * 180 / PI;

  // Integrate gyro data
  gyroXangle += gyroX * dt;
  gyroYangle += gyroY * dt;
  gyroZangle += gyroZ * dt;

  // Complementary filter
  compAngleX = alpha * (compAngleX + gyroX * dt) + (1 - alpha) * rollAcc;
  compAngleY = alpha * (compAngleY + gyroY * dt) + (1 - alpha) * pitchAcc;

  // Yaw angle from gyroZ integration (no magnetometer correction)
  yaw += gyroZ * dt;

  // Convert to radians for publishing
  roll = compAngleX * PI / 180.0f;
  pitch = compAngleY * PI / 180.0f;
  yaw = yaw * PI / 180.0f;

  imu_raw_msg.data[0] = yaw;
  imu_raw_msg.data[1] = pitch;
  imu_raw_msg.data[2] = roll;

  imu_raw_pub.publish(&imu_raw_msg);

  
  // Trigger ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse duration
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.000344 / 2;  // Convert to meters (speed of sound ~344 m/s)

  // Update and publish Range message
  range_msg.range = distance;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);

  


  // --- Encoder publishing ---
  updateEncoderMsg(enc_msg);
  enc_pub.publish(&enc_msg);
  
  nh.spinOnce();
  delay(10);
}
