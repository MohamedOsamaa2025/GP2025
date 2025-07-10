#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "sensor_msgs/Imu.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher imu_pub;
ros::Subscriber imu_raw_sub;
sensor_msgs::Imu imu_data;
ros::Time current_time, last_time;

// Create a Quaternion
tf2::Quaternion quaternion;

double roll = 0.0;  // roll angle
double pitch = 0.0; // pitch angle
double yaw = 0.0;   //  yaw angle

double linear_acc_x = 0.0;  // roll angle
double linear_acc_y = 0.0; // pitch angle
double linear_acc_z = 0.0;   //  yaw angle

void IMUCallback(const std_msgs::Float32MultiArray::ConstPtr &raw_imu_msg)
{

  yaw = raw_imu_msg->data[0];
  pitch = raw_imu_msg->data[1];
  roll = raw_imu_msg->data[2];

  linear_acc_x= 0;
  linear_acc_y=0;
  linear_acc_z=1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;

  imu_raw_sub = nh.subscribe("imu_raw", 50, IMUCallback);
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();

    // Set the quaternion using Euler angles
    quaternion.setRPY(roll, pitch, yaw);

    imu_data.orientation.x = quaternion.x();
    imu_data.orientation.y = quaternion.y();
    imu_data.orientation.z = quaternion.z();
    imu_data.orientation.w = quaternion.w();

    imu_data.linear_acceleration.x=linear_acc_x;
    imu_data.linear_acceleration.y=linear_acc_y;
    imu_data.linear_acceleration.z=linear_acc_z;

    imu_data.header.frame_id = "imu_link";
    current_time = ros::Time::now();

    imu_data.header.stamp = current_time;
    imu_pub.publish(imu_data);
    last_time = current_time;

    loop_rate.sleep();
  }

  return 0;
}