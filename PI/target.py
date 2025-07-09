#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

# Define angle thresholds
LEFT_THRESHOLD = -10
RIGHT_THRESHOLD = 10

# Variable to store latest target angle
target_angle = 0.0
last_direction = None  # To detect direction change

# Publisher will be assigned later
direction_pub = None

def angle_callback(msg):
    global target_angle
    target_angle = msg.data

def depth_callback(depth_msg):
    global last_direction, direction_pub
    try:
        # Convert raw depth image (16UC1) to NumPy array
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape((depth_msg.height, depth_msg.width))
        
        # Get center pixel depth (in mm, convert to meters)
        center_pixel = depth_image[depth_image.shape[0] // 2, depth_image.shape[1] // 2]
        depth_in_meters = center_pixel / 1000.0

        direction = determine_direction(target_angle, depth_in_meters)

        if direction != last_direction:
            last_direction = direction
            rospy.loginfo(f"Target Angle: {target_angle}°, Depth: {depth_in_meters:.2f} m → Direction: {direction}")

            if direction_pub is not None:
                direction_pub.publish(direction)

    except Exception as e:
        rospy.logerr(f"Error processing depth image: {e}")

def determine_direction(angle, depth):
    if (angle < LEFT_THRESHOLD):
        return "L"
    elif (angle > RIGHT_THRESHOLD)):
        return "R"
    elif depth < 1.5:
        return "C"
    else:
        return "A"

if __name__ == "__main__":
    rospy.init_node("target_angle_depth_tracker")

    direction_pub = rospy.Publisher("/person_tracking/direction", String, queue_size=1)

    rospy.Subscriber("/person_tracking/target_angle", Float32, angle_callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)

    rospy.spin()
