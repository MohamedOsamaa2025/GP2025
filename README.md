#  GP2025-ECE-Autonomous_Shopping_Cart

This project presents an autonomous smart shopping cart designed to enhance the in
store experience by combining AI, robotics, embedded systems, and IoT protocols. The 
cart can follow a selected customer in real time using Kinect-based depth sensing and a 
YOLOv3 + DeepSORT tracking model running on a GPU-powered PC. Motion commands 
are sent to an Arduino Mega that drives the motors based on encoder feedback. Product 
recognition is performed live using an ESP32-CAM and a YOLOv8-OBB model, with results 
displayed instantly in a Flutter mobile app. The app also supports person ID input, live 
video streaming, product list display, and a return-to-parking feature. All components 
communicate over an offline Wi-Fi network using MQTT, WebSocket, and ROS, forming a 
fully modular, internet-independent system for contactless, intelligent shopping.

![image](https://github.com/user-attachments/assets/22de16a3-41b7-4d9a-8745-747235187f0e)
