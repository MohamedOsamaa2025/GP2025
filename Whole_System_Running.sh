#!/bin/bash
# STEP 1: Launch websocket_bridge.py
echo "[INFO] Launching websocket_bridge.py..."
gnome-terminal --tab --title="websocket_bridge" -- bash -c "
rosrun mqtt_bridge websocket_bridge.py
exec bash
"
# STEP 2: Launch Kinect node on Raspberry Pi
echo "[INFO] Connecting to Raspberry Pi & launching Kinect node..."
gnome-terminal --tab --title="Kinect on Pi" -- bash -c "
sshpass -p 'cart2025' ssh -tt carttt@192.168.0.12 << 'ENDSSH'
echo '[INFO] Logged into Raspberry Pi'
roslaunch freenect_launch freenect.launch
echo '[INFO] Terminal will remain open...'
bash -i
ENDSSH
exec bash
"
sleep 15

# STEP 3: Launch person_tracker on local machine
echo "[INFO] Launching person_tracker..."
gnome-terminal --tab --title="person_tracker" -- bash -c "
roslaunch person_tracking person_tracker.launch
exec bash
"
sleep 5
# STEP 4: Launch target_ID.py
echo "[INFO] Launching target_ID.py..."
gnome-terminal --tab --title="target_ID" -- bash -c "
rosrun mqtt_bridge target_ID.py
exec bash
"
# STEP 5: Run Arduino serial node on Raspberry Pi
echo "[INFO] Running Arduino serial node..."
gnome-terminal --tab --title="Arduino Serial on Pi" -- bash -c "
sshpass -p 'cart2025' ssh -tt carttt@192.168.0.12 << 'ENDSSH'
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
echo '[INFO] Terminal will remain open...'
bash -i
ENDSSH
exec bash
"
sleep 10

# STEP 6: Run motor tracking code on Raspberry Pi
echo "[INFO] Running motor tracking code on Raspberry Pi..."
gnome-terminal --tab --title="Motor Tracking on Pi" -- bash -c "
sshpass -p 'cart2025' ssh -tt carttt@192.168.0.12 << 'ENDSSH'
rosrun target_depth target.py
echo '[INFO] Terminal will remain open...'
bash -i
ENDSSH
exec bash
"
