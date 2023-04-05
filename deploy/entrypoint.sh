#!/bin/bash
export DISPLAY=:20
Xvfb :20 -screen 0 1920x1080x16 &
x11vnc -passwd 1234 -display :20 -N -forever -rfbport 21800 &

source /ros/catkin_ws/devel/setup.sh && \
    roslaunch spot_navigation start_mapping.launch username:=admin password:=$SPOT_PASSWORD real_data:=$SPOT_REAL_DATA use_apriltag:=$USE_APRILTAG
