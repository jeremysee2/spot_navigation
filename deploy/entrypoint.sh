#!/bin/bash
export DISPLAY=:20
Xvfb :20 -screen 0 1366x768x16 &
x11vnc -passwd 1234 -display :20 -N -forever -rfbport 21900 &

source /ros/catkin_ws/devel/setup.sh && \
    roslaunch spot_navigation start_mapping.launch username:=admin password:=$SPOT_PASSWORD real_data:=$SPOT_REAL_DATA & roslaunch spot_navigation view_map.launch
