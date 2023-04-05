#!/bin/bash

if [ "$START_VNC_SERVER" == "true" ]
    export DISPLAY=:20
    Xvfb :20 -screen 0 1920x1080x16 &
    x11vnc -passwd 1234 -display :20 -N -forever -rfbport 21800 &
fi

source /ros/catkin_ws/devel/setup.sh && \
    roslaunch spot_navigation start_mapping.launch username:=$SPOT_USERNAME password:=$SPOT_PASSWORD \
    real_data:=$SPOT_REAL_DATA use_apriltag:=$USE_APRILTAG delete_db:=$DELETE_DB_ON_START
