#!/bin/bash
source /ros/catkin_ws/devel/setup.sh && \
    roslaunch spot_navigation start_mapping.launch username:=admin password:=$SPOT_PASSWORD real_data:=$SPOT_REAL_DATA
