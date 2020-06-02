#!/bin/bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -x sh -c "roslaunch cam_calib cam_calib.launch; bash" 
