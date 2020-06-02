#!/bin/bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -x sh -c "roslaunch handeye_calib handeye_calib.launch; bash" 
