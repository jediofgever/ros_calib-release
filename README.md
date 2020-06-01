## Aim
This repository contains ROS packages, which are to be used to calibrate;
* extrinsics,  Hand-in-Eye or Eye-in-Hand.   
* intrinsics,  parameters of a 2D camera. 

## Problem Statement for Hand-Eye or Eye-in-Hand calibration

The camera can be either; 
* attached to Robot's End-effector (Eye in Hand) 
* fixed to a certain postion with respect to Robots base frame (Hand in Eye)

In first case we try to find the rigid body transfrom matrix that would transform a 6DOF pose in camera frame to End-Effector frame (Eye in Hand). While in second case we try to find the rigid body transfrom matrix that would transform a 6DOF pose in camera frame to Robot Base frame (Hand in Eye).
A depiction of this two cases can be seen in the following figure;

![.](resources/cam_calib.png)

## handeye_calib

This package can be configured to find both cases mentioned above. under the handeye_calib/cfg/config.yaml find eye_in_hand parameter and configure for the type of setup you use. The default value is set to true for case of eye_in_hand.

We use MoveIt configured 6DOF robotic arm, other parameters under the handeye_calib/cfg/config.yaml should be adjusted to your robots MoveIt configuration, but these parameters are mostly the same for moveit configured robots. The parameters names are descriptive enough to adress their values. In the defualt setup they are as; 

robot_base_frame: "base_link"
robot_effector_frame: "link_6"
tracking_base_frame: "camera_link"
tracking_marker_frame:  "camera_marker"

### Aruco Tracker for handeye_calib
Obviously we need a marker that we can detect its 6DOF pose and track it in camera frame. handeye_calib uses Aruco tracker for this purpose. In the launch file of handeye_calib you will find parameters configured for Aruco tracker and its ROS node call. Generate a aruco marker here; https://chev.me/arucogen/ , care about its size after you print it to a A4. You will need to enter the aruco marker ID and its size to correct places handeye_calib.launch
Refer to Installation section for installing Aruco asa aROS package , binary install reccomended.

### Type of Camera that You use for handeye_calib
I have used realsense d435 camera and in the launch file you will see that I start realsense camera,
but any RGB camera that can be run with ROS should be usable. Though the topic names and coordinate frames should be double chechked when setting the parameters. in config.yaml and handeye_calib.launch files. 

Replace the Realsense part in handeye_calib.launch, if you are using another camera 
### Running the handeye_calib
For handeye_calib to run , you will need to make sure you have bringed up the robot, and a TF stream is available for the TF listener, This typically is achieved by initilizing the Moveit generated launch files , e.g move_group launch file. 

![.](resources/robot.png)

To start handeye_calib; 

> roslaunch handeye_calib calib.launch

A QT popup should be displayed whowing you the setup of the marker and camera as; 

![.](resources/handeye_start.png)


