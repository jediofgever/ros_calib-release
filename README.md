# Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
    + [ROS Melodic](#ros-melodic)
    + [MoveIt](#moveit)
    + [QT5 Default](#qt5-default)
    + [vision_visp](#vision-visp)
    + [visp](#visp)
    + [Aruco](#aruco)
    + [ros_calib](#ros-calib)
    + [Build anmd Source](#build-anmd-source)
- [Aim](#aim)
- [Problem Statement for Hand-Eye or Eye-in-Hand calibration](#problem-statement-for-hand-eye-or-eye-in-hand-calibration)
- [handeye_calib](#handeye-calib)
  * [Aruco Tracker for handeye_calib](#aruco-tracker-for-handeye-calib)
  * [Type of Camera that You use for handeye_calib](#type-of-camera-that-you-use-for-handeye-calib)
  * [Running the handeye_calib](#running-the-handeye-calib)
- [cam_calib](#cam-calib)
  * [Type of Camera that You use for cam_calib](#type-of-camera-that-you-use-for-cam-calib)
  * [Running the cam_calib](#running-the-cam-calib)
- [common lib and its utilities](#common-lib-and-its-utilities)

## Prerequisites

* ROS Melodic
* Moveit (Binary Install);
* QT5 Default from Ubuntu apt-get
* vision_visp
* visp
* Aruco Tracker for handeye_calib
* YOUR CAMERAS ROS PACKAGE

## Installation
#### ROS Melodic
  Installation [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
#### MoveIt 

> sudo apt-get install ros-melodic-moveit*

#### QT5 Default
> sudo apt-get install build-essential

> sudo apt-get install qtcreator

> sudo apt-get install qt5-default

#### vision_visp
> sudo apt-get install ros-melodic-vision-visp 

#### visp
> sudo apt-get install ros-melodic-visp

#### Aruco
clone aruco to your catkin_ws/src 
> cd catkin_ws/src

> git clone https://github.com/pal-robotics/aruco_ros


#### ros_calib
clone this repository to your workspace

> cd catkin_ws/src

> git clone https://github.com/jediofgever/ros_calib.git

#### Build anmd Source 
satisfy package dependencies if any 

> rosdep install --from-paths src --ignore-src --rosdistro melodic

> cd catkin_ws

> catkin_make 

> source devel/setup.bash


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

* robot_base_frame: "base_link"
* robot_effector_frame: "link_6"
* tracking_base_frame: "camera_link"
* tracking_marker_frame:  "camera_marker"

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

Click OK if your setup is similar, After pressing ok The handeye_calib will generate around 50 random poses around the marker and these poses will be executed respectively. Note that this random poses will not approach the marker(z direction) , random poses will vary only in x-y plane and they are symetrical. 

![.](resources/handeye_calib_run.png)
At Left you will be able to see the resulting aruco marker detection for debugging purposes. 

on The right side; The Arrows surronding the Robot end-effector, are actually 6DOF poses randomly generated by the utility class in common library,  the robot will plan to each of this poses and take a sample for the calibration. The blue line is current planned path. PointCloud is only for visualization puposes, it is not needed to perform this calibration.

Note that this process is fully autmatic and once started it does not need any more interaction with the user.

After after each pose the calibration will be calculated and a .yaml file will be dumped to calibration_path that you set in config.yaml file

## cam_calib

This package can be used to calibrate intrinsics of camera. Again this package is based on vision_visp, a marker that consists of circles is used,  
the marker can be found here;
 https://github.com/lagadic/vision_visp/blob/master/visp_camera_calibration/launch/images/grid2d.pdf
 print the marker in 1:1 scale, measure to make sure the linear distance between center of circles is 30mm. The marker should be on a flat and rigid surface to avoid distortions. 

### Type of Camera that You use for cam_calib
Again I tested cam_calib with realsense d435 camera but it is no big deal to use another camera as long as you make it run with ROS. In the ros_calib/cam_calib/cfg/config.yaml file change/check that topic names are matching to your camera's; 

for example in realsense following topics are used;

* camera_image_topic_name: "/camera/color/image_raw"
* set_camera_info_service_topic_name: "/camera/color/set_camera_info"
* calibration_path: "/home/atas/calibration.ini"

### Running the cam_calib
Just like handeye_calib , once we run this package a set of randomly generated poses be qued, After arriving at each pose the user will be asked to select the keypoints 1-2-3-4. After selected keypoints , the user should click mouse-left and the robot will be moved to next pose, thisprocess will take place until all generated poses are visited, finally a .ini file will dumped to calibraion_path that you set in cfg/config.yaml file. 

At Each pose the shot will be taken and the user will be asked to click on center of keypoints 1-2-3-4. 
click on the center of key points after selected all 4 points  click left to go to next pose
![.](resources/cam_calib_select.png)

the remaining circles will uatomaically be detected by VISP.
![.](resources/cam_calib_after_select.png)

## common lib and its utilities
If you read until here carefully  you might have noticed there is some magical things going on , such as random generated poses, planning to this poses, plan execution etc. 

This functions are provided by common lib , RandomPoseGenerator class of common lib is responsible to create poses surronding robot end-effector. RobotController class is responsible to make a plan and move robot along the path planned. A visualization of plan in blue color is popped up in RVIZ at each plan execution.
