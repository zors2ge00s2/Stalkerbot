## Introduction

stalkerbot is an ROS application which runs on the turtlebot3.
The application takes camera and lidar sensory data and detects fiducial / aruco markers and follow the target accordingly.

Status: Maintained by Dan and Max

## Getting Started

#### Package Installation
Install Fiducial Package:
> $ sudo apt-get install ros-melodic-fiducials

Install PyYAML:
> $ pip install PyYAML --ignore-installed PyYAML

Install ros image transport
> $ sudo apt-get install ros-melodic-image-transport

***Check if any of these are included in the launch file before you proceed***

Run Aruco Detect: (Already included in all of our launch files)
> $ roslaunch aruco_detect aruco_detect.launch

Marker Generation:
> $ rosrun aruco_detect create_markers.py 100 112 fiducials.pdf

Move camera configuration launch file to the raspicam_node folder
> $ sudo scp stalkerbot/miscellaneous/camerav2_410x308_30fps_sports.launch {robo}@{robot}.dyn.brandeis.edu:~/camerav2_410x308_30fps_sports.launch

Then from raspberrypi of the robot, run
> $ sudo scp ~/camerav2_410x308_30fps_sports.launch /opt/ros/kinetic/share/raspicam_node/launch/camerav2_410x308_30fps_sports.launch

> $ rm ~/camerav2_410x308_30fps_sports.launch

Bring up the camera panel:
> $ rqt_image_view

#### Install Kinect One Camera:
Edit /.bashrc with
> $ export TURTLEBOT_3D_SENSOR=kinect

## Run Stalkerbot

#### Activate Camera
Activate 2D-Camera (via ssh):
> $ roslaunch raspicam_node camerav2_410x308_30fps_sports.launch

#### Run Launch File
Operating mode
> $ roslaunch stalkerbot advanced_follow.launch

Debug mode (Robot does not move, instead, teleop is called)
> $ roslaunch stalkerbot stationary.launch

## Configuration

#### Camera configuration

Move camera configuration launch file to the raspicam_node folder
> $ sudo scp stalkerbot/miscellaneous/camerav2_410x308_30fps_sports.launch {robo}@{robot}.dyn.brandeis.edu:~/camerav2_410x308_30fps_sports.launch

Then from raspberrypi of the robot, run
> $ sudo scp ~/camerav2_410x308_30fps_sports.launch /opt/ros/kinetic/share/raspicam_node/launch/camerav2_410x308_30fps_sports.launch

> $ rm ~/camerav2_410x308_30fps_sports.launch

Finally, to check if the camera is working, bring up the camera panel:
> $ rqt_image_view

Things I had to do to get move_base running right
> $ roscd turtlebot3_navigation/param

Open global_costmap_param.yaml  
Change/add these parameters:  
static_map: false  
width: 40.0  
height: 40.0  
origin_x: -10.0  
origin_y: -10.0  

Go to turtlebot3_navigation  
Open the move_base.launch file  
Change move_forward_only to true  

> $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping


> $ roslaunch turtlebot3_navigation move_base.launch

#### config.yaml file
The file is a centralized method to store and change constants in our algorithm when necessary.
Helpful reading: https://martin-thoma.com/configuration-files-in-python/#yaml
