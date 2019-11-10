## Configuration

#### Getting started
Install Fiducial Package:
> $ sudo apt-get install ros-melodic-fiducials

***Check if any of these are included in the launch file before you proceed***

Marker Generation:
> $ rosrun aruco_detect create_markers.py 100 112 fiducials.pdf

Install ros image transport
> $ sudo apt-get install ros-melodic-image-transport

Run Aruco Detect:
> $ roslaunch aruco_detect aruco_detect.launch

<!-- Where does the file come from? -->
Move camera configuration launch file to the raspicam_node folder
> $ sudo scp {location_folder}/camerav2_410x308_30fps_sports.launch {robo}@{robot}.dyn.brandeis.edu:~/camerav2_410x308_30fps_sports.launch

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
Activate 2D-Camera:
> $ roslaunch raspicam_node camerav2_410x308_30fps_sports.launch

#### Run Launch File
Operating mode
> $ roslaunch stalkerbot follow.launch

Debug mode (Robot does not move, instead, teleop is called)
> $ roslaunch stalkerbot test.launch
