## Configuration

#### Getting started
Install Fiducial Package:
> $ sudo apt-get install ros-melodic-fiducials

***Check if any of these are included in the launch file before you proceed***

Marker Generation:
> $ rosrun aruco_detect create_markers.py 100 112 fiducials.pdf

Run Aruco Detect:
> $ roslaunch aruco_detect aruco_detect.launch

Activate 2D-Camera:
> $ roslaunch raspicam_node camerav2_410x308_30fps_sports.launch

Bring up the camera panel:
> $ rqt_image_view

#### Install Kinect One Camera:
Edit /.bashrc with
> $ export TURTLEBOT_3D_SENSOR=kinect


