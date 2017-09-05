drone-load-transportation
=========================
"drone-load-transportaion" is a series of modified ROS packages to implement autonomous control functionality on an AR.Drone 2.0 with a suspended load in order to stabilize it during the flight. It uses the "ardrone_autonomy" driver (from AutonomyLab) and "tum_ardrone" (from tum-vision) for implementing autonomous flight with PTAM-based visual navigation. For estimating the load's position, the package "ar_pose" (from LucidOne) is used. It contains AR Marker tools for ROS based on ARToolKit for publishing pose data (tf) from a camera and a marker. The idea is to integrate all trhee packages in order to obtain a platform capable of load transportation using the AR.Drone bottom camera to estimate its position.

![camera_carga](https://cloud.githubusercontent.com/assets/9382891/5175796/cfcbd64e-7426-11e4-8bfd-dadac481f86e.png)

###Installation (with catkin)
Install external dependencies:

``` bash
sudo apt-get install libv4l-dev
sudo apt-get install qtcreator libgl1-mesa-dev
```
Download the original packages from:

- ardrone_autonomy: https://github.com/tum-vision/ardrone_autonomy

- tum_ardrone: https://github.com/tum-vision/tum_ardrone/tree/indigo-devel

- ar_pose e libArToolKit: https://github.com/srv/ccny_vision

In case the following packages are not included in your ROS installation, get them from:

- rosbag: https://github.com/ros/ros_comm.git

- tf: https://github.com/ros/geometry.git

- rviz: https://github.com/ros-visualization/rviz.git

- dynamic_reconfigure: https://github.com/ros/dynamic_reconfigure.git

After download, unzip the files into the source folder of your workspace and execute:
``` bash
cd catkin_ws/src/<package name>
rosdep install <package name>
```
Redo it for all packages.

Download all the files in this repository [drone-load-transportation](), look into the original packages and replace the existing files with these ones (search for the same file name). Make sure to have a backup of the original files, in case something goes wrong.

for the AutopilotPArams.cfg file, run the following commands after replacing it with the original
``` bash
chom a+x your_package/cfg/AutopilotParams.cfg
```

Then build your workspace.
``` bash
cd catkin_ws
catkin_make
```
If you get errors of undeclared variables or some unespected problem, try to erase all built files and then recompile everything
``` bash
cd catkin_ws/build
rm -r
cd ..
catkin_make clean
catkin_make
```
###Quick Start

First, you need to prepare your drone, for that print the tag (that came with this repository) in a 5x5cm size, cut a piece of fishing line or some other very thin and discrete line of abaout 1.20m, glue the tag on a piece of cardboard and pass the line rigth in the middle of it, só when you hold it by the line, the tag stays leveld and finally use a piece of duct tape to glue it on the lowe part of the drone.

Connect your AR.Drone battery, turn on your wifi, connect to the drone's network and, in separate terminals, launch the nodes:

- ardrone_driver: this may take a few seconds to build. Check the prompt messages for connection failures. The AR.Drone will best perform with full charge.
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ardrone_autonomy
rosmake ardrone_autonomy
roslaunch ardrone_autonomy ardrone.launch
```
If you experience some kind of conflict error, choose one of the two .launch files listed.

- ar_pose: this node will perform the marker pose estimation (assuming you already attached the marker to the suspended load)
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ar_pose
rosmake ar_pose
rosmake rviz rosbag
roslaunch ar_pose ar_pose_single.launch
```

The [rviz](http://ros.org/wiki/rviz) window should pop on the screen, but you will not see the image until the Drone's bottom camera streaming is enabled (later).

- tum_ardrone: this will launch three nodes. Do not proceed without checking [tum_ardrone](https://github.com/tum-vision/tum_ardrone/tree/indigo-devel) for information about the nodes functionalities.
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/tum_ardrone
rosmake tum_ardrone
roslaunch tum_ardrone tum_ardrone.launch
```
Your system is ready. Run the [rqt_graph](http://wiki.ros.org/rqt_graph) tool to make sure you have all nodes running:

``` bash
rosrun rqt_graph rqt_graph
```

![envio_tf](https://cloud.githubusercontent.com/assets/9382891/5175632/c8205962-7424-11e4-894b-19378d03720d.png)

After the first time, all tou should need to do to run the whole system are the following commnads, one in each command line window:
``` bash
roslaunch ardrone_autonomy ardrone.launch
roslaunch ar_pose ar_pose_single.launch
roslaunch tum_ardrone tum_ardrone.launch
```
First to connect to the drone, then open image proc. and tag detection and finally to move the drone around.

###Flying

AR.Drone 2.0 does not allow streaming of both camera images simultaneously, which means that for the load transportation porpuse it is necessary to give up the PTAM functionality. The Drone will still be able to fly autonomously, but its pose estimation will be worse. Let's hope that Parrot changes that in the future. 

In order to disable PTAM:
``` bash
rosrun rqt_reconfigure rqt_reconfigure
```
This will open the [rqt_reconfigure](http://wiki.ros.org/dynamic_reconfigure) screen, which allows the user to dynamically configure node parameters without having to access the source code or stop running it.
> IMPORTANT: requires the node drone_stateestimation to be running.

Select the node [drone_stateestimation](http://wiki.ros.org/tum_ardrone/drone_stateestimation) and uncheck "Use PTAM". For further experiments.

In order to enable the load stabilization controller, select the node [drone_autopilot](http://wiki.ros.org/tum_ardrone/drone_autopilot) and check "Use Load Control". The controller parameters are set for a suspended load with approximately 10% of the Drone's weight, but feel free to experiment.

![UseLoadConfig](https://cloud.githubusercontent.com/assets/9382891/5175752/12f73fea-7426-11e4-89cc-1a1ac70605e5.png)

Open the [drone_gui](http://wiki.ros.org/tum_ardrone/drone_gui#Keyboard_Control) interface, click on "Toggle Cam". Now you should see the bottom image on the rviz screen.



If you see an error on the rviz UI about the camera image, tou shuld try to calibrate both cameras of the ardrone and generate a .yaml file (the steps are listed at the end of this page).

![rviz_screen](https://cloud.githubusercontent.com/assets/9382891/5175742/0266cdf8-7426-11e4-966a-8efcbdc31c14.png)

On the top left box of [drone_gui](http://wiki.ros.org/tum_ardrone/drone_gui#Keyboard_Control) you can either select one of the flight plans included in the package or write your own. Here is a simple flight plan using only control gains:

``` bash
takeoff

goto 0 0 0.7 0

goto 0.8 0.8 0.7 0
goto 0 1.6 0.7 0
goto -0.8 0.8 0.7 0
goto 0 0 0.7 0

land
```
####Procedure
1. Position the Drone on the ground with a lot of free space around it. If you are using PTAM (which means you are not monitoring the load's position), you should have enough key points in front of the Drone. Give preference to furnitured indoor environmnents.
2. Load the flight plan or write one.
3. Click on "Reset" then "Clear and Send"
4. Always be ready to press "Land" in case of imminent crash.

####Recording Flight Data

Create a folder to store data
``` bash
mkdir ~/bagfiles
cd ~/bagfiles
```
Recording messages from all running topics
``` bash
rosbag record -a
```
...or from a specific topic (e.g.: navdata)
``` bash
rosbag record -O subset /ardrone/navdata
```
Verifying content
``` bash
rosbag info subset.bag
```
In order to take the recorded date to other softwares, it is useful to convert the bag file to txt:
``` bash
rostopic echo -b subset.bag -p /ardrone/navdata > output.txt
```

####Adjusting PID controler
First adjust the PID controler for the drone without the load, for that run in 3 separate terminals:

``` bash
roslaunch ardrone_autonomy ardrone.launch
roslaunch tum_ardrone tum_ardrone.launch
rosrun rqt_reconfigure rqt_reconfigure
```
At the rqt_reconfigure window, uncheck the options UsePTAM and UseLoadControl.
We will have to set the PID controler one by one, so set all the gains (Ki_x, Kp_x, Kd_x where X may be rp for roll an pith, yaw for yaw, and gaz for z) to zero.
In this example, we will adjust the PID controller for de yaw(z rotation) of the drone, later you should repeat the process for rp, gaz and load as well.
 
To monitor the variable we want to control, run in other 2 separate terminals:
``` bash
rqt_plot /ardrone/pdictedPose/yaw:x:y:z
rostopic echo /ardrone/predictedPose/yaw
```
The plot is only for visual reference, we will be looking at the rostopich echo for a reference.

Arrange the 4 windows (tum_ardrone GUI, rostopic echo terminal, plot and rqt_reconfigure), in a way that you can see the plot and the numbers for our target variable, send writen commands trough GUI and click on emergency button, and hav access to the 3 variables on rqt_reconfigure (ki, kp and kd). In my case i used a transparent terminal, used half the sceen for the plot and terminal, and the other half split in 2 for the rqt_reconfigure and GUI as shown:

Put the drone in a open space with no wind and far from other objects, then proceed with the commando goto 0 0 0 yaw_value, and ajust the Ki, Kp and Kd gains until you've reched a good combination using your prefered method.


###Camera calibrating

Install the camera calibration and follow the instructions on the link bellow
``` bash
rosdep install camera_calibration
```
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Save the files with the names 'ardrone_bottom.yaml' and 'ardrone_front.yaml' after converting it to yaml as shown in the tutorial, at the /home/[username]/.ros/camera_info/ and try running all programs again.

