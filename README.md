<h1 align="center"> Turtlebot Walker - Algorithm for roomba like robot </h1>
<p align="center">
<a href='https://github.com/rohit517/turtlebot_walker/blob/master/LICENSE'><img src='https://img.shields.io/badge/license-MIT-blue.svg'/></a>
</p>

## Overview

Walker algorithm for roomba like cleaning robot on turtlebot

## Dependencies
For running the above package we must have the following dependencies:

- Ubuntu 16.04
- ROS Kinetic 
- Turtlebot package.

ROS Kinetic can be installed by following the instructions given [here](http://wiki.ros.org/kinetic/Installation). 
Once we have ROS installed, we can install turtlebot using the following command
```
sudo apt-get install ros-kinetic-turtlebot-*
```

## Build instructions

We first need to create a catkin workspace. In your directory, run the following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
We then source the setup.bash file from the devel folder.
```
source devel/setup.bash
```
We now clone the turtlebot_walker package in the src folder and build the package.
```
cd src/
git clone --recursive https://github.com/rohit517/turtlebot_walker.git
cd ..
catkin_make
```
## Demo using roslaunch

To run the demo using the launch file, open a terminal and run the following commands
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker walker.launch
```

## Demo using rosrun 

Run the following commands in the terminal opened in your catkin workspace. First we need to start roscore. If you already have
roscore running, you may skip this command.
```
roscore
```

When running using rosrun, we need to launch the turtlebot gazebo environment. In a new terminal run the following command
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

To launch the walker node, open a new terminal in your catkin workspace and run the following commands
```
source devel/setup.bash
rosrun turtlebot_walker walker
```
To close the nodes, press CTRL+C in the terminal.

## Recording bag file with launch file

The launch file can be passed a parameter to record all the topics (except /camera/*) being published as a rosbag file into the results folder. File named turtlebotWalker.bag can be found inside the results folder with a ~20 sec recording. 

To record rosbag, open a new terminal and enter the following commands to launch the walker.launch file. The argument record is used to choose whether to record a bagfile or not. Default is false (record:=false). 
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker walker.launch record:=true
```
This will record all the topics (except /camera/*) and store the bag file in the results folder. Press Ctrl+C to stop recording. 

### Inspect bag file
To inspect the bagfile created in the above step run the following commands
```
cd ~/catkin_ws/src/turtlebot_walker/results
rosbag info turtlebotWalker.bag
```
A sample output can be seen below
```
path:        turtlebotWalker.bag
version:     2.0
duration:    35.3s
start:       Dec 31 1969 19:00:00.22 (0.22)
end:         Dec 31 1969 19:00:35.48 (35.48)
size:        14.5 MB
messages:    29760
compression: none [19/19 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            3527 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               3507 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              3507 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     3509 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     68 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     352 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     3519 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  136 msgs    : bond/Status                           (3 connections)
             /odom                                             3518 msgs    : nav_msgs/Odometry                    
             /rosout                                            379 msgs    : rosgraph_msgs/Log                     (8 connections)
             /rosout_agg                                        364 msgs    : rosgraph_msgs/Log                    
             /scan                                              338 msgs    : sensor_msgs/LaserScan                
             /tf                                               7028 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

```

### Play bag file

The rosbag generated, can be played following the steps below. Start roscore if already not running
```
roscore
```
Next, in a new terminal run the following commands to play the rosbag file.
```
cd ~/catkin_ws/src/turtlebot_walker/results
rosbag play turtlebotWalker.bag
```
To close the rosbag play, press CTRL+C in the terminal.
