#!/bin/bash
cd ~/Desktop/robotics-master
catkin_make
source devel/setup.bash
roslaunch ~/Desktop/robotics-master/launch/project1.launch world_file:=$HOME/Desktop/robotics-master/worlds/project1.world
