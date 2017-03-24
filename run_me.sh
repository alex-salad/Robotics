#!/bin/bash
cd ~/robotics
catkin_make
source devel/setup.bash
roslaunch ~/robotics/launch/project1.launch world_file:=$HOME/robotics/worlds/project1.world
