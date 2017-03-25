#!/bin/bash
cd $HOME/robotics
catkin_make
source devel/setup.bash
roslaunch project1 project1.launch
