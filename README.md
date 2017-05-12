# Robotics Project
- Jordan Nguyen
- Alex Salazar
- Nicholas Von Busch
- Tyler Woodfin
- Koby Pascual

# To Run (Project 1)
- Clone or download repository and change into that directory.
- Execute the following command: `$ ./run_me.sh`
- Gazebo should open. If it does not, run the command again.
- In a new terminal window, execute the following command: `$ ./run_me2.sh`
- The project should now run.

# To Run (Project 2)
- Clone or download repository and change into that directory.
- Execute the following command: `catkin_make`
- Then execute the following command: `source devel/setup.bash`
- SSH into turtlebot and roslaunch the minimual bringup and 3d bringup
- In a new terminal, execute the following command: `roslaunch kame_project project2.launch`

# To Run (Tour Project)
- Clone or download repository and change into that directory.
- Then execute the following command: `source devel/setup.bash`
- Execute the following command: `roslaunch kame_project tour.launch` this sets up the world, map and rviz
- In a new terminal repeat step 2, then execute: `rosrun kame_project tour.py`

# Troubleshooting
Please ensure:
- roscore is running properly
- you are in the the correct directory (**robotics**: for clone; **robotics-master**: for download)

If you continue to experience problems, please open *run_me* and *run_me2* in your preferred text editor and execute the commands in this file in the terminal, one at a time.

## Have a lovely day! :smile:
