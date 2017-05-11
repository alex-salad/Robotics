#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// sample goals
double XGoal1 = 1.83026990584;
double YGoal1 = 1.80745018798;

double XGoal2 = -4.64742646512;
double YGoal2 = -0.524048690996;

int main (int argc, char **argv) {
    ros::init(argc, argv, "tour");
    ros::NodeHandle n;
    
    // wait for everything to be set up first
    ros::Duration(5.0).sleep();
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

   // moving towards the goal

    goal.target_pose.pose.position.x =  XGoal1;
    goal.target_pose.pose.position.y =  YGoal1;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal location ...");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
       ROS_INFO("You have reached the destination");
    }
    else{
       ROS_INFO("The robot failed to reach the destination");
    }

    return 0;
}

