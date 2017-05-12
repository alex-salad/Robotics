#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h" 

// goal estimates
double XGoal1 = 2.53566564977;
double YGoal1 = -0.773420049965;

double XGoal2 = 4.49860523334;
double YGoal2 = 0.348575832826;

double XGoal3 = 2.33846501217;
double YGoal3 = 2.30097446639;

double XGoal4 = -0.0342952696324;
double YGoal4 = 1.04674247832;

bool moveToGoal(double X, double Y) {
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

    goal.target_pose.pose.position.x =  X;
    goal.target_pose.pose.position.y =  Y;
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
       return true;
    }
    else{
       ROS_INFO("The robot failed to reach the destination");
    }
    return false;
}

void localize(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    ROS_INFO("Position: x = %f, y = %f", x, y);

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "tour");
    ros::NodeHandle n;
       
    // wait for everything to be set up first
    ros::Duration(5.0).sleep();
    
    ros::Subscriber detect_sub = n.subscribe("amcl_pose", 10, localize);
    
    ros::spin();
    
    return 0;
}

