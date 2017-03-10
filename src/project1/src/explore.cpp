// imports =================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <boost/thread/thread.hpp>

// constants ===============================
#define TURN_ANGLE M_PI / 12
#define SPEED 0.2

// global variable
double distance_counter = 0;
bool canKeyboard = true;
bool canEscape = true;
bool canAvoid = true;
bool canTurn = true;
bool canDrive = true;

/**
* Publishes commands for rotating 15 degrees in a random direction.
* Commands are published whenever 1 meter has approximately been traveled.
*/
void turn() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // publisher for sending move commands to gazebo
    ros::Publisher turn_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);
    
    // the move to be published: TURN ONLY
    geometry_msgs::Twist turn_cmd;
    // should take two seconds to turn
    double angular_vel = TURN_ANGLE / 2;    
    turn_cmd.linear.x = 0;
    turn_cmd.linear.y = 0; 
    // randomly determine which direction to turn   
    turn_cmd.angular.z = (rand() % 2) ? angular_vel : -1 * angular_vel;
    
    // run at 5Hz
    ros::Rate loop_rate(5);
    
    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();
        
        // turn after 1 meter has been traveled
        if (distance_counter >= 1) {
            canDrive = false;
            double current_angle = 0;
            double start_time = ros::Time::now().toSec();
            // publish command for turning
            while (ros::ok() && canTurn && current_angle < TURN_ANGLE) {
                turn_pub.publish(turn_cmd);
                ros::spinOnce();
                current_angle =  angular_vel * ((ros::Time::now().toSec()) - start_time);
                loop_rate.sleep();
            }
            // reset distance counter
            distance_counter = 0;
            canDrive = true;
        }
        loop_rate.sleep();
    }
}

/**
* Publishes commands for traveling in a straigh line for one meter.
*/
void drive() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // publisher for sending move commands to gazebo
	ros::Publisher drive_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);

    // the move to be published: STRAIGHT ONLY
	geometry_msgs::Twist move_cmd;
	move_cmd.linear.x = SPEED;
	move_cmd.linear.y = 0;
	move_cmd.angular.z = 0;
	
	// run at 5Hz
	ros::Rate loop_rate(5);
    
    // enters loop until ros quits
	while (ros::ok()) {
		double start_time = ros::Time::now().toSec();
		double current_distance = 0;		
		// publishes command for moving 1 meter at a time
		while (ros::ok() && canDrive && current_distance < 1) {
			drive_pub.publish(move_cmd);
			ros::spinOnce();
			// calulate approximate distance traveled
			current_distance = SPEED * (ros::Time::now().toSec() - start_time);
			loop_rate.sleep();
		}
		// update distance traveled
		distance_counter += distance_counter;
		loop_rate.sleep();
	}
}

/**
 * Initiates the node and begins our turtlebot's journey into the unknown
 */
int main (int argc, char **argv) {
	ros::init(argc, argv, "explore");
	ros::NodeHandle n;
	// start threads
	boost::thread turn_thread(turn);
	boost::thread drive_thread(drive);
	//detach threads
	turn_thread.detach();
	drive_thread.detach();
	// wait until program is terminated
	ros::Rate loop_rate(1);
	while (ros::ok()) {
	    loop_rate.sleep();
	}
	return 0;
}
