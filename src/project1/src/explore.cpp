// imports
// --------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include <math.h>
#include <boost/thread/thread.hpp>
#include <iostream>
// constants
// --------------------------------------------------------
#define TURN_ANGLE M_PI / 12
#define SPEED 0.2

// global variables
// --------------------------------------------------------
double distance_counter = 0;
bool canEscape = true;
bool canAvoid = true;
bool canTurn = true;
bool canDrive = true;
int cooldown = 0;

// ========================================================
// HAULT FEATURE
// ========================================================
/*
* Handles bumber events
*/
void hault(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    // shut down the system! Turtle bot has died!
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        ros::shutdown();
    }
}

// ========================================================
// KEYBOARD FEATURE
// ========================================================
/**
* Deals with the keyboard commands from teleop keyboard
*/
void keyboard(const geometry_msgs::Twist::ConstPtr& msg) {
    // get the distance traveled
    double deltaX = msg->linear.x;

    // increment cooldown counter and disable features
    cooldown++;
    canEscape = false;
    canAvoid = false;
    canTurn = false;
    canDrive = false;

    // decrement cooldown and update distance travelled after sleeping for 1 second
    ros::Duration(1.0).sleep();
    cooldown--;
    distance_counter += deltaX;

    // return functionality if applicable
    if (cooldown == 0) {
        canEscape = true;
        canAvoid = true;
        canTurn = true;
        canDrive = true;
    }
}

// ========================================================
// TURN FEATURE
// ========================================================
/**
* Publishes commands for rotating 15 degrees in a random direction.
* Commands are published whenever 1 meter has approximately been traveled.
*/
void turn() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // publisher for sending turn commands to gazebo
    ros::Publisher turn_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);
    // the turn command
    geometry_msgs::Twist turn_cmd;
    double angular_vel = TURN_ANGLE;    
    turn_cmd.linear.x = 0;
    turn_cmd.linear.y = 0;
    
    // run at 5Hz
    ros::Rate loop_rate(5);    
    while (ros::ok()) {        
        // make sure drive is enabled while not currently turning
        if (canTurn) {
            canDrive = true;
        }
        double start_time = ros::Time::now().toSec();
        // turn after 1m of moving and turn off driving while publishing turn command
        if (distance_counter >= 1 && canTurn) {
            canDrive = false;
            double current_angle = 0;
            double start_time = ros::Time::now().toSec();
            // get a random direction for turning
            turn_cmd.angular.z = (rand() % 2) ? angular_vel : -1 * angular_vel;
            while (ros::ok() && canTurn && current_angle < TURN_ANGLE) {
                turn_pub.publish(turn_cmd);
                current_angle =  angular_vel * ((ros::Time::now().toSec()) - start_time);
                loop_rate.sleep();
            }
            // reset distance counter
            distance_counter = 0;
        }
        loop_rate.sleep();
    }
}

// ========================================================
// DRIVE FEATURE
// ========================================================
/**
* Publishes commands for traveling in a straigh line for one meter.
*/
void drive() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // publisher for sending move commands to gazebo
    ros::Publisher drive_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    // the move to be published: STRAIGHT ONLY
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = SPEED;
    move_cmd.linear.y = 0;
    move_cmd.angular.z = 0;
    
    // run at 5Hz
    ros::Rate loop_rate(10);
    
    // enters loop until ros quits
    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();
        double current_distance = 0;        
        // publishes command for moving 1 meter at a time
        while (ros::ok() && canDrive && current_distance < 1) {
            drive_pub.publish(move_cmd);
            current_distance = SPEED * (ros::Time::now().toSec() - start_time);
            loop_rate.sleep();
        }
        // update distance traveled
        distance_counter += distance_counter;
        loop_rate.sleep();
    }
}

// ========================================================
// MAIN METHOD
// ========================================================

/**
 * Initiates the node and begins our turtlebot's journey into the unknown
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "explore");
    ros::NodeHandle n;
    // create subscibers
    ros::Subscriber hault_sub = n.subscribe("mobile_base/events/bumper", 1, hault);
    ros::Subscriber keyboard_sub = n.subscribe("turtlebot_telop_keyboard/cmd_vel", 5, keyboard);
    // start threads
    boost::thread turn_thread(turn);
    boost::thread drive_thread(drive);
    // detach threads
    turn_thread.detach();
    drive_thread.detach();
    // spin away
    ros::spin();
    return 0;
}
