// imports
// --------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include <math.h>
#include <boost/thread/thread.hpp>
#include "sensor_msgs/LaserScan.h"

// constants
// --------------------------------------------------------
#define TURN_ANGLE M_PI / 12
#define ESCAPE_ANGLE M_PI * 5.0 / 6.0
#define SPEED 0.2
#define FRONT_OBSTACLE 0
#define RIGHT_OBSTACLE 1
#define LEFT_OBSTACLE 2
#define NO_OBSTACLE -1
#define DELTA 0.15


// ========================================================
// CLASS DEFINITION
// ========================================================
/**
* contains all the functions for allowing the turtle bot to explore
*/
class Explorer {
private:
    // private variables
    ros::NodeHandle *n;
    ros::Publisher p;
    bool canTurn;
    bool canDrive;
    double distance_counter;
    int cooldown;
    // private functions
    void halt(const kobuki_msgs::BumperEvent::ConstPtr &msg);
    void rotate(double angle, double angular_vel, bool &condition);
    void keyboard(const geometry_msgs::Twist::ConstPtr &msg);
    void turn();
    void drive();

    void detect(const sensor_msgs::LaserScanConstPtr &msg);

public:
    // public functions
    Explorer(ros::NodeHandle *n);
    ~Explorer();
    void explore();
};

// ========================================================
// CONSTRUCTOR
// ========================================================
/**
* initiates all the variables
*/
Explorer::Explorer(ros::NodeHandle *n) {
    this->n = n;
    // publisher for auto movement
    p = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    canTurn = true;
    canDrive = true;
    cooldown = 0;   
}

// ========================================================
// DESTRUCTOR
// ========================================================
/**
* empty since there is nothing to destroy as of yet
*/
Explorer::~Explorer() {}

// ========================================================
// HALT FEATURE
// ========================================================
/**
* Handles bumber events
*/
void Explorer::halt(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
    // shut down the system! Turtle bot has died!
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
	ROS_INFO("Man down!");
        ros::shutdown();
    }
}

// ========================================================
// DETECT FEATURE
// ========================================================
/**
* detects if there are any obstacle and where they are from
*/
void Explorer::detect(const sensor_msgs::LaserScanConstPtr &msg) {
    ROS_INFO("FUCK THIS %f", msg->range_min);
}

// ========================================================
// ROTATE FEATURE
// ========================================================
/**
 * publishes commands for rotating the turtle bot
 */
void Explorer::rotate(double angle, double angular_vel, bool &condition) {
    // message for rotating
    geometry_msgs::Twist rotate_cmd;
    // not moving
    rotate_cmd.linear.x = 0;
    rotate_cmd.linear.y = 0;
    rotate_cmd.linear.z = 0;
    // only rotating
    rotate_cmd.angular.z = angular_vel;

    double current_angle = 0;
    double start_time = ros::Time::now().toSec();
    
    // run at 10Hz
    ros::Rate loop_rate(10); 
    // get a random direction for turning
    while (ros::ok() &&  condition && current_angle < angle) {
        p.publish(rotate_cmd);
        current_angle =  fabs(angular_vel) * ((ros::Time::now().toSec()) - start_time);
        loop_rate.sleep();
    }


}

// ========================================================
// KEYBOARD FEATURE
// ========================================================
/**
* Deals with the keyboard commands from teleop keyboard
*/
void Explorer::keyboard(const geometry_msgs::Twist::ConstPtr &msg) {
    // disable features and publish command
    if (msg->linear.x != 0 || msg->angular.z != 0) {
        canTurn = false;
        canDrive = false;
	
	p.publish(*msg);
    }
    // enable features
    else {
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
void Explorer::turn() {
    // the turn command
    geometry_msgs::Twist turn_cmd;
    double angular_vel = TURN_ANGLE;    
    turn_cmd.linear.x = 0;
    turn_cmd.linear.y = 0;
    
    // run at 10z
    ros::Rate loop_rate(10);    
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
                p.publish(turn_cmd);
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
void Explorer::drive() {
    // the move to be published: STRAIGHT ONLY
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = SPEED;
    move_cmd.linear.y = 0;
    move_cmd.angular.z = 0;
    
    // run at 10Hz
    ros::Rate loop_rate(10);
    
    // enters loop until ros quits
    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();
        double current_distance = 0;        
        // publishes command for moving 1 meter at a time
        while (ros::ok() && canDrive && current_distance < 1) {
            p.publish(move_cmd);
            current_distance = SPEED * (ros::Time::now().toSec() - start_time);
            loop_rate.sleep();
        }
        // update distance traveled
        distance_counter += current_distance;
        loop_rate.sleep();
    }
}

// ========================================================
// EXPLORE CONTROLLER
// ========================================================
/**
* Creates all the subscribers and threads and begins our turtlebot's journey into the unknown
*/
void Explorer::explore() {
    // create subscibers
    ros::Subscriber halt_sub = n->subscribe("mobile_base/events/bumper", 20, &Explorer::halt, this);
    ros::Subscriber keyboard_sub = n->subscribe("keyboard_controls", 100, &Explorer::keyboard, this);
    ros::Subscriber escape_sub = n->subscribe("scan", 2, &Explorer::detect, this);
    // start threads
    boost::thread turn_thread(&Explorer::turn, this);
    boost::thread drive_thread(&Explorer::drive, this);
    // detach threads
    turn_thread.detach();
    drive_thread.detach();
    // spin away
    ros::spin();
}

// ========================================================
// MAIN METHOD
// ========================================================

/**
 * Initiates the node and explorer for well ... exlporation
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "explore");
    ros::NodeHandle n;
    Explorer dora(&n);
    dora.explore();
    return 0;
}
