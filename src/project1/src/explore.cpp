// imports
// --------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumberEvent"
#include <math.h>
#include <boost/thread/thread.hpp>

// constants
// --------------------------------------------------------
#define TURN_ANGLE M_PI / 12
#define SPEED 0.2

// global variables
// --------------------------------------------------------
double distance_counter = 0;
bool canKeyboard = true;
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
void haultCallback(const kobuki_msgs::BumberEvent::ConstPtr& msg) {
    // turn off other featurs when collision detected
    if (msg->state == kobuki_msgs::BumberEvent.PRESSED) {
        canEscape = false;
        canAvoid = false;
        canTurn = false;
        canDrive = false;
    } 
    // reenable features when free again
    else if (msg->state == kobuki_msgs::BumberEvent.RELEASED) {
        canEscape = true;
        canAvoid = true;
        canTurn = true;
        canDrive = true;        
    }
}

/*
* Listens to bumber events
*/
void hault() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // subscribe to teleop publish command
    ros::Subscriber sub = n.subscribe("mobile_base/events/bumber", 2, keyboardCallback);
    // spin away
    ros::spin();
}

// ========================================================
// KEYBOARD FEATURE
// ========================================================
/**
* Deals with the keyboard commands from teleop keyboard
*/
void keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // get the distance traveled
    double deltaX = msg->linear.x;

    // increment cooldown counter
    cooldown++;
    // disable other functions
    canEscape = false;
    canAvoid = false;
    canTurn = false;
    canDrive = false;

    // sleep for 1 second
    ros::Duration(1.0).sleep();

    // decrement cooldown counter
    cooldown--;
    // update distance travelled
    distance_counter += deltaX;

    // return functionality if applicable
    if (cooldown == 0) {
        canEscape = true;
        canAvoid = true;
        canTurn = true;
        canDrive = true;
    }
}

/**
* Listens to keyboard commands from teleop keyboard
*/
void keyboard() {
    // node pointer
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    // subscribe to teleop publish command
    ros::Subscriber sub = n.subscribe("turtlebot_telop_keyboard/cmd_vel", 5, keyboardCallback);
    // spin away
    ros::spin();
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
        // makes sure driving is enabled when turning is enabled
        if (!canTurn) {
            canDrive = false;
        } else if (!canDrive) {
            canDrive = true;
        }    
        // turn after 1 meter has been traveled and turns off driving while turning
        if (distance_counter >= 1 && canTurn) {
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

// ========================================================
// MAIN METHOD
// ========================================================

/**
 * Initiates the node and begins our turtlebot's journey into the unknown
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "explore");
    ros::NodeHandle n;
    // start threads
    boost::thread hault_thread(hault);
    boost::thread keyboard_thread(keyboard);
    boost::thread turn_thread(turn);
    boost::thread drive_thread(drive);
    //detach threads
    hault_thread.detach();
    keyboard_thread.detach();
    turn_thread.detach();
    drive_thread.detach();
    // wait until program is terminated
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        loop_rate.sleep();
    }
    return 0;
}
