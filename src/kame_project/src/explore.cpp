// imports
// --------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <boost/thread/thread.hpp>

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
    void keyboard(const geometry_msgs::Twist::ConstPtr &msg);
    int detect(const sensor_msgs::LaserScanConstPtr &msg);
    void avoid()
    void turn();
    void drive();

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
* Shutsdown when a bumper has been pressed
*/
void Explorer::halt(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
    // shut down the system! Turtle bot has died!
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
	ROS_INFO("Man down, we need a medic!");
        ros::shutdown();
    }
}

// ========================================================
// KEYBOARD FEATURE
// ========================================================
/**
* Disables other features to enable somewhat uninterupted control
* Enables feautes when no keyboard input that moves
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
// DETECT FEATURE
// ========================================================
/**
* Determines where the obstacle comes from
*/
int Explorer::detect(const sensor_msgs::LaserScanConstPtr &msg) {
    double[] ranges = msg->ranges;
    double left_min = 1.0;
    double right_min = 1.0;

    for (int i = 0; i < ranges.size() < i++) {
        if (ranges[i] <= msg->range_min || ranges[i] >= msg->rang_max) {
            continue;
        }

        double angle = msg->min_anle + i * msg->angle_increment;

        // if distance is on the right
        if (angle < 0) {
            right_min = (right_min < ranges[i]) ? right_min : ranges[i];
        } else {
            left_min = (left_min < ranges[i]) ? left_min : ranges[i];
        }
    }

    if (left_min > 1 + DELTA && right_min > 1 + DELTA) {
        ROS_INFO("NO OBSTACLE :)");
        return NO_OBSTACLE;
    }

    if (fabs(right_min - left_min) < DELTA) {
        ROS_INFO("OBSTACLE DETECTED AT FRONT!");
        return FRONT_OBSTACLE;
    }

    else if (right_min < left_min) {
        ROS_INFO("OBSTACLE DETECTED AT RIGHT!");
        return RIGHT_OBSTACLE;
    }

    else {
        ROS_INFO("OBSTACLE DETECTED AT LEFT!");
        return LEFT_OBSTACLE;
    }

    return NO_OBSTACLE;
}

// ========================================================
// AVOID FEATURE
// ========================================================
/**
* Determines where the obstacle comes from
*/
int Explorer::detect(const sensor_msgs::LaserScanConstPtr &msg) {
    int status = detect(msg);
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

    // loop until ros quits
    while (ros::ok()) {        
        // make sure drive is enabled while not currently turning
        if (canTurn) {
            canDrive = true;
        }

        // check if it is time to turn
        if (distance_counter >= 1 && canTurn) {
            // shut off driving
            canDrive = false;
            double current_angle = 0;
            double start_time = ros::Time::now().toSec();

            // get a random direction for turning
            turn_cmd.angular.z = (rand() % 2) ? angular_vel : -1 * angular_vel;

            // publish turn commands
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
* Publishes commands for traveling in a straight line for one meter.
*/
void Explorer::drive() {
    // the move to be published: STRAIGHT ONLY
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = SPEED;
    move_cmd.linear.y = 0;
    move_cmd.angular.z = 0;
    
    // run at 10Hz
    ros::Rate loop_rate(10);
    
    // loops until ros quits
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
    ros::Subscriber avoid_sub = n->subscribe("scan", 2, &Explorer::avoid, this);

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
