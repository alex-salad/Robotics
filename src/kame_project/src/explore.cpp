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
#define DELTA 0.15

#define DRIVING 0
#define TURNING 1
#define AVOIDING 2
#define ESCAPING 3
#define KEYBOARDING 4


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
    int state;
    double distance_counter;

    // private functions
    void halt(const kobuki_msgs::BumperEvent::ConstPtr &msg);
    void keyboard(const geometry_msgs::Twist::ConstPtr &msg);
    void detect(const sensor_msgs::LaserScanConstPtr &msg);    
    void rotate(double goal, double velocity, int condition);
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
* initiates some things
*/
Explorer::Explorer(ros::NodeHandle *n) {
    this->n = n;
    p = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10); 
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
        state = KEYBOARDING;
        // add to publishing queue
        p.publish(*msg);
    }

    // reset to driving
    else {
        state = DRIVING;
    }
}

// ========================================================
// ROTATE FEATURE
// ========================================================
/**
* Publishes commands for rotating
*/
void Explorer::rotate(double goal, double velocity, int condition) {
    // the rotate command
    geometry_msgs::Twist rotate_cmd; 
    rotate_cmd.linear.x = 0;
    rotate_cmd.linear.y = 0;
    rotate_cmd.linear.z = 0;
    rotate_cmd.angular.z = velocity;

    double current_angle = 0;
    double start_time = ros::Time::now().toSec();
    
    // run at 10z
    ros::Rate loop_rate(10);

    // publish turn commands
    while (ros::ok() && current_angle < goal && state == condition) {
        p.publish(rotate_cmd);
        current_angle =  fabs(velocity) * ((ros::Time::now().toSec()) - start_time);
        loop_rate.sleep();
    }
}

// ========================================================
// DETECT FEATURE
// ========================================================
/**
* Determines where the obstacle comes from
*/
void Explorer::detect(const sensor_msgs::LaserScanConstPtr &msg) {
    // do nothing if in a higher priority state or already escaping
    if (state >= ESCAPING) {
        return;
    }

    double left_min = 2.0;
    double right_min = 2.0;

    // get the minimum value for right and left sides
    for (int i = 0; i < msg->ranges.size(); i++) {
        double range = msg->ranges[i];
        // ignore invalid data
        if (isnan(range) || range <= msg->range_min || range >= msg->range_max) {
            continue;
        }

        double angle = msg->angle_min + i * msg->angle_increment;

        // if distance is on the right
        if (angle < 0) {
            right_min = (right_min < range) ? right_min : range;
        } 
        // if distance is on the left
        else if (angle > 0) {
            left_min = (left_min < range) ? left_min : range;
        }
    }

    // no obstacle
    if (left_min > 1 + DELTA && right_min > 1 + DELTA) {
        ROS_INFO("NO OBSTACLE :)");

        // stop avoiding if no obstacle
        if (state == AVOIDING) {
            state = DRIVING;
        }
    }

    // obstacle to the front
    else if (fabs(right_min - left_min) < DELTA) {
        ROS_INFO("OBSTACLE DETECTED AT FRONT!");
        if (state < ESCAPING) {
            // change state and rotate
            state = ESCAPING;
            rotate(ESCAPE_ANGLE, ESCAPE_ANGLE / 3.0, ESCAPING);

            // reset state after escaping
            if (state == ESCAPING) {
                state = DRIVING;
            }
        }
    }
    // obstacle to the right
    else if (right_min < left_min) {
        ROS_INFO("OBSTACLE DETECTED AT RIGHT!");
        if (state <= AVOIDING) {
            // change state and rotate
            state = AVOIDING;
            rotate(TURN_ANGLE, TURN_ANGLE, AVOIDING);
        }
    }
    // obstacle to the left
    else {
        ROS_INFO("OBSTACLE DETECTED AT LEFT!");
        if (state <= AVOIDING) {
            // change state and rotate
            state = AVOIDING;
            rotate(TURN_ANGLE, -1 * TURN_ANGLE, AVOIDING);
        }
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
    // run at 10z
    ros::Rate loop_rate(10);

    // loop until ros quits
    while (ros::ok()) {        
        // make sure drive is enabled while not currently turning
        if (state == TURNING) {
            state = DRIVING;
        }

        // check if it is time to turn
        if (distance_counter >= 1 && state == DRIVING) {
            // change state
            state = TURNING;

            // get a random direction for turning
            double angular_vel = (rand() % 2) ? TURN_ANGLE : -1 * TURN_ANGLE;
            rotate(TURN_ANGLE, angular_vel, TURNING);

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
    // the move to be published: FORWARD ONLY
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = SPEED;
    move_cmd.linear.y = 0;
    move_cmd.linear.z = 0;
    move_cmd.angular.z = 0;
    
    // run at 10Hz
    ros::Rate loop_rate(10);
    
    // loops until ros quits
    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();
        double current_distance = 0;

        // publishes command for moving 1 meter at a time
        while (ros::ok() && state == DRIVING && current_distance < 1 - distance_counter) {
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
    ros::Subscriber keyboard_sub = n->subscribe("keyboard_controls", 10, &Explorer::keyboard, this);
    ros::Subscriber detect_sub = n->subscribe("scan", 2, &Explorer::detect, this);

    // start threads
    boost::thread turn_thread(&Explorer::turn, this);
    boost::thread drive_thread(&Explorer::drive, this);

    // detach threads
    turn_thread.detach();
    drive_thread.detach();

    // put into driving state
    state = DRIVING;

    // spin away
    ros::spin();
}

// ========================================================
// MAIN METHOD
// ========================================================

/**
 * Initiates the node and explorer for well ... exlporing
 */
int main (int argc, char **argv) {
    ros::init(argc, argv, "explore");
    ros::NodeHandle n;
    Explorer dora(&n);
    dora.explore();
    return 0;
}
