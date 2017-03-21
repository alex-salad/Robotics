// imports
// --------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdlib.h>

// constants
// --------------------------------------------------------
#define TURN_ANGLE M_PI / 12
#define AVOID_ANGLE M_PI
#define SPEED 0.2
#define FRONT_OBSTACLE 0
#define RIGHT_OBSTACLE 1
#define LEFT_OBSTACLE 2
#define NO_OBSTACLE -1
#define DELTA 0.1

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
    bool canEscape;
    bool canAvoid;
    bool canTurn;
    bool canDrive;
    double distance_counter;
    int cooldown;
    // private functions
    void hault(const kobuki_msgs::BumperEvent::ConstPtr &msg);
    int detect(const pcl::PointCloud<pcl::PointXYZ> *cloud);
    void escape(const sensor_msgs::PointCloud2ConstPtr &msg);
    void avoid(const sensor_msgs::PointCloud2ConstPtr &msg);
    void keyboard(const geometry_msgs::Twist::ConstPtr &msg);
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
    distance_counter = 0;
    canEscape = true;
    canAvoid = true;
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
// HAULT FEATURE
// ========================================================
/**
* Handles bumber events
*/
void Explorer::hault(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
    // shut down the system! Turtle bot has died!
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        ros::shutdown();
    }
}

// ========================================================
// DETECT FEATURE
// ========================================================
/**
* detects if there are any obstacle and where they are from
*/
int Explorer::detect(const pcl::PointCloud<pcl::PointXYZ> *cloud) { 
    // NOTE X = RIGHT, Y = DOWN, Z = FORWARD
    double distance_right = 0;
    int right_points = 0;
    double distance_left = 0;
    int left_points = 0;

    // will use z as distance even though distance is (x^2 + y^2 + z^2)^(1/2)

    // iterate through points and calculate the distance
    for (int i = 0; i < cloud->size(); i++) {
        const pcl::PointXYZ *temp = &(cloud->points[i]);
        // check for invalid points and only work with decent ones
        if(!isnan(temp->x) && !isnan(temp->y) && !isnan(temp->z)) {
            // x > 0 is points to the right
            if(temp->x > 0) {
                distance_right += temp->z;
                right_points++;
            } else if (temp->x < 0) {
                distance_left += temp->z;
                left_points++;
            }
        }
    }

    // if obstacle further than 1 meter or if no points detected then no obstacle assumed
    if ((right_points == 0 && left_points == 0) || (distance_right > 1.0 && distance_left > 1.0)) {
        return NO_OBSTACLE;
    }

    // get the averages for each side; no points assumes far away e.g. 2 meters
    distance_right = (right_points > 0) ? distance_right / right_points : 2.0;
    distance_left = (left_points > 0) ? distance_left / left_points : 2.0;

    // if distances are about the same than assume symmetric obstacle ahead
    if (abs(distance_right - distance_left) < DELTA) {
        return FRONT_OBSTACLE;
    }

    // determine if obstacle is to the left or right
    if (distance_left < 1.0 && distance_left < distance_right) {
        return LEFT_OBSTACLE;
    }
    else if (distance_right < 1.0 && distance_right < distance_left) {
        return RIGHT_OBSTACLE;
    }

    // no obstacle by default
    return NO_OBSTACLE;
}

// ========================================================
// ESCAPE FEATURE
// ========================================================
/**
 * Deals with point cloud for escaping from dangerous obstacles
 */
void Explorer::rotate(double angle, double angular_velocity, bool &condition) {
    ros::Publisher rotate_cmd = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);
    // not moving
    rotate_cmd.linear.x = 0;
    rotate_cmd.linear.y = 0;
    rotate_cmd.linear.z = 0;
    // only rotating
    rotate_cmd.angular.z = angular_velocity;

    double current_angle = 0;
    double start_time = ros::Time::now().toSec();
    // get a random direction for turning
    while (ros::ok() &&  condition && current_angle < angle) {
        turn_pub.publish(turn_cmd);
        current_angle =  angular_vel * ((ros::Time::now().toSec()) - start_time);
        loop_rate.sleep();
    }


}

// ========================================================
// ESCAPE FEATURE
// ========================================================
/**
 * Deals with point cloud for escaping from dangerous obstacles
 */
void Explorer::escape(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> *cloud;
    pcl::fromROSMsg(*msg, *cloud);
    int status = detect(cloud);
    if (status == FRONT_OBSTACLE) {
        // TODO implement escape behavior
        std::cout << "obstacle detected! status: " << status << std::endl;
    }
}

// ========================================================
// AVOID FEATURE
// ========================================================
/**
 * Deals with point cloud for avoiding dangerous obstacles
 */
void Explorer::avoid(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> *cloud;
    pcl::fromROSMsg(*msg, *cloud);
    int status = detect(cloud);
    if (status == LEFT_OBSTACLE || status == RIGHT_OBSTACLE) {
        // TODO implement avoid behavior
        std::cout << "obstacle detected! status: " << status << std::endl;
    }
}



// ========================================================
// KEYBOARD FEATURE
// ========================================================
/**
* Deals with the keyboard commands from teleop keyboard
*/
void Explorer::keyboard(const geometry_msgs::Twist::ConstPtr &msg) {
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
void Explorer::turn() {
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
void Explorer::drive() {
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
            current_distance = SPEED * (ros::Time::now().toSec() - start_time);
            loop_rate.sleep();
        }
        // update distance traveled
        distance_counter += distance_counter;
        loop_rate.sleep();
    }
}

// ========================================================
// EXPLOR CONTROLLER
// ========================================================
/**
* Creates all the subscribers and threads and begins our turtlebot's journey into the unknown
*/
void Explorer::explore() {
    // create subscibers
    ros::Subscriber hault_sub = n->subscribe("mobile_base/events/bumper", 1, &Explorer::hault, this);
    ros::Subscriber keyboard_sub = n->subscribe("turtlebot_telop_keyboard/cmd_vel", 5, &Explorer::keyboard, this);
    ros::Subscriber escape_sub = n->subscribe("camera/depth/points", 2, &Explorer::escape, this);
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
