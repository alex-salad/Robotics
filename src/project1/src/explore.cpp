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

// constants
// --------------------------------------------------------
#define TURN_ANGLE M_PI / 12
#define AVOID_ANGLE M_PI
#define SPEED 0.2
#define FRONT_OBSTACLE 0
#define RIGHT_OBSTACLE 1
#define LEFT_OBSTACLE 2
#define NO_OBSTACLE -1

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
    int detect(const pcl::PointCloud<PointXYZ> *cloud);
    void escape(const sensor_msgs::PointCloud2ConstPtr &msg);
    void avoid(const sensor_msgs::PointCloud2ConstPtr &msg);
    void keyboard(const geometry_msgs::Twist::ConstPtr &msg);
    void turn();
    void drive();
public:
    // public functions
    Explorer(const ros::NodeHandle &n);
    ~Explorer();
    void run();
}

// ========================================================
// CONSTRUCTOR
// ========================================================
/**
* initiates all the variables
*/
Explorer::Explorer(const ros::NodeHandle &n) {
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
    double ax = 0;
    double ay = 0;
    double az = 0;
    // iterate through points
    int valid_points = 0;
    for (int i = 0; i < cloud->size(); i++) {
        pcl::PointXYZ *temp = cloud.points[i];
        // check for invalid points and only work with decent ones
        if(!isnan(temp->x) && !isnan(temp->y) && !isnan(temp->z)) {
            ax += temp->x;
            ay += temp->y;
            az += temp->z;
            valid_points++;
        }
    }
    if (valid_points > 0) {
        ax /= valid_points;
        ay /= valid_points;
        az /= valid_points;
        std::cout << "averages x: " << ax << " y: " << ay << " z: " << az << std::endl;
    }
    // TODO: FIX THIS THING
    return -1;
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
    ros::Subscriber hault_sub = n->subscribe("mobile_base/events/bumper", 1, hault);
    ros::Subscriber keyboard_sub = n->subscribe("turtlebot_telop_keyboard/cmd_vel", 5, keyboard);
    ros::Subscriber escape_sub = n->subscribe("camera/depth/points", 2, escape);
    // start threads
    boost::thread turn_thread(turn);
    boost::thread drive_thread(drive);
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
    Explorer dora(n);
    dora.exlore();
    return 0;
}
