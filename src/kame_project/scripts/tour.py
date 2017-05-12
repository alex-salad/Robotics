#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import itertools
import numpy as np

# starting point of robot: updated through subscriber
position = [0, 0]


# gets the shortest path from predefined points and a origin
# from the robot's current position
def get_shortest_path(x,y):
    # Points and origin values
    origin = [x,y]                           # UPDATE TO CORRECT ORIGIN
    point_1 = [2.53566564977, -0.773420049965]
    point_2 = [4.49860523334, 0.348575832826]
    point_3 = [2.33846501217, 2.30097446639]
    point_4 = [-0.0342952696324, 1.04674247832]
    points = [point_1, point_2, point_3, point_4]

    # Now get all possible paths by getting permutations
    paths = list(itertools.permutations(points))

    # Iterate through each path
    distances = []
    for path in paths:
        dist = 0
        current_origin = origin
        for point in path:
            dist += np.linalg.norm(np.array(point)-np.array(current_origin))
            current_origin = point

        distances.append(dist)

    # Get the index of the shortest path
    shortest_index = distances.index(min(distances))

    # Get shortest path from paths using shortest_index
    shortest_path = paths[shortest_index]

    return shortest_path

# updates the current position of the turtlebot relative to map
def localize(msg):
    position[0] = msg.pose.pose.position.x
    position[1] = msg.pose.pose.position.y
    # rospy.loginfo('position updated: x = %f, y = %f', position[0], position[1])

def main():
    # setup the node and subscriber.
    # NOTE: subscriber runs in its own thread so no need for rospy.spin()
    rospy.init_node('tour', anonymous=True)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, localize)
    
    # Loops for user input
    while not rospy.is_shutdown():
        user = raw_input('Start the tour? (y/n): ').lower()
        # the yes option
        if user == 'y':
            rospy.loginfo('Thank you for joining us today. Pleas wait for us to get the tour ready :o')
            path = get_shortest_path(position[0], position[1])
            rospy.sleep(1.0)
                        
            # set up for sending goals
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()
            
            rospy.loginfo('Thank you for waiting. We will now begin our tour :)')
            for point in path:
                goal = MoveBaseGoal()

                #set up the frame parameters
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()

                # moving towards the current point in path
                goal.target_pose.pose.position =  Point(point[0], point[1],0)
                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0
                
                rospy.loginfo('Sending goal location ...')
                
                # send path and wait for result
                client.send_goal(goal)                
                client.wait_for_result()
                
                # continue only if goal successfully reached otherwise end the tour
                if(client.get_state() ==  GoalStatus.SUCCEEDED):
                    rospy.loginfo('Here we have [insert location name] which is [insert location desc] :P')

                else:
                    rospy.loginfo('The robot failed to reach the destination')
                    break
                    
            rospy.loginfo('This concludes our tour :D')
                
        # the no option
        else:
            rospy.loginfo('Have a nice day (^_^)')
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
