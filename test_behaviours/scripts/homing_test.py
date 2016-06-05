#!/usr/bin/env python
"""
Tests visual homing which is achieved through the 'SetGoalLocation' and
'GetBearingForGoal' services.

Andrew Vardy
"""

import rospy, math
from bupimo_msgs.msg import ClusterArray
from ???.srv import SetGoalLocation
from ???.srv import GetBearingForGoal
from geometry_msgs.msg import Twist

from bupimo_utils.movements import move_to_puck
from bupimo_utils.movements import move_towards_bearing

def clusters_callback(cluster_array_msg):
    """Doing things in this callback just because that will be the main method
    for cache_cons."""

    # Get the bearing to the goal location
    try:
        response = get_bearing(0)
    except rospy.ServiceException as excep
        print("Service problem: " + str(excep))

    # Move towards the bearing found
    twist = move_towards_bearing(response.bearing):

    cmd_vel_publisher.publish(twist)
        
if __name__ == '__main__':
    rospy.init_node('homing_test')

    # Wait for the 'set_goal' service then call it to capture the current
    # image as the goal.
    rospy.wait_for_service('set_goal')
    set_goal = rospy.ServiceProxy('set_goal', SetGoalLocation)
    try:
        ignored_response = set_goal(0)
    except rospy.ServiceException as excep
        print("Service problem: " + str(excep))

    # Wait for the 'get_bearing' service which will be called within
    # 'clusters_callback'.
    rospy.wait_for_service('get_bearing')
    get_bearing = rospy.ServiceProxy('get_bearing', GetBearingForGoal)

    # Setup publisher for controlling the robot's movement
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Subscribe to the clustered pucks topic
    rospy.Subscriber('clusters', ClusterArray, clusters_callback)
    
    rospy.spin()
