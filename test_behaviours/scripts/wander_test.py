#!/usr/bin/env python
"""
First test of wandering while avoiding "obstacle pucks".

Andrew Vardy
"""

import rospy, math, random
from bupimo_msgs.msg import ClusterArray
from bupimo_msgs.msg import ObstacleArray
from geometry_msgs.msg import Twist

from bupimo_utils.movements import wander_while_avoiding_obs_pucks

def clusters_callback(msg):
    global cluster_array_msg
    # Just store to handle in 'obstacles_callback'
    cluster_array_msg = msg

def obstacles_callback(obstacle_array_msg):
    global last_twist

    twist = wander_while_avoiding_obs_pucks(obstacle_array_msg, \
                                            cluster_array_msg, last_twist)
    cmd_vel_publisher.publish(twist)
    last_twist = twist
        
if __name__ == '__main__':
    global cluster_array_msg, last_twist
    cluster_array_msg = None
    last_twist = Twist()

    rospy.init_node('wander_test')

    # Publish to 'cmd_vel'
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('clusters', ClusterArray, clusters_callback)
    rospy.Subscriber('obstacles', ObstacleArray, obstacles_callback)
    
    rospy.spin()
