#!/usr/bin/env python
"""
First test of wandering while avoiding "cast obstacles".

Andrew Vardy
"""

import rospy, math, random
from obstacle_detector.msg import CastObstacle
from obstacle_detector.msg import CastObstacleArray
from geometry_msgs.msg import Twist

from bupimo_utils.movements import wander_while_avoiding_castobs

def castobstacles_callback(castobstacle_array_msg):
    twist = wander_while_avoiding_castobs(castobstacle_array_msg)
    cmd_vel_publisher.publish(twist)
        
if __name__ == '__main__':
    rospy.init_node('castwander_test')

    # Publish to 'cmd_vel'
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('castobstacles', CastObstacleArray, castobstacles_callback)
    
    rospy.spin()
