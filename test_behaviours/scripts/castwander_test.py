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

def pausing_callback(castobstacle_array_msg):
    global pause_counter

    if (pause_counter % pause_interval) == 0:
        # Do the actual work
        working_callback(castobstacle_array_msg)

    elif (pause_counter % pause_interval) < pause_movetime:
        # Continue with previously published speed
        pass

    else:
        # Stop
        cmd_vel_publisher.publish(Twist())

    pause_counter += 1

def working_callback(castobstacle_array_msg):
    twist = wander_while_avoiding_castobs(castobstacle_array_msg)
    cmd_vel_publisher.publish(twist)
        
if __name__ == '__main__':
    global pause_interval, move_interval, pause_counter

    do_pause = False
    pause_interval = 25
    pause_movetime = 5 
    pause_counter = 0

    rospy.init_node('castwander_test')

    # Publish to 'cmd_vel'
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    if do_pause:
        rospy.Subscriber('castobstacles', CastObstacleArray, pausing_callback)
    else:
        rospy.Subscriber('castobstacles', CastObstacleArray, working_callback)
    
    rospy.spin()
