#!/usr/bin/env python
"""
Inspired by Gauci et al's controller for object clustering.

Andrew Vardy
"""

import rospy, math
from picamera_ops.msg import Hit
from picamera_ops.msg import HitArray
from geometry_msgs.msg import Twist

def pausing_callback(hit_array_msg):
    global pause_counter

    if (pause_counter % pause_interval) == 0:
        # Do the actual work
        working_callback(hit_array_msg)

    elif (pause_counter % pause_interval) < pause_movetime:
        # Continue with previously published speed
        pass

    else:
        # Stop
        cmd_vel_publisher.publish(Twist())

    pause_counter += 1

def working_callback(hit_array_msg):

    # Pick the leftmost hit 
    leftmost_hit = None
    if len(hit_array_msg.hits) > 0:
        leftmost_hit = hit_array_msg.hits[-1]

    # Placeholder for now
    react_to_robot = False
        
    twist = Twist()
    if react_to_robot:
        # Turn left and slow
        twist.linear.x = 0.02
        twist.angular.z = 1.0

    elif leftmost_hit == None or leftmost_hit.theta <= 0:
        # Turn right
        twist.linear.x = 0.05
        twist.angular.z = -0.5

    else:
        # Turn left
        twist.linear.x = 0.05
        twist.angular.z = 0.5

    cmd_vel_publisher.publish(twist)
        
if __name__ == '__main__':
    global pause_interval, move_interval, pause_counter

    do_pause = False
    pause_interval = 25
    pause_movetime = 5 
    pause_counter = 0

    rospy.init_node('edge_cluster')

    # Publish to 'cmd_vel'
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    if do_pause:
        rospy.Subscriber('hits', HitArray, pausing_callback)
    else:
        rospy.Subscriber('hits', HitArray, working_callback)
    
    rospy.spin()
