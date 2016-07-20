#!/usr/bin/env python
"""
First test of picking up pucks on bupimo.

Andrew Vardy
"""

import rospy, math
from bupimo_msgs.msg import ClusterArray
from bupimo_msgs.msg import ZumoData
from geometry_msgs.msg import Twist

from bupimo_utils.pucks_and_clusters import get_closest_puck
from bupimo_utils.pucks_and_clusters import get_closest_puck_to_puck
from bupimo_utils.movements import move_to_puck
from bupimo_utils.movements import forwards

def zumo_callback(zumo_msg):
    """The front prox. sensor will tell us if a puck is in the gripper."""
    global puck_in_gripper
    puck_in_gripper = zumo_msg.frontProximity <= 8

def transition(new_state, message):
    global state
    state = new_state
    print("transition: " + state + " --- " + message)

def clusters_callback(cluster_array_msg):
    global target_puck

    closest_puck = get_closest_puck(cluster_array_msg)

    # Handle state transitions
    if state == "SCAN":
        if closest_puck != None:
            target_puck = closest_puck
            transition("TARGET", "Target acquired")
    elif state == "TARGET":
        target_puck = get_closest_puck_to_puck(cluster_array_msg, target_puck, \
                                               0.1)
        if target_puck == None:
            transition("SCAN", "Lost target")
        else:
            x = target_puck.position.x
            y = target_puck.position.y
            dist = math.sqrt(x*x + y*y)
            if dist < 0.1:
                transition("TARGET_CLOSE", "Puck is close") 
    elif state == "TARGET_CLOSE":
        target_puck = get_closest_puck_to_puck(cluster_array_msg, target_puck, \
                                               0.1)
        if target_puck == None:
            transition("TARGET_FINAL", "Target puck invisible, but almost in")
    elif state == "TARGET_FINAL":
        if puck_in_gripper:
            transition("SCAN", "")
    else:
        sys.exit("Unknown state")

    # Apply movement
    twist = Twist()
    if state == "SCAN":
        pass
    elif state == "TARGET" or state == "TARGET_CLOSE":
        twist = move_to_puck(target_puck)
    elif state == "TARGET_FINAL":
        twist = forwards()
    else:
        sys.exit("Unknown state")

    cmd_vel_publisher.publish(twist)
        
if __name__ == '__main__':
    global puck_in_gripper, state, target_puck

    # We need to initialize this here as the Zumo is turned on afterwards,
    # so 'zumo_callback' (which sets puck_in_gripper) will not be immediately
    # called.
    puck_in_gripper = False
    state = "SCAN"

    rospy.init_node('pickup_test')

    # Publish to 'cmd_vel'
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('clusters', ClusterArray, clusters_callback)
    rospy.Subscriber('zumo_data', ZumoData, zumo_callback)
    
    rospy.spin()
