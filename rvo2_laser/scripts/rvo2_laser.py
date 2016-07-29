#!/usr/bin/env python  
"""
Uses RVO2 collision avoidance simulation to avoid obstacles inferred from laser data.

Andrew Vardy
"""

import rospy, rvo2
from math import *
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

def scan_callback(scan):

    # Add laser points as static obstacles
    n = len(scan.ranges)
    obstacles = []
    for i in range(0, n):
        rho = scan.ranges[i]
        theta = scan.angle_min + i * scan.angle_increment

        obstacles.append((rho*cos(theta), rho*sin(theta)))

    sim = rvo2.PyRVOSimulator(1/60., 1.5, 5, 1.5, 2, 0.4, 2)
    agent = sim.addAgent((0, 0))
    sim.setAgentPrefVelocity(agent, (1, 0)) # Try and go forwards
    sim.addObstacle(obstacles)
    sim.processObstacles()
    sim.doStep()
    (x, y) = sim.getAgentPosition(0)
    
    # Now treat (x, y) as the goal and apply the simple control law
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = y
    cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('rvo2_laser')

    # Subscribe to scan
    rospy.Subscriber('scan', LaserScan, scan_callback)

    # Create a publisher so that we can output command velocities.
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist)

    rospy.spin()
