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
    points = []
    for i in range(0, n):
        rho = scan.ranges[i]
        if not (rho == float('inf') or isnan(rho)) \
           and rho < 2:
            theta = scan.angle_min + i * scan.angle_increment
            points.append((rho*cos(theta), rho*sin(theta)))

    sim = rvo2.PyRVOSimulator(1/60., # Time step
                              1.5,   # neighborDist
                              5,     # maxNeighbors
                              1.5,   # timeHorizon (other agents)
                              2, #2     # timeHorizon (obstacles)
                              0.05,   # agent radius
                              1)     # agent max speed

    offset  = 0.01
    for point in points:
        # An obstacle needs to be a polygon with verticies in CCW order.  We
        # just create a triangle from each laser point
        top = (point[0], point[1] + offset)
        left = (point[0] - offset, point[1] - offset)
        right = (point[0] + offset, point[1] - offset)
        sim.addObstacle([top, left, right])
    sim.processObstacles()

    agent = sim.addAgent((0, 0))
    sim.setAgentPrefVelocity(agent, (1, 0)) # Try and go forwards

    sim.doStep()
    (vx, vy) = sim.getAgentVelocity(0)
    print "vel: {}, {}".format(vx, vy)

    # Debug
    #n = len(scan.ranges)
    #points = []
    #max_range = 0
    #min_range = float('inf')
    #for i in range(0, n):
    #    rho = scan.ranges[i]
    #    if not (rho == float('inf') or isnan(rho)):
    #        if rho >= max_range:
    #            max_range = rho
    #        if rho <= min_range:
    #            min_range = rho
    #print "max, min: {}, {}".format(max_range, min_range)

        
    
    # Now treat (x, y) as the goal and apply the simple control law
    twist = Twist()
    twist.linear.x = 0.0005 * vx
    twist.angular.z = 10.0 * vy
    cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('rvo2_laser')

    # Subscribe to scan
    rospy.Subscriber('scan', LaserScan, scan_callback)

    # Create a publisher so that we can output command velocities.
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.spin()
