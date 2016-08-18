#!/usr/bin/env python  
"""
Just prints the maximum and minimum ranges from the laser.

Andrew Vardy
"""

import rospy
from math import *
from sensor_msgs.msg import LaserScan 

def scan_callback(scan):

    n = len(scan.ranges)
    points = []
    max_range = 0
    max_range_angle = None
    min_range = float('inf')
    min_range_angle = None
    for i in range(0, n):
        rho = scan.ranges[i]
        theta = scan.angle_min + i * scan.angle_increment
        if not (rho == float('inf') or isnan(rho)):
            if rho >= max_range:
                max_range = rho
                max_range_angle = theta
            if rho <= min_range:
                min_range = rho
                min_range_angle = theta

    print "max: {} at theta: {}, min: {} at theta {}".format(max_range, max_range_angle, min_range, min_range_angle)

if __name__ == '__main__':
    rospy.init_node('laser_max_min')

    # Subscribe to scan
    rospy.Subscriber('scan', LaserScan, scan_callback)

    rospy.spin()
