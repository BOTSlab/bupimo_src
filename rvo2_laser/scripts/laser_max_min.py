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
    min_range = float('inf')
    for i in range(0, n):
        rho = scan.ranges[i]
        if not (rho == float('inf') or isnan(rho)):
            if rho >= max_range:
                max_range = rho
            if rho <= min_range:
                min_range = rho

    print "max, min: {}, {}".format(max_range, min_range)

if __name__ == '__main__':
    rospy.init_node('laser_max_min')

    # Subscribe to scan
    rospy.Subscriber('scan', LaserScan, scan_callback)

    rospy.spin()
