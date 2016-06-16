#!/usr/bin/env python  
"""
ROS Node which visualizes various bupimo-related topics using Matplotlib.

Andrew Vardy
"""

import rospy, math
import matplotlib
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt

#from bupimo_msgs.msg import Puck
#from bupimo_msgs.msg import PuckArray
from bupimo_msgs.msg import ClusterArray
from sensor_msgs.msg import LaserScan

def scan_callback(scan):
    """ Plot this LaserScan message. """
    n = len(scan.ranges)

    angles = []
    valid_ranges = []
    for i in range(n):
        angles.append(scan.angle_min + i * scan.angle_increment)
        if not math.isnan(scan.ranges[i]):
            valid_ranges.append(scan.ranges[i])
        else:
            valid_ranges.append(0)

    laser_axis.cla()
    laser_axis.set_title("Laser Scan", va='bottom')
    laser_axis.plot(angles, valid_ranges, color='r', linestyle='None', marker='.', )
    plt.draw()

def clusters_callback(cluster_array):

    clusters_axis.cla()
    clusters_axis.set_title("Pucks and Clusters")

    for cluster in cluster_array_msg.clusters:
        cluster_x_vals = []
        cluster_y_vals = []
        for puck in cluster.array.pucks:
            cluster_x_vals.append(puck.x)
            cluster_y_vals.append(puck.y)
            if puck.type == 1:
                clusters_axis.plot(puck.x, puck.y, color='r', linestyle='None', marker='.', )
            else:
                clusters_axis.plot(puck.x, puck.y, color='g', linestyle='None', marker='.', )

        # Draw the cluster
        clusters_axis.plot(cluster_x_vals, cluster_y_vals, color='k')

    plt.draw()


if __name__ == '__main__':
    global laser_axis, clusters_axis

    rospy.init_node('visualizer')

    # Create the subplots for each visualized topic
    plt.ion()
    laser_axis = plt.subplot(121, projection='polar')
    clusters_axis = plt.subplot(122)

    plt.show()

    # Subscribe to 'scan'
    rospy.Subscriber('scan', LaserScan, scan_callback, queue_size=1)

    # Subscribe to 'clusters'
    rospy.Subscriber('clusters', ClusterArray, clusters_callback, queue_size=1)

    rospy.spin()
