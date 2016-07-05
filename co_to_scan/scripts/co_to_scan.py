#!/usr/bin/env python  
"""
ROS Node which subscribes to 'castobstacles' (published by picamera_ops) and
converts each such message into a LaserScan message published on 'scan'.  The
cast obstacles have a 'rho' field but this is distance in the image plane, not
true spatial distance.  In order to convert to spatial distance we interpolate
between a set of known distances at different radii.

Andrew Vardy
"""

import rospy
from scipy.interpolate import interp1d
from picamera_ops.msg import CastObstacle
from picamera_ops.msg import CastObstacleArray
from sensor_msgs.msg import LaserScan

def co_callback(coa): # coa = CastObstacleArray

    scan = LaserScan()
    scan.header.frame_id = '/base_link'
    scan.angle_min = 0

    n = len(co_msg.obstacles)
    scan.angle_min = coa.obstacles[0].theta
    scan.angle_max = coa.obstacles[n-1].theta
    scan.angle_increment = coa.obstacles[1].theta - coa.obstacles[0].theta

    for obs in coa.obstacles:
        distance = rho2range(obs.rho)
        scan.ranges.append(distance)
        scan.range_min = min(scan.range_min, distance)
        scan.range_max = max(scan.range_min, distance)

    scan_publisher.publish(scan)

if __name__ == '__main__':
    global image_mirror_30cm_radius image_mirror_60cm_radius \
            image_mirror_90cm_radius rho2range scan_publisher

    rospy.init_node('co_to_scan')

    # Read the following parameters managed by the ROS parameter server:
    image_mirror_30cm_radius = rospy.get_param('image_mirror_30cm_radius')
    image_mirror_60cm_radius = rospy.get_param('image_mirror_60cm_radius')
    image_mirror_90cm_radius = rospy.get_param('image_mirror_90cm_radius')

    # Create interpolating function 'rho2range'
    x = [0.3, 0.6, 0.9]
    y = [image_mirror_30cm_radius, \
         image_mirror_60cm_radius, 
         image_mirror_90cm_radius ]
    rho2range = interp1d(x, y)

    # Publish to /scan
    scan_publisher = rospy.Publisher('scan', LaserScan, queue_size=1)

    # Subscribe to /castobstacles
    rospy.Subscriber('castobstacles', CastObstacleArray, co_callback)

    rospy.spin()
