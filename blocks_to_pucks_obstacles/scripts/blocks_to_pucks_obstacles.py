#!/usr/bin/env python  
"""
ROS Node which subscribes to 'block_data' (published by pixy_node) and publishes corresponding puck types and positions on 'pucks' as well as obstacles on 'obstacles'.  The crucial step provided here is to reference the calibration data for the Pixy camera so that the pucks (and obstacles) have both image coordinates (xi, yi) and corresponding real-world coordinates (xr, yr).

Andrew Vardy
"""

import rospy
import rospkg
import csv
import math
from pixy_node.msg import PixyData
from pixy_node.msg import PixyBlock
from bupimo_msgs.msg import Puck
from bupimo_msgs.msg import PuckArray
from bupimo_msgs.msg import Obstacle
from bupimo_msgs.msg import ObstacleArray
from sensor_msgs.msg import PointCloud # For visualization of pucks
from sensor_msgs.msg import ChannelFloat32 # For visualization of pucks

def block_data_callback(block_data_msg):

    obs_array = ObstacleArray()
    puck_array = PuckArray()

    # For visualization
    vis_pucks = PointCloud()
    vis_pucks.header.frame_id = '/base_link'
    channel = ChannelFloat32()
    channel.name = "intensity"

    for block in block_data_msg.blocks:
        if block.signature >= obstacle_signature_start and \
           block.signature <= obstacle_signature_stop:
            # Its an obstacle
            obs = Obstacle()
            obs.xi = int(block.roi.x_offset)
            obs.yi = int(block.roi.y_offset)
            key = (obs.xi, obs.yi)
            if key in correspDict:
                (xr, yr) = correspDict[key]
                obs.position.x = xr
                obs.position.y = yr
                obs_array.obstacles.append(obs)
        else:
            # Its a puck
            puck = Puck()
            puck.type = block.signature
            puck.xi = int(block.roi.x_offset)
            puck.yi = int(block.roi.y_offset)
            key = (puck.xi, puck.yi)
            if key in correspDict:
                (xr, yr) = correspDict[key]
                puck.position.x = xr
                puck.position.y = yr
                add_puck(puck_array, puck)

            # For visualization
            vis_pucks.points.append(puck.position)
            # Intensity starts at 0 and scales up to 1 based on type.
            channel.values.append((puck.type - 1)/ (number_puck_types - 1.0))

    obs_publisher.publish(obs_array)
    pucks_publisher.publish(puck_array)
    vis_pucks.channels.append(channel)
    vis_pucks_publisher.publish(vis_pucks)

def add_puck(puck_array, puck):
    """Add the puck to the array, but only if there are no other already-added pucks within the 'duplicate_puck_threshold' distance.  The reason for this check is that the pixy_node seems to give duplicate (or almost duplicate) entries sometimes.  That is, one block seems to be reported as two.
"""

    duplicate = False
    for other in puck_array.pucks:
        # Calculate Euclidean distance between the 'other' puck and the given
        # puck.  If this is less than the threshold then we consider them
        # duplicates.
        dx = puck.position.x - other.position.x
        dy = puck.position.y - other.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < duplicate_puck_threshold:
            duplicate = True
        
    if not duplicate:
        puck_array.pucks.append(puck)

def readCorrespondences(filename):
    f = open(filename, 'rt')
    x = []
    y = []
    Xr = []
    Yr = []
    try:
        reader = csv.reader(f, delimiter=',')
        # Skip the first (header) row
        next(reader)
        for row in reader:
            x.append(int(row[0]))
            y.append(int(row[1]))
            Xr.append(float(row[2]))
            Yr.append(float(row[3]))
        return [x, y, Xr, Yr]
    finally:
        f.close()

if __name__ == '__main__':
    global number_puck_types, obstacle_signature_start, obstacle_signature_stop, duplicate_puck_threshold, pucks_publisher, vis_pucks_publisher, correspDict

    rospy.init_node('blocks_to_pucks')

    # Read the following parameters managed by the ROS parameter server:
    #
    #   number_puck_types:  Total number of puck types expected.
    #
    #   obstacle_signature_start:  Starting signature (i.e. pixy signature) of
    #                              blocks to be treated as obstacles
    #
    #   obstacle_signature_stop:  Last signature (i.e. pixy signature) of
    #                             blocks to be treated as obstacles
    #
    #   duplicate_puck_threshold:  See comment in 'add_puck' above.
    #
    number_puck_types = rospy.get_param('number_puck_types')
    obstacle_signature_start = rospy.get_param('obstacle_signature_start')
    obstacle_signature_stop = rospy.get_param('obstacle_signature_stop')
    duplicate_puck_threshold = rospy.get_param('~duplicate_puck_threshold')

    # Read in the interpolated correspondences determined via the calibration
    # process.
    [Xi, Yi, Xr, Yr] = readCorrespondences('/home/pi/robot_int_cor.csv')

    # Create a dictionary for convenient look-up of the position corresponding
    # to each available image coordinate.
    correspDict = {}
    for i in range(len(Xi)):
        correspDict[(Xi[i], Yi[i])] = (Xr[i], Yr[i])

    # Publish to /obstacles
    obs_publisher = rospy.Publisher('/obstacles', ObstacleArray, queue_size=1)

    # Publish to /pucks
    pucks_publisher = rospy.Publisher('/pucks', PuckArray, queue_size=1)

    # Publish to /vis_pucks
    vis_pucks_publisher = rospy.Publisher('/vis_pucks', PointCloud, queue_size=1)

    # Subscribe to /block_data
    rospy.Subscriber('/block_data', PixyData, block_data_callback)

    rospy.spin()
