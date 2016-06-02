#!/usr/bin/env python  
"""
This script takes the tags published on 'tag_array', associates them with known positions for tags on the calibration target, and writes these correspondences to a comma-separated file where each row contains xi, yi, xr, yr:
    (xi, yi): Image coordinates
    (xr, yr): Coordinates of the corresponding point in the robot reference
              frame (i.e. the /base_link frame).

In principle, this is a ROS node, but it only executes once on the first set of tags seen.

This is really the first step of the calibration process.  The second is achieved by 'interpolator.py' which reads 'known_correspondences.csv' and interpolates correspondences between them.

Andrew Vardy
"""

import roslib
roslib.load_manifest('arbot_calib')
import rospy
import sys
import csv
from math import floor
from apriltag_cpp.msg import TagPose
from apriltag_cpp.msg import TagPoseArray
from common_calib import get_corresponding_point

def tag_array_callback(tag_array_msg):
    filename = 'known_correspondences.csv'

    # Write to the correspondences in CSV format where the first two columns
    # are the (xi, yi) coordinates in image space and the next two columns
    # are the corresponding (xr, xr) coordinates in robot space.
    with open(filename, 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['xi', 'yi', 'xr', 'yr'], \
                                delimiter=',')
        writer.writeheader()

        for tag in tag_array_msg.tags:
            index = tagDict[tag.id]
            (xr, yr) = get_corresponding_point(index)

            # Compute the centroid of the four corners of the tag
            #xi = 0
            #yi = 0
            #for i in range(4):
            #    xi += tag.image_coordinates[i].x
            #    yi += tag.image_coordinates[i].y
            #xi /= 4
            #yi /= 4
            xi = int(tag.image_coordinates[0].x)
            yi = int(tag.image_coordinates[0].y)

            rowDict = {'xi':xi, \
                       'yi':yi, \
                       'xr':xr, \
                       'yr':yr }
            writer.writerow(rowDict)

    rospy.signal_shutdown("Work complete")

if __name__ == '__main__':
    global tagDict

    rospy.init_node('correspondences_from_tags')

    # Create a dictionary that maps tag id numbers onto the left-to-right,
    # top-to-bottom index required by 'get_corresponding_point'.  These values
    # are for the calibration image used with AprilCal: 
    #   8.5x11in-tag-mosaic-1in.png
    #
    # I have no idea why this particular mapping was chosen.
    tagDict = {}
    tagDict[14] = 0 
    tagDict[15] = 1 
    tagDict[16] = 2 
    tagDict[17] = 3 
    tagDict[18] = 4 
    tagDict[19] = 5 
    tagDict[20] = 6 
    tagDict[21] = 7 
    tagDict[22] = 8 
    tagDict[23] = 9 
    tagDict[38] = 10
    tagDict[39] = 11
    tagDict[40] = 12
    tagDict[41] = 13
    tagDict[42] = 14
    tagDict[43] = 15
    tagDict[44] = 16
    tagDict[45] = 17
    tagDict[46] = 18
    tagDict[47] = 19
    tagDict[62] = 20
    tagDict[63] = 21
    tagDict[64] = 22
    tagDict[65] = 23
    tagDict[66] = 24
    tagDict[67] = 25
    tagDict[68] = 26
    tagDict[69] = 27
    tagDict[70] = 28
    tagDict[71] = 29
    tagDict[86] = 30
    tagDict[87] = 31
    tagDict[88] = 32
    tagDict[89] = 33
    tagDict[90] = 34
    tagDict[91] = 35
    tagDict[92] = 36
    tagDict[93] = 37
    tagDict[94] = 38
    tagDict[95] = 39
    tagDict[110] = 40
    tagDict[111] = 41
    tagDict[112] = 42
    tagDict[113] = 43
    tagDict[114] = 44
    tagDict[115] = 45
    tagDict[116] = 46
    tagDict[117] = 47
    tagDict[118] = 48
    tagDict[119] = 49
    tagDict[134] = 50
    tagDict[135] = 51
    tagDict[136] = 52
    tagDict[137] = 53
    tagDict[138] = 54
    tagDict[139] = 55
    tagDict[140] = 56
    tagDict[141] = 57
    tagDict[142] = 58
    tagDict[143] = 59
    tagDict[158] = 60
    tagDict[159] = 61
    tagDict[160] = 62
    tagDict[161] = 63
    tagDict[162] = 64
    tagDict[163] = 65
    tagDict[164] = 66
    tagDict[165] = 67
    tagDict[166] = 68
    tagDict[167] = 69
    tagDict[182] = 70
    tagDict[183] = 71
    tagDict[184] = 72
    tagDict[185] = 73
    tagDict[186] = 74
    tagDict[187] = 75
    tagDict[188] = 76
    tagDict[189] = 77
    tagDict[190] = 78
    tagDict[191] = 79

    # Subscribe to /tag_array
    rospy.Subscriber('/tag_array', TagPoseArray, tag_array_callback)

    rospy.spin()
