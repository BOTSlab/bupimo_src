#!/usr/bin/env python

"""
Interface to sets of functionality based on the PiCamera class.
"""

import rospy
import cv2, time
import settings
import numpy as np

from picamera_ops.srv import *

import scale_space_homing
import warp_homing
from castobstacledetectorop import CastObstacleDetectorOp
#from rangegridop import RangeGridOp

from math import atan2, sin, cos, degrees, pi
from picamera import PiCamera
from picamera.array import PiRGBArray
from optparse import OptionParser

from std_msgs.msg import Float32

busy_handling_image = False

def handle_get_bearing_for_goal(req):
    print "handle_get_bearing_for_goal"
    busy_handling_image = True
    goalId = req.goalId
    res = GetBearingForGoalResponse()
    #res.bearing, res.is_valid = scale_space_homing.get_bearing_for_goal( \
    #                                                            image, goalId)
    res.bearing, res.is_valid = warp_homing.get_bearing_for_goal(image, goalId)
    busy_handling_image = False
    return res

def handle_set_goal_location(req):
    print "handle_set_goal_location"
    #scale_space_homing.set_goal_location(image, req.goalId)
    warp_homing.set_goal_location(image, req.goalId)
    return SetGoalLocationResponse()

def init_pubs_servs():
    #Define all services, publishers etc
    rospy.init_node('picamera_ops', anonymous=False)

    s = rospy.Service('get_bearing_for_goal', GetBearingForGoal, \
                        handle_get_bearing_for_goal)
    t = rospy.Service('set_goal_location', SetGoalLocation, \
                        handle_set_goal_location)

if __name__ == '__main__':
    settings.init()
    print "settings.debug: ",settings.debug
    print "settings.xRes: ",settings.xRes
    print "settings.yRes: ",settings.yRes

    # Initialize the components
    #scale_space_homing.init(settings)
    warp_homing.init(settings)
    ops = []
    op = CastObstacleDetectorOp(settings)
#    op = RangeGridOp(settings)
    ops.append(op)

    # Begin processing images
    with PiCamera() as camera:
            camera.resolution = (settings.xRes,settings.yRes)
            camera.ISO = 100
            camera.sa = 100
            camera.awb = "flash"
            camera.co = 100

            raw_capture = PiRGBArray(camera)

            time.sleep(0.1)

            firstFrame = True

            for frame in camera.capture_continuous(raw_capture, \
                                            format="bgr", use_video_port=True):
            	global image
                image = cv2.cvtColor(frame.array, cv2.COLOR_BGR2GRAY)

                if firstFrame == True:
                    init_pubs_servs()

                    print "image.shape: " + str(image.shape)
                    firstFrame = False

                if not busy_handling_image:
                    # Apply all defined ops
                    for op in ops:
                        op.apply(image, frame.array)
                    
                raw_capture.truncate(0)
                
                if rospy.is_shutdown():
                    break
    
    rospy.spin()
