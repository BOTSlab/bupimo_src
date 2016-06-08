#!/usr/bin/env python

import cv2
import time
import rospy
import settings
import numpy as np

import scale_space_homing as ss
import collision

from math import atan2, sin, cos, degrees, pi
from picamera.array import PiRGBArray
from picamera import PiCamera
from optparse import OptionParser

from std_msgs.msg import Float32

from bubblescope_property_service.srv import *
from scale_space_homing_service.srv import *
from obstacle_detector.msg import *

isFindingBearing = False

def handle_get_bearing_for_goal(req):
    #TODO: call the proper method in scale_space_homing
    isFindingBearing = True
    goalId = req.goalId
    res = GetBearingForGoalResponse()
    res.bearing = ss.get_bearing_for_goal(image, goalId)
    isFindingBearing = False
    return res

def handle_set_goal_location(req):
    print "pre SS call"
    ss.set_goal_location(image, req.goalId)
    print "post SS call"
    return SetGoalLocationResponse()


if __name__ == '__main__':
    #Check for debug 
    parser = OptionParser()
    parser.add_option("-d", "--debug", dest="debug_on", help="enable debug output", default=False, action='store_true')
    (options, args) = parser.parse_args()

    #Define all the obstacle and homing services, publishers etc
    rospy.init_node('bubblescope_analysis', anonymous=False)
    castObsPub = rospy.Publisher('castobstacles', CastObstacleArray, queue_size=1)
    

    s = rospy.Service('get_bearing_for_goal', GetBearingForGoal, handle_get_bearing_for_goal)
    t = rospy.Service('set_goal_location', SetGoalLocation, handle_set_goal_location)
    
    settings.init(options.debug_on)

    ss.generateMask(settings.roiCenter, int(settings.outer_rad))

    collision.generate_collision_lines(settings.roiCenter, settings.inner_rad, settings.outer_rad)

    print "settings.xRes: ",settings.xRes
    print "settings.yRes: ",settings.yRes
    with PiCamera() as camera:
            camera.resolution = (settings.xRes,settings.yRes)
            camera.ISO = 100
            camera.sa = 100
            camera.awb = "flash"
            camera.co = 100

            raw_capture = PiRGBArray(camera)

            time.sleep(0.1)

            firstFrame = True

            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            	global image
                image = cv2.cvtColor(frame.array, cv2.COLOR_BGR2GRAY)

                if firstFrame == True:
                    print "image.shape: " + str(image.shape)
                    firstFrame = False
            	#don't do this while there is a pending homing calculation
                if isFindingBearing == False:
                	
                 	# Publish array of detected obstacles for others to have fun with
		    msg = CastObstacleArray()
		    msg.obstacles = collision.find_obstacles_on_all_lines(image, frame.array, options.debug_on)
		    castObsPub.publish(msg)
                raw_capture.truncate(0)
                
                if rospy.is_shutdown():
                    break
    

    rospy.spin()
