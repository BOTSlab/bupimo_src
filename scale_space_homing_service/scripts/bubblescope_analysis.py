#!/usr/bin/env python

import cv2
import time
import rospy
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

#TODO: Get these from the bubblescope property service
xRes = 1296
yRes = 972



def handle_get_bearing_for_goal(req):
	#TODO: call the proper method in scale_space_homing

	goalId = req.goalId
    res = GetBearingForGoalResponse()
    res.bearing = ss.get_bearing_for_goal(image, goalId)

    return res

def handle_set_goal_location(req):
	ss.set_goal_location(image, req.goalId)



if __name__ == '__main__':
    Check for debug 
    parser = OptionParser()
    parser.add_option("-d", "--debug", dest="debug_on", help="enable debug output", default=False, action='store_true')
    (options, args) = parser.parse_args()
    global debug
    debug = options.debug_on

    #Define all the obstacle and homing services, publishers etc
    rospy.init_node('obstacle_detector', anonymous=False)
    castObsPub = rospy.Publisher('castobstacles', CastObstacleArray, queue_size=1)
    

    rospy.init_node('scale_space_homing_service')
    s = rospy.Service('get_bearing_for_goal', GetBearingForGoal, handle_get_bearing_for_goal)
    t = rospy.Service('set_goal_location', SetGoalLocation, handle_set_goal_location)

    # Fetch bubblescope information from the service
    rospy.wait_for_service('get_bubblescope_properties')
    get_bubblescope_properties = rospy.ServiceProxy('get_bubblescope_properties', GetBubblescopeProperties)
    res = get_bubblescope_properties()    

    if res is not None:
        global roiCenter
        roiCenter = (int(res.center[0]), int(res.center[1]))
        outerRad = res.outer_radius

        #Generate and save the mask to be applied when finding keypoints
        ss.generateMask(roiCenter, int(outerRad))

        lines = generate_collision_lines(roiCenter, innerRadius, outerRadius)


    with PiCamera() as camera:
            camera.resolution = (img_width,img_height)
            camera.ISO = 100
            camera.sa = 100
            camera.awb = "flash"
            camera.co = 100

            raw_capture = PiRGBArray(camera)

            time.sleep(0.1)

            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            	global image
                image = frame.array
                #image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                

    rospy.spin()