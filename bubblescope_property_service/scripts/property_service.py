#!/usr/bin/env python

import time
import rospy
import cv2
from bubblescope_property_service.srv import *

from picamera.array import PiRGBArray
from picamera import PiCamera

# TODO: these should come from the parameter server
img_width = 1296
img_height = 972

def handle_get_bubblescope_properties(req):
    with PiCamera() as camera:
        # Camera settings
        # I don't know what they mean
        camera.resolution = (img_width,img_height)
        camera.ISO = 100
        camera.sa = 100
        camera.awb = "flash"
        camera.co = 100

        raw_capture = PiRGBArray(camera)

        time.sleep(0.1)

        # Before doing anything, capture the first frame and find the inner circle
        camera.capture(raw_capture, format="bgr")
        image = raw_capture.array
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(image_gray, cv2.cv.CV_HOUGH_GRADIENT, 1, 200, param1=50, param2=40, minRadius=10, maxRadius=180)
        raw_capture.truncate(0)

        if circles is not None:
            x, y, r = circles[0][0]

            res = GetBubblescopePropertiesResponse()
            res.center = (x,y)
            res.inner_radius = r*1.2
            res.outer_radius = r*2.25
            return res
        else:
            print "no circle found\n"
            return GetBubblescopePropertiesResponse()

if __name__ == '__main__':
    rospy.init_node('bubblescope_property_service')
    s = rospy.Service('get_bubblescope_properties', GetBubblescopeProperties, handle_get_bubblescope_properties)
    rospy.spin()
