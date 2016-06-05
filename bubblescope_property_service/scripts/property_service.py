#!/usr/bin/env python

import time
import rospy
import cv2
from bubblescope_property_service.srv import *

from optparse import OptionParser
from picamera.array import PiRGBArray
from picamera import PiCamera

# TODO: these should come from the parameter server
img_width = 1296
img_height = 972
inner_radius_factor = 1.2
outer_radius_factor = 2.25

def auto_estimate_properties(req):
    """Try and fit a circle to the image using Hough transform and use that
    circle's properties."""

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
            res.inner_radius = r * inner_radius_factor
            res.outer_radius = r * outer_radius_factor
            return res
        else:
            print "no circle found\n"
            return GetBubblescopePropertiesResponse()

def manual_properties(req):
    """Use manually specified (i.e. through command-line arguments)."""
    res = GetBubblescopePropertiesResponse()
    res.center = (centre_x, centre_y)
    res.inner_radius = radius * inner_radius_factor
    res.outer_radius = radius * outer_radius_factor
    return res

if __name__ == '__main__':
    global centre_x, centre_y, radius

    # Parse command-line options
    parser = OptionParser()
    parser.add_option("-m", "--manual", dest="manual", help="indicates that the inner circle's coordinates are manually specified through arguments -x and -y", action="store_true")
    parser.add_option("-x", "--centre_x", dest="centre_x", help="used with -m to specify the inner cicle's x-coordinate", action="store")
    parser.add_option("-y", "--centre_y", dest="centre_y", help="used with -m to specify the inner cicle's y-coordinate", action="store")
    parser.add_option("-r", "--radius", dest="radius", help="used with -m to specify the inner cicle's radius", action="store")
    (options, args) = parser.parse_args()

    rospy.init_node('bubblescope_property_service')

    if options.manual:
        centre_x = float(options.centre_x)
        centre_y = float(options.centre_y)
        radius = float(options.radius)
        s = rospy.Service('get_bubblescope_properties', GetBubblescopeProperties, manual_properties)
    else:
        s = rospy.Service('get_bubblescope_properties', GetBubblescopeProperties, auto_estimate_properties)

    rospy.spin()
