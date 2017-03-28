#!/usr/bin/env python

"""
Interface to sets of functionality based on the PiCamera class.
"""

import rospy
import cv2, time
import settings
import numpy as np

from castcolourdetector import CastColourDetector

from math import atan2, sin, cos, degrees, pi
from picamera import PiCamera
from picamera.array import PiRGBArray
from optparse import OptionParser

from std_msgs.msg import Float32

if __name__ == '__main__':

    image_width = rospy.get_param("image_width")
    image_height = rospy.get_param("image_height")

    ops = []
    op = CastColourDetectorOp()
    ops.append(op)

    # Begin processing images
    with PiCamera() as camera:
            camera.resolution = (image_width, image_height)
            camera.ISO = 100
            camera.sa = 100
            camera.awb = "flash"
            camera.co = 100

            raw_capture = PiRGBArray(camera)

            time.sleep(0.1)

            firstFrame = True

            for frame in camera.capture_continuous(raw_capture, \
                                            format="bgr", use_video_port=True):

                #image = cv2.cvtColor(frame.array, cv2.COLOR_BGR2GRAY)
                image_debug = frame.array.copy()

                if firstFrame == True:
                    rospy.init_node('picamera_ops', anonymous=False)
                    print "frame.array.shape: " + str(frame.array.shape)
                    firstFrame = False

                # Apply all defined ops
                for op in ops:
                    op.apply(frame.array, image_debug)
                    
                raw_capture.truncate(0)
                
                if rospy.is_shutdown():
                    break
    
    rospy.spin()
