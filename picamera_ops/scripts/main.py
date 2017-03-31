#!/usr/bin/env python

"""
Interface to sets of functionality based on the PiCamera class.
"""

import rospy, cv2, time
import numpy as np

from hitdetectorop import HitDetectorOp
#from castedgedetectorop import CastEdgeDetectorOp

from math import atan2, sin, cos, degrees, pi
from picamera import PiCamera
from picamera.array import PiRGBArray
from optparse import OptionParser

from std_msgs.msg import Float32

if __name__ == '__main__':

    image_width = rospy.get_param("image_width")
    image_height = rospy.get_param("image_height")

    ops = []
    #op = CastEdgeDetectorOp()
    op = HitDetectorOp()
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

                image = frame.array
                image_debug = frame.array.copy()

                if firstFrame == True:
                    rospy.init_node('picamera_ops', anonymous=False)
                    print "image.shape: " + str(image.shape)
                    firstFrame = False

                # Apply all defined ops
                for op in ops:
                    op.apply(image, image_debug)
                    
                raw_capture.truncate(0)
                
                if rospy.is_shutdown():
                    break
    
    rospy.spin()
