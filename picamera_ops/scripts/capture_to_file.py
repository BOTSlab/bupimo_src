#!/usr/bin/env python

"""
Use PiCamera to capture an image and save it to a file.
"""

import time

from picamera import PiCamera
from picamera.array import PiRGBArray

if __name__ == '__main__':

    image_width = 640
    image_height = 480

    with PiCamera() as camera:
        camera.resolution = (image_width, image_height)
        camera.ISO = 100
        camera.sa = 100
        camera.awb = "flash"
        camera.co = 100

        time.sleep(2)

        camera.capture('raspicam.jpg')
