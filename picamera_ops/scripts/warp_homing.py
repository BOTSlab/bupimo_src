#!/usr/bin/env python
"""
Visual homing using the 'warping' module.
"""

import warping.warping as warping
import cv2
import numpy as np
import settings

from math import atan2, sin, cos, degrees, pi

IMAGE_WIDTH = 40

delta_angle = (2.0*pi) / IMAGE_WIDTH

def init(settings):
    global image_centre, horizon

    # We have 8 possible colours from the pixy so we'll set 8 as the number
    # of warping objects to create --- i.e. the number of snapshot images to
    # maintain
    warping.init(8)

    image_centre = settings.roiCenter
    horizon = settings.horizon

def extract_1d_image(image):
    pixels = []

    angle = 0
    while angle < 2*pi:
        x = image_centre[0] + horizon * cos(angle)
        y = image_centre[1] + horizon * sin(angle)
        pixels.append( float(image[y][x]) )
        angle += delta_angle

    if settings.debug:
        image_debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        angle = 0
        while angle < 2*pi:
            x = int(image_centre[0] + horizon * cos(angle))
            y = int(image_centre[1] + horizon * sin(angle))
            cv2.circle(image_debug, (x, y), 5, (0,0,255), -1)
            angle += delta_angle
 
        cv2.imshow("extract_1d_image", image_debug)
        key = cv2.waitKey(1) & 0xFF

    return pixels

def get_bearing_for_goal(image, goalId):
    bearing = 0
    isValid = True

    cv = extract_1d_image(image)

    warping.compute_home_vector(goalId, cv)
    hx = warping.get_home_x()
    hy = warping.get_home_y()
    bearing = atan2(hy, hx)

    return bearing, isValid


def set_goal_location(image, goalId):
    ss = extract_1d_image(image)
    warping.set_snapshot(goalId, ss)
