#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Daniel Cook <dac456@mun.ca>

import time
import cv2
import cv
import numpy as np
import rospy, math
import settings

from picamera_ops.msg import *
from bupimo_utils.angles import constrain_angle

# TODO: these should come from the parameter server
#img_width = 1296
#img_height = 972
#img_width = 1296/2
#img_height = 972/2
inner_radius = None

first_run_calc_offset = True

TWO_PI = 2 * math.pi

# Transform point on a line to zeroth octant
# Needed for input to Bresenham line algorithm
def switch_to_octant_zero(theta, p):
    step = 6.28/8.0

    if theta < step and theta >= 0.0:
        return p
    if theta < step*2.0 and theta >= step:
        return (p[1],p[0])
    if theta < step*3.0 and theta >= step*2.0:
        return (p[1],-p[0])
    if theta < step*4.0 and theta >= step*3.0:
        return (-p[0],p[1])
    if theta < step*5.0 and theta >= step*4.0:
        return (-p[0],-p[1])
    if theta < step*6.0 and theta >= step*5.0:
        return (-p[1],-p[0])
    if theta < step*7.0 and theta >= step*6.0:
        return (-p[1],p[0])
    if theta < step*8.0 and theta >= step*7.0:
        return (p[0],-p[1])

# Transform point on a line from the zeroth octant
# Needed for output from Bresenham line algorithm
def switch_from_octant_zero(theta, p):
    step = 6.28/8.0

    if theta < step and theta >= 0.0:
        return p
    if theta < step*2.0 and theta >= step:
        return (p[1],p[0])
    if theta < step*3.0 and theta >= step*2.0:
        return (-p[1],p[0])
    if theta < step*4.0 and theta >= step*3.0:
        return (-p[0],p[1])
    if theta < step*5.0 and theta >= step*4.0:
        return (-p[0],-p[1])
    if theta < step*6.0 and theta >= step*5.0:
        return (-p[1],-p[0])
    if theta < step*7.0 and theta >= step*6.0:
        return (p[1],-p[0])
    if theta < step*8.0 and theta >= step*7.0:
        return (p[0],-p[1])

# Given a grayscale image, two line end points and a line angle
# build a line using Bresenham algorithm
# Image pixels added to line are then convolved to find gradient over the line
# Pixels exceeding threshold in the line gradient are returned as obstacle locations
def detect_collision_in_ray(image, theta, p1, p2, offset, invert_mask = False):
    # Input line points are warped to octant zero in order to generate the line
    # Pixel positions are warped back to the original octant as they are appended
    # using switch_from_octant_zero
    p1c = switch_to_octant_zero(theta, p1)
    p2c = switch_to_octant_zero(theta, p2)
    dx = (p2c[0] - p1c[0])
    dy = (p2c[1] - p1c[1])
    D = dy - dx

    # March using Bresenham algorithm, appending position and intensity as we go
    # OpenCV's LineIterator only returns intensity, so we do this here instead
    line_pos = []
    line_col = []
    y = p1c[1]
    if y < settings.yRes:
        for x in range(p1c[0], p2c[0]-1):
            if x < settings.xRes:
                line_pos.append(switch_from_octant_zero(theta, (x,y)))
                if line_pos[-1][1] < settings.yRes and line_pos[-1][0] < settings.xRes:
                    line_col.append(image[line_pos[-1][1]][line_pos[-1][0]])
                else:
                    line_col.append(0.0)

                if D >= 0:
                    y += 1
                    D -= dx
                D += dy

    # Convolve with filter to find image gradient over the line
    filter = [-1,-1,-1,-1,0,1,1,1,1]
    if invert_mask:
        filter = [1,1,1,1,0,-1,-1,-1,-1]
    line_grad =  np.convolve(line_col, filter, 'same')

    for idx, val in enumerate(line_grad):
        # This portion of the image is obscured by the robot
        # Pick a point further away to start considering edges
#        if theta > 3.49:
#            if val > 100 and idx > 60 and idx < line_grad.size-10:
#                return line_pos[idx]
#        # The rest of the image has a tighter starting boundary
#        else:
        if val > 100 and idx > (5 + offset) and idx < line_grad.size-10:
            return line_pos[idx]

    # Return the last value, indicating maximum range
    return line_pos[len(line_pos) - 1]

def init(settings):
    """ Generates the lines used to detect collisions. """
    
    center = settings.roiCenter
    inner_rad = settings.inner_rad
    outer_rad = settings.outer_rad

    global inner_radius

    x = center[0]
    y = center[1]
    inner_radius = inner_rad

    # Lines start offset slightly from the inner circle
    # Assume endpoint is another offset from inner circle
    p1x = int(x + inner_rad)
    p1y = int(y)
    p2x = int(x + outer_rad)
    p2y = int(y)

    # Generate lines around our area of interest
    global lines

    lines = []
    for theta in np.linspace(0.0, 6.28, 50, False):
        p1xr = int( np.cos(theta) * (p1x - x) - np.sin(theta) * (p1y - y) + x )
        p1yr = int( np.sin(theta) * (p1x - x) + np.cos(theta) * (p1y - y) + y )
        p2xr = int( np.cos(theta) * (p2x - x) - np.sin(theta) * (p2y - y) + x )
        p2yr = int( np.sin(theta) * (p2x - x) + np.cos(theta) * (p2y - y) + y )

#        lines.append( (theta, p1xr, p1yr, p2xr, p2yr, 0) )
        lines.append( [theta, p1xr, p1yr, p2xr, p2yr, 0] )
                                                      # THE OFFSET FROM FIRST_RUN


def find_obstacles_on_all_lines(image_gray, image_debug, debug = False):
    global first_run_calc_offset

    # Iterate over each line and detect edges
    # detect_collision_in_ray will return the first obstacle encountered on a line
    # The angle and distance from image centre are stored to describe the obstacle position (polar coords)
    # Image centre is also reported

    x = settings.roiCenter[0]
    y = settings.roiCenter[1]

    if debug:
        cv2.circle(image_debug, (x,y), int(inner_radius), (0,255,0), 2)
        min_rho = float('inf')
        for line in lines:
            collision_pos = detect_collision_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]), line[5], first_run_calc_offset)

            if collision_pos is not None:
                rho = np.sqrt((collision_pos[0]-x)**2 + (collision_pos[1]-y)**2)
                if rho < min_rho:
                    min_rho = rho

    obstacles = []
    for line in lines:
        collision_pos = detect_collision_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]), line[5], first_run_calc_offset)

        offset_to_line = 0  # Used only for first_run_calc_offset

        if collision_pos is not None:
            obstacle = CastObstacle()
            obstacle.theta = constrain_angle(TWO_PI - line[0])
            obstacle.rho = np.sqrt( (collision_pos[0]-x)**2 + (collision_pos[1]-y)**2 )
            obstacles.append(obstacle)

            # Valid only for first_run_calc_offset
            if obstacle.rho < inner_radius + 15: # Excluding distant non-self
                offset_to_line = obstacle.rho - inner_radius + 1

            if debug:
                if obstacle.rho == min_rho:
                    cv2.circle(image_debug, collision_pos, 5, (0,0,255), -1)
                else:
                    cv2.circle(image_debug, collision_pos, 5, (0,255,0), -1)

        if first_run_calc_offset:
            line[5] = offset_to_line


    if debug:
        cv2.imshow("DEBUG", image_debug)
        key = cv2.waitKey(1) & 0xFF
#        if key == ord("q"):
#            break

    first_run_calc_offset = False

    return obstacles
