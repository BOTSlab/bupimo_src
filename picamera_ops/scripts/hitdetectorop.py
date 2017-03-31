#!/usr/bin/env python
"""
Casts out rays from the centre of the panoramic image and detects the first
contact of each ray with a set of hard-coded colour values.  This contact
constitutes a 'hit'.

BAD: Code copied from castedgedetectorop.  Should refactor to eliminate common code.

Authors: Daniel Cook <dac456@mun.ca>, Andrew Vardy <av@mun.ca>
"""

import rospy, time, math, cv2, cv, sys
import numpy as np

from math import pi
from op import Op
from picamera_ops.msg import *
from bupimo_utils.angles import constrain_angle_neg_pos_pi, constrain_angle_pos_twopi

class HitDetectorOp(Op):

    def __init__(self):

        # Get parameters
        self.debug = rospy.get_param("debug")
        self.image_width = rospy.get_param("image_width")
        self.image_height = rospy.get_param("image_height")
        self.centre_x = rospy.get_param("image_mirror_centre_x")
        self.centre_y = rospy.get_param("image_mirror_centre_y")
        self.inner_radius = rospy.get_param("image_mirror_inner_radius")
        self.outer_radius = rospy.get_param("image_mirror_outer_radius")
        self.centre = (self.centre_x, self.centre_y)

        self.first_run_calc_offset = False

        # Colors defined in HSV
        self.wrapped_hsv = True 
        self.lower_puck_hsv = np.array([4, 120, 100])
        self.upper_puck_hsv = np.array([176, 160, 240])
    
        self.generate_lines()

        self.publisher = rospy.Publisher('hits', HitArray, queue_size=1)

    def apply(self, image, image_debug):
        msg = HitArray()
        msg.hits = self.find_hits_on_all_lines(image, image_debug, self.debug)
        self.publisher.publish(msg)

    def generate_lines(self):
        """ Generates the lines used to detect collisions. """
        
        x = self.centre_x
        y = self.centre_y

        # Lines start offset slightly from the inner circle
        # Assume endpoint is another offset from inner circle
        p1x = int(x + self.inner_radius)
        p1y = int(y)
        p2x = int(x + self.outer_radius)
        p2y = int(y)

        # Generate lines around our area of interest
        self.lines = []
    #    for theta in np.linspace(0.0, 6.28, 50, False):
        angles = np.linspace(math.pi, 2*math.pi, 50, False)
        #angles = [3*pi/2]

        for theta in angles:
            theta = constrain_angle_pos_twopi(theta)

            p1xr = int( np.cos(theta) * (p1x - x) - np.sin(theta) * (p1y - y) + x )
            p1yr = int( np.sin(theta) * (p1x - x) + np.cos(theta) * (p1y - y) + y )
            p2xr = int( np.cos(theta) * (p2x - x) - np.sin(theta) * (p2y - y) + x )
            p2yr = int( np.sin(theta) * (p2x - x) + np.cos(theta) * (p2y - y) + y )

    #        lines.append( (theta, p1xr, p1yr, p2xr, p2yr, 0) )
            self.lines.append( [theta, p1xr, p1yr, p2xr, p2yr, 0] )
                                                          # THE OFFSET FROM FIRST_RUN

    def find_hits_on_all_lines(self, image_gray, image_debug, debug = False):
        # Iterate over each line and detect edges
        # detect_hit_in_ray will return the first obstacle encountered on a line
        # The angle and distance from image centre are stored to describe the obstacle position (polar coords)
        # Image centre is also reported

        x = self.centre_x
        y = self.centre_y

        if debug:
            # Draw inner and outer radii
            cv2.circle(image_debug, (x,y), int(self.inner_radius), (0,255,0), 2)
            cv2.circle(image_debug, (x,y), int(self.outer_radius), (0,255,0), 2)

        hits = []
        for line in self.lines:
            hit, hit_x, hit_y = self.detect_hit_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]), line[5])
            offset_to_line = 0  # Used only for first_run_calc_offset

            if hit is not None:
                hits.append(hit)

                # Valid only for first_run_calc_offset
                if hit.rho < self.inner_radius + 15: # Excluding distant non-self
                    offset_to_line = hit.rho - self.inner_radius + 1

                if debug:
                    if hit.theta < 0:
                        # Negative angles in blue
                        byte_value = int(-255 * hit.theta / pi)
                        cv2.circle(image_debug, (hit_x, hit_y), 5, (byte_value,0,0), -1)
                    else:
                        # Positive angles in green
                        byte_value = int(255 * hit.theta / pi)
                        cv2.circle(image_debug, (hit_x, hit_y), 5, (0,byte_value,0), -1)

            if self.first_run_calc_offset:
                line[5] = offset_to_line

        # Sort the hits by angle
        hits = sorted(hits, key=lambda hit: hit.theta)

        if debug:
            cv2.imshow("hit_detector", image_debug)
            key = cv2.waitKey(1) & 0xFF
            #if key == ord("q"):
            #    break

        self.first_run_calc_offset = False

        return hits

    # Given a color image, the ray's angle and two end points,
    # build a line using Bresenham algorithm
    def detect_hit_in_ray(self, image, theta, p1, p2, offset):
        # Input line points are warped to octant zero in order to generate the line
        # Pixel positions are warped back to the original octant as they are appended
        # using switch_from_octant_zero
        p1c = self.switch_to_octant_zero(theta, p1)
        p2c = self.switch_to_octant_zero(theta, p2)
        dx = (p2c[0] - p1c[0])
        dy = (p2c[1] - p1c[1])
        D = dy - dx

        # March using Bresenham algorithm, appending position and intensity as we go
        # OpenCV's LineIterator only returns intensity, so we do this here instead
        line_pos = []
        line_col = []
        y = p1c[1]
        if y < self.image_height:
            for x in range(p1c[0], p2c[0]-1):
                if x < self.image_width:
                    line_pos.append(self.switch_from_octant_zero(theta, (x,y)))
                    if line_pos[-1][1] < self.image_height and line_pos[-1][0] < self.image_width:
                        line_col.append(image[line_pos[-1][1]][line_pos[-1][0]])
                    else:
                        line_col.append(0.0)

                    if D >= 0:
                        y += 1
                        D -= dx
                    D += dy

        #print line_col

        # The code below assumes we're using pink, the boundaries for which
        # wrap around from 0 to 180
        assert self.wrapped_hsv

        for index in range(len(line_col)):
            bgr = np.uint8([[line_col[index]]])
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            if ((hsv[0][0][0] < self.lower_puck_hsv[0] or 
                 hsv[0][0][0] >= self.upper_puck_hsv[0]) and 
                hsv[0][0][1] > self.lower_puck_hsv[1] and 
                hsv[0][0][2] > self.lower_puck_hsv[2] and 
                hsv[0][0][1] <= self.upper_puck_hsv[1] and 
                hsv[0][0][2] <= self.upper_puck_hsv[2]):

                # A hit!
                hit = Hit()
                hit.kind = 0
                # Convert the angle to be counter-clockwise in the robot's
                # reference frame where the forwards direction of the robot is
                # facing upwards in the image.
                hit.theta = constrain_angle_neg_pos_pi(3.0*pi/2.0 - theta)
                x = line_pos[index][0]
                y = line_pos[index][1]
                hit.rho = np.sqrt( (self.centre_x - x)**2 +
                                   (self.centre_y - y)**2 )
                return hit, x, y
            
        # Nothing found along this ray
        return None, None, None

    # Transform point on a line to zeroth octant
    # Needed for input to Bresenham line algorithm
    def switch_to_octant_zero(self, theta, p):
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
    def switch_from_octant_zero(self, theta, p):
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