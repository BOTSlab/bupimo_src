#!/usr/bin/env python
# Authors: Daniel Cook <dac456@mun.ca>, Andrew Vardy <av@mun.ca>

import rospy, time, math, cv2, cv
import numpy as np

from math import pi
from picamera_ops.msg import *
from bupimo_utils.angles import constrain_angle_neg_pos_pi, constrain_angle_pos_twopi

class CastColourDetectorOp(Op):

    def __init__(self, settings):

        # Get parameters
        self.debug = rospy.get_param("debug")
        self.image_width = rospy.get_param("image_width")
        self.image_height = rospy.get_param("image_height")
        self.centre_x = rospy.get_param("image_mirror_centre_x")
        self.centre_y = rospy.get_param("image_mirror_centre_y")
        self.inner_radius = rospy.get_param("image_mirror_inner_radius")
        self.outer_radius = rospy.get_param("image_mirror_outer_radius")
        self.centre = (self.image_mirror_centre_x, self.image_mirror_centre_y)

        self.first_run_calc_offset = False

        self.generate_lines(self)

        self.publisher = rospy.Publisher('castobstacles', CastColourArray, \
                                          queue_size=1)

    def apply(self, image, image_debug):
        msg = CastColourArray()
        msg.obstacles = self.find_obstacles_on_all_lines(image, image_debug, self.debug)
        self.publisher.publish(msg)

    def generate_lines(self):
        """ Generates the lines used to detect collisions. """
        
        x = self.center[0]
        y = self.center[1]

        # Lines start offset slightly from the inner circle
        # Assume endpoint is another offset from inner circle
        p1x = int(x + self.inner_radius)
        p1y = int(y)
        p2x = int(x + self.outer_radius)
        p2y = int(y)

        # Generate lines around our area of interest
        self.lines = []
    #    for theta in np.linspace(0.0, 6.28, 50, False):
    #    angles = np.linspace(math.pi, 2*math.pi, 50, False)
        angles = [3*pi/2]

        for theta in angles:
            theta = constrain_angle_pos_twopi(theta)

            p1xr = int( np.cos(theta) * (p1x - x) - np.sin(theta) * (p1y - y) + x )
            p1yr = int( np.sin(theta) * (p1x - x) + np.cos(theta) * (p1y - y) + y )
            p2xr = int( np.cos(theta) * (p2x - x) - np.sin(theta) * (p2y - y) + x )
            p2yr = int( np.sin(theta) * (p2x - x) + np.cos(theta) * (p2y - y) + y )

    #        lines.append( (theta, p1xr, p1yr, p2xr, p2yr, 0) )
            lines.append( [theta, p1xr, p1yr, p2xr, p2yr, 0] )
                                                          # THE OFFSET FROM FIRST_RUN

    def find_obstacles_on_all_lines(self, image_gray, image_debug, debug = False):
        global self.first_run_calc_offset

        # Iterate over each line and detect edges
        # detect_collision_in_ray will return the first obstacle encountered on a line
        # The angle and distance from image centre are stored to describe the obstacle position (polar coords)
        # Image centre is also reported

        x = settings.roiCenter[0]
        y = settings.roiCenter[1]

        if debug:
            # Draw inner and outer radii
            cv2.circle(image_debug, (x,y), int(self.inner_radius), (0,255,0), 2)
            cv2.circle(image_debug, (x,y), int(self.outer_radius), (0,255,0), 2)
            min_rho = float('inf')
            for line in lines:
                collision_pos = detect_collision_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]), line[5], self.first_run_calc_offset)

                if collision_pos is not None:
                    rho = np.sqrt((collision_pos[0]-x)**2 + (collision_pos[1]-y)**2)
                    if rho < min_rho:
                        min_rho = rho

        obstacles = []
        for line in lines:
            collision_pos = detect_collision_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]), line[5], self.first_run_calc_offset)

            offset_to_line = 0  # Used only for first_run_calc_offset

            if collision_pos is not None:
                obstacle = CastObstacle()
                # Convert the angle to be counter-clockwise in the robot's reference frame where
                # the forwards direction of the robot is facing upwards in the image.
                obstacle.theta = constrain_angle_neg_pos_pi(3.0*pi/2.0 - line[0])
                obstacle.rho = np.sqrt( (collision_pos[0]-x)**2 + (collision_pos[1]-y)**2 )
                obstacles.append(obstacle)

                # Valid only for first_run_calc_offset
                if obstacle.rho < self.inner_radius + 15: # Excluding distant non-self
                    offset_to_line = obstacle.rho - self.inner_radius + 1

                if debug:
                    if obstacle.rho == min_rho:
                        cv2.circle(image_debug, collision_pos, 5, (0,0,255), -1)
                    else:
                        if obstacle.theta < 0:
                            # Negative angles in blue
                            byte_value = int(-255 * obstacle.theta / pi)
                            cv2.circle(image_debug, collision_pos, 5, (byte_value,0,0), -1)
                        else:
                            # Positive angles in green
                            byte_value = int(255 * obstacle.theta / pi)
                            cv2.circle(image_debug, collision_pos, 5, (0,byte_value,0), -1)

            if self.first_run_calc_offset:
                line[5] = offset_to_line

        # Sort the obstacles by angle
        obstacles = sorted(obstacles, key=lambda obs: obs.theta)

        if debug:
            cv2.imshow("cast_obstacle_detector", image_debug)
            key = cv2.waitKey(1) & 0xFF
            #if key == ord("q"):
            #    break

        self.first_run_calc_offset = False

        return obstacles

    # Given a grayscale image, two line end points and a line angle
    # build a line using Bresenham algorithm
    # Image pixels added to line are then convolved to find gradient over the line
    # Pixels exceeding threshold in the line gradient are returned as obstacle locations
    def detect_collision_in_ray(self, image, theta, p1, p2, offset, invert_mask = False):
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
        print "LINE"
        print line_col

        # Convolve with filter to find image gradient over the line
    #    filter = [-1,-1,-1,-1,0,1,1,1,1]
    #    if invert_mask:
    #        filter = [1,1,1,1,0,-1,-1,-1,-1]
        filter = [1]
        line_grad =  np.convolve(line_col, filter, 'same')

        print "CONVOLVED"
        print line_grad

        for idx, val in enumerate(line_grad):
            # This portion of the image is obscured by the robot
            # Pick a point further away to start considering edges
    #        if theta > 3.49:
    #            if val > 100 and idx > 60 and idx < line_grad.size-10:
    #                return line_pos[idx]
    #        # The rest of the image has a tighter starting boundary
    #        else:

    #        if val > 100 and idx > (5 + offset) and idx < line_grad.size-10:
    #            return line_pos[idx]
            if val > 100 and idx > offset:# and idx < line_grad.size-10:
                return line_pos[idx]

        # Return the last value, indicating maximum range
        return line_pos[len(line_pos) - 1]

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
