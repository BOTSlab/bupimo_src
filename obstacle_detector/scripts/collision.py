#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Daniel Cook <dac456@mun.ca>

from picamera.array import PiRGBArray
from picamera import PiCamera
from optparse import OptionParser
import time
import cv2
import cv
import numpy as np
import rospy
from obstacle_detector.msg import *
from bubblescope_property_service.srv import *

# TODO: these should come from the parameter server
img_width = 1296
img_height = 972

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
def detect_collision_in_ray(image, theta, p1, p2):
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
    if y < img_height:
        for x in range(p1c[0], p2c[0]-1):
            if x < img_width:
                line_pos.append(switch_from_octant_zero(theta, (x,y)))
                if line_pos[-1][1] < img_height and line_pos[-1][0] < img_width:
                    line_col.append(image[line_pos[-1][1]][line_pos[-1][0]])
                else:
                    line_col.append(0.0)

                if D >= 0:
                    y += 1
                    D -= dx
                D += dy

    # Convolve with filter to find image gradient over the line
    filter = [-1,-1,-1,-1,0,1,1,1,1]
    line_grad =  np.convolve(line_col, filter, 'same')

    for idx, val in enumerate(line_grad):
        # This portion of the image is obscured by the robot
        # Pick a point further away to start considering edges
        if theta > 3.49:
            if val > 100 and idx > 60 and idx < line_grad.size-10:
                return line_pos[idx]
        # The rest of the image has a tighter starting boundary
        else:
            if val > 100 and idx > 5 and idx < line_grad.size-10:
                return line_pos[idx]

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("-d", "--debug", dest="debug_on", help="enable debug output", default=False, action='store_true')
    (options, args) = parser.parse_args()

    rospy.init_node('obstacle_detector', anonymous=False)
    pub = rospy.Publisher('obstacle', ObstacleArray, queue_size=1)

    # Fetch bubblescope information from the service
    rospy.wait_for_service('get_bubblescope_properties')
    get_bubblescope_properties = rospy.ServiceProxy('get_bubblescope_properties', GetBubblescopeProperties)
    res = get_bubblescope_properties()

    if res is not None:
        x = int(res.center[0])
        y = int(res.center[1])
        inner_rad = res.inner_radius
        outer_rad = res.outer_radius

        # Lines start offset slightly from the inner circle
        # Assume endpoint is another offset from inner circle
        p1x = int(x + inner_rad)
        p1y = int(y)
        p2x = int(x + outer_rad)
        p2y = int(y)

        # Generate lines around our area of interest
        lines = []
        for theta in np.linspace(0.0, 6.28, 50, False):
            p1xr = int( np.cos(theta) * (p1x - x) - np.sin(theta) * (p1y - y) + x )
            p1yr = int( np.sin(theta) * (p1x - x) + np.cos(theta) * (p1y - y) + y )
            p2xr = int( np.cos(theta) * (p2x - x) - np.sin(theta) * (p2y - y) + x )
            p2yr = int( np.sin(theta) * (p2x - x) + np.cos(theta) * (p2y - y) + y )

            lines.append( (theta, p1xr, p1yr, p2xr, p2yr) )

        # Once we have the inner circle and a set of lines, we can start finding objects
        with PiCamera() as camera:
            camera.resolution = (img_width,img_height)
            camera.ISO = 100
            camera.sa = 100
            camera.awb = "flash"
            camera.co = 100

            raw_capture = PiRGBArray(camera)

            time.sleep(0.1)
    
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                image = frame.array
                image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                #image_edge = cv2.Canny(image,50,200)
                #kernel = np.ones((7,7),np.uint8)
                #image_edge = cv2.erode(image_edge,kernel,iterations = 1)

                if options.debug_on:
                    cv2.circle(image, (x,y), int(inner_rad), (0,255,0), 2)

                # Iterate over each line and detect edges
                # detect_collision_in_ray will return the first obstacle encountered on a line
                # The angle and distance from image centre are stored to describe the obstacle position (polar coords)
                # Image centre is also reported
                obstacles = []
                for line in lines:
                    collision_pos = detect_collision_in_ray(image_gray, line[0], (line[1],line[2]), (line[3],line[4]))

                    if collision_pos is not None:
                        obstacle = ObstacleLocation()
                        obstacle.centre = (x,y)
                        obstacle.theta = line[0]
                        obstacle.radius = np.sqrt( (collision_pos[0]-x)**2 + (collision_pos[1]-y)**2 )
                        obstacles.append(obstacle)

                        if options.debug_on:
                            cv2.circle(image, collision_pos, 6, (0,0,255), 1)

                # Publish array of detected obstacles for others to have fun with
                msg = ObstacleArray()
                msg.obstacles = obstacles
                pub.publish(msg)

                if options.debug_on:
                    cv2.imshow("DEBUG", image)
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("q"):
                        break

                raw_capture.truncate(0)
