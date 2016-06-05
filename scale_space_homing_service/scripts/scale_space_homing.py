#!/usr/bin/env python

import cv2
import time
import rospy
import picamera
import numpy as np

from math import atan2, sin, cos, degrees, pi
from picamera.array import PiRGBArray
from optparse import OptionParser
from bubblescope_property_service.srv import *

#Threshold for scale change to use a point for homing, if 0 any change in scale is used
scaleDiffThresh = 0

#The ratio between the first and second best match that must be used to consider a point for homing.
# match1.distance must be less than matchDistanceThresh*match2.distance
# so a lower value means better matches are used
matchDistanceThresh=.7

#TODO: Get these from the bubblescope property service
xRes = 1296
yRes = 972

#Threshold value used in SURF key point detection
surfHessianThresh = 600
surf = cv2.SURF(surfHessianThresh)

#Number of goal locations to allow for
# 8 represents the number of colors the Pixy camera can be configured to detect
nGoalLocations = 8
goalLocationInformation = [([0],[0])]*nGoalLocations

roiCenter = (0,0)

debug = False

mask = [0]

#define the thickness (in pixels) of the ring used for masking the above horizon KP search space
maskRingWidth = 80

#Amount of pixels to ignore on the outeer ring of the image
#This is to help not include the lens edge KPs in the images
outterLensBufferPixels = 10



def handle_get_bearing_for_goal(req):
    goalId = req.goalId
    if goalId in range(0,nGoalLocations):
        kpGoal, desGoal = goalLocationInformation[goalId]
        kpCurr, desCurr = get_kp_and_des_at_current_location()

        return findHomingAngle(kpCurr, desCurr, kpGoal, desGoal)

    #if goalId is outside range return 0, meaning straight ahead
    return 0

def handle_set_goal_location(req):
    goalId = req.goalId
    if goalId in range(0,nGoalLocations):        
        goalLocationInformation[goalId] = get_kp_and_des_at_current_location()

def get_kp_and_des_at_current_location():
    imageGoal = get_image()

    imgGray = cv2.cvtColor(imageGoal, cv2.COLOR_BGR2GRAY)
    return surf.detectAndCompute(imgGray, mask)

#TODO: this might be a bad idea, should maybe just instantiate the camera once
#if so i'm not sure how to ensure the connection is properly close...damn python
def get_image():
    with picamera.PiCamera as picamera:
        rawCapture = PiRGBArray(camera)
        camera.resolution = (xRes,yRes)
        time.sleep(1)

        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array

        return image

def generateMask(roiCenter, outterRadius):

    #Leave a small buffer of pixels around the edge of the lens so as to not include the lens itself anywhere
    outterRadius = outterRadius - outterLensBufferPixels
    innerRadius = outterRadius - maskRingWidth    

    mask = np.zeros((xRes,yRes), np.uint8)
    cv2.circle(mask, roiCenter, outterRadius,1,-1)
    cv2.circle(mask, roiCenter, innerRadius,0,-1)

    return mask

#based on the number and direction of the contraction and expansion angles, calculate the final weighted homing direction
def calcHomingAngle(contractionAngles, expansionAngles):
    sinSumContraction = 0
    cosSumContraction = 0
    sinSumExpansion = 0
    cosSumExpansion = 0

    for angle in contractionAngles:
        sinSumContraction = sinSumContraction + sin(angle)
        cosSumContraction = cosSumContraction + cos(angle)

    for angle in expansionAngles:
        sinSumExpansion = sinSumExpansion + sin(angle)
        cosSumExpansion = cosSumExpansion + cos(angle)

    avgContractionAngle = atan2(sinSumContraction,cosSumContraction)
    avgExpansionAngle = atan2(sinSumExpansion,cosSumExpansion)

    #logging.debug("average contraction Angle: "+str(degrees(avgContractionAngle)))
    #logging.debug("average expansion Angle: "+str(degrees(avgExpansionAngle)))

    sBar = len(expansionAngles)*sin(avgExpansionAngle) + len(contractionAngles)*(sin(avgContractionAngle + pi))
    cBar = len(expansionAngles)*cos(avgExpansionAngle) + len(contractionAngles)*(cos(avgContractionAngle + pi))

    homingAngle = atan2(sBar,cBar)
    return homingAngle

#First only take a match if it is significantly better than the second best match
#Then sort the matches into those where the scale has increased, and those where it has decreased
def sortMatches(knnmatches, kpCurr, kpGoal):

    global scaleDiffThresh
    global matchDistanceThresh

    contractionMatches = []
    expansionMatches = []
    contractionAngles = []
    expansionAngles = []

    for m,n in knnmatches:
    	#If the best match is closer (by the threshold ratio) than the second best match then use it
        if m.distance < matchDistanceThresh*n.distance:

            currkpCurr = kpCurr[m.queryIdx]
            currkpGoal = kpGoal[m.trainIdx]
            rads = atan2(roiCenter[1]-currkpCurr.pt[1],currkpCurr.pt[0]-roiCenter[0])
            if rads < 0:
                rads=rads + (2*pi)
            	#print "kpCurr sise",currkpCurr.size
            	#print "kpGoal size",currkpGoal.size
            if currkpCurr.size > (currkpGoal.size + scaleDiffThresh):
                contractionMatches.append(m)
                #print "contractionAngle: ",rads
                contractionAngles.append(rads)
            elif currkpCurr.size < (currkpGoal.size - scaleDiffThresh):
                expansionMatches.append(m)
                #print "expansionAngle: ",rads
                expansionAngles.append(rads)

    return contractionMatches, expansionMatches, contractionAngles, expansionAngles


#Calulate the homing angle based on the keypoints found in the current and goal images
def findHomingAngle(kpCurr, desCurr, kpGoal, desGoal):
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(desCurr,desGoal,k=2)

	contractionMatches, expansionMatches, contractionAngles, expansionAngles = sortMatches(matches, kpCurr, kpGoal)

    	if debug:
    		print "Total number of Matches: "+str(len(matches))
    		print "Total number of \'good\' Matches: "+str(len(contractionMatches)+len(expansionMatches))
    		print "contractionMatches size: " +str(len(contractionMatches))
    		print "expansionMatches size: "+str(len(expansionMatches))

	homingAngle = calcHomingAngle(contractionAngles, expansionAngles)

	
    	if debug:
        	homingAngleDegrees = degrees(homingAngle)
    		if homingAngleDegrees < 0:
        		homingAngleDegrees += 360
	   	print "Homing Direction (degrees): ",homingAngleDegrees

	return homingAngle


if __name__ == '__main__':
    #Check for debug 
    #parser = OptionParser()
    #parser.add_option("-d", "--debug", dest="debug_on", help="enable debug output", default=False, action='store_true')
    #(options, args) = parser.parse_args()
    #global debug
    #debug = debug_on

    rospy.init_node('scale_space_homing_service')
    s = rospy.Service('get_bearing_for_goal', GetBearingForGoal, handle_get_bearing_for_goal)
    t = rospy.Service('set_goal_location', SetGoalLocation, handle_set_goal_location)

    global roiCenter

    # Fetch bubblescope information from the service
    rospy.wait_for_service('get_bubblescope_properties')
    get_bubblescope_properties = rospy.ServiceProxy('get_bubblescope_properties', GetBubblescopeProperties)
    res = get_bubblescope_properties()

    

    if res is not None:
        roiCenter = (int(res.center[0]), int(res.center[1]))
        outterRad = res.outer_radius

        #Generate and save the mask to be applied when finding keypoints
        global mask
        mask = generateMask(roiCenter, outterRad)


    rospy.spin()
