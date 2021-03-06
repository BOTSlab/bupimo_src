#!/usr/bin/env python

import cv2
import numpy as np
#import bubblescope_analysis as bsa
import settings

from math import atan2, sin, cos, degrees, pi


#Threshold for scale change to use a point for homing, if 0 any change in scale is used
scaleDiffThresh = 0

#The ratio between the first and second best match that must be used to consider a point for homing.
# match1.distance must be less than matchDistanceThresh*match2.distance
# so a lower value means better matches are used
matchDistanceThresh=.7


#Threshold value used in SURF key point detection
surfHessianThresh = 600
surf = cv2.SURF(surfHessianThresh)

#Number of goal locations to allow for
# 8 represents the number of colors the Pixy camera can be configured to detect
nGoalLocations = 8
goalLocationInformation = [(([0],[0]),False)]*nGoalLocations


mask = np.zeros((0,0),np.uint8)

#define the thickness (in pixels) of the ring used for masking the above horizon KP search space
maskRingWidth = 60

#Amount of pixels to ignore on the outeer ring of the image
#This is to help not include the lens edge KPs in the images
outerLensBufferPixels = 5

def get_bearing_for_goal(image, goalId):
    bearing = 0
    isValid = False

    if goalId in range(0,nGoalLocations):

        #This will be true if we have taken keypoints at a location
        isValid = goalLocationInformation[goalId][1]


        kpGoal, desGoal = goalLocationInformation[goalId][0]
        if isValid and len(kpGoal) > 0:
            kpCurr, desCurr = get_kp_and_des_from_image(image)
            bearing, isValid  =  findHomingAngle(kpCurr, desCurr, kpGoal, desGoal, image)

    return bearing, isValid


def set_goal_location(image, goalId):
    if goalId in range(0,nGoalLocations):        
        goalLocationInformation[goalId] = (get_kp_and_des_from_image(image),True)

    #Just for testing, remove this
    if settings.debug:
        print "SET GOAL LOCATION DEBUG"
        kps,des = goalLocationInformation[goalId][0]
        imageColor = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for kp in kps:
            x, y = kp.pt
            cv2.circle(imageColor, (int(x),int(y)),2, (0,255,0))
 
        cv2.imwrite("test.jpg", imageColor)
        cv2.imshow("KPs", imageColor)
        key = cv2.waitKey(0) & 0xFF
        
        if key == ord("q"):
            cv2.destroyAllWindows()


def get_kp_and_des_from_image(image):
    return surf.detectAndCompute(image, mask)

def init(settings):
    """Generates the mask."""
    center = settings.roiCenter
    outerRadius = int(settings.outer_rad)

    #Leave a small buffer of pixels around the edge of the lens so as to not include the lens itself anywhere
    global mask

    outerRadius = outerRadius - outerLensBufferPixels
    innerRadius = outerRadius - maskRingWidth    

    mask = np.zeros((settings.yRes,settings.xRes), np.uint8)
    cv2.circle(mask, center, outerRadius,1,-1)
    cv2.circle(mask, center, innerRadius,0,-1)


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

    #totalContractionDist = 0
    #totalExpansionDist = 0

    for m,n in knnmatches:
    	#If the best match is closer (by the threshold ratio) than the second best match then use it
        if m.distance < matchDistanceThresh*n.distance:

            currkpCurr = kpCurr[m.queryIdx]
            currkpGoal = kpGoal[m.trainIdx]
            rads = atan2(settings.roiCenter[1]-currkpCurr.pt[1],currkpCurr.pt[0]-settings.roiCenter[0])
            if rads < 0:
                rads=rads + (2*pi)
            	#print "kpCurr sise",currkpCurr.size
            	#print "kpGoal size",currkpGoal.size
            if currkpCurr.size > (currkpGoal.size + scaleDiffThresh):
                contractionMatches.append(m)

    # Change this to the difference in size between the KPs and it should work
     #           totalContractionDist += m.distance 
                #print "contractionAngle: ",rads
                contractionAngles.append(rads)
            elif currkpCurr.size < (currkpGoal.size - scaleDiffThresh):
                expansionMatches.append(m)
     #           totalExpansionDist += m.distance
                #print "expansionAngle: ",rads
                expansionAngles.append(rads)


    #print "totalContractionDist: ",totalContractionDist
    #print "totalExpansionDist: ",totalExpansionDist

    #print "normalized contraction dist:",(totalContractionDist/len(contractionMatches))
    #print "normalized expansion dist:",(totalExpansionDist/len(expansionMatches))

    return contractionMatches, expansionMatches, contractionAngles, expansionAngles


#Calulate the homing angle based on the keypoints found in the current and goal images
def findHomingAngle(kpCurr, desCurr, kpGoal, desGoal, image):
	bf = cv2.BFMatcher()
#	matches = bf.knnMatch(desCurr,desGoal,k=2)
        if desCurr == None or desGoal == None:
            print "\n\n\n\nNONE"
#        if desCurr.empty() or desGoal.empty():
        if desCurr.size == 0 or desGoal.size == 0:
            print "\n\n\n\nEMPTY"
	matches = bf.knnMatch(desCurr,desGoal,2)

	contractionMatches, expansionMatches, contractionAngles, expansionAngles = sortMatches(matches, kpCurr, kpGoal)

    	if settings.debug:
            print "Total number of Matches: "+str(len(matches))
            print "Total number of \'good\' Matches: "+str(len(contractionMatches)+len(expansionMatches))
	    print "contractionMatches size: " +str(len(contractionMatches))
	    print "expansionMatches size: "+str(len(expansionMatches))

            imageColor = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for match in contractionMatches:
                kp = kpCurr[match.queryIdx]
                x,y = kp.pt
                cv2.circle(imageColor, (int(x),int(y)),2, (0,255,0))


            for match in expansionMatches:
                kp = kpCurr[match.queryIdx]
                x,y = kp.pt
                cv2.circle(imageColor, (int(x),int(y)),2, (0,0,255))

            cv2.imshow("Matchess", imageColor)
            key = cv2.waitKey(10) & 0xFF
        

        homingAngle = 0
        isValid = True

        if len(contractionMatches) == 0 and len(expansionMatches) == 0:
            isVlid = False
        else:
            homingAngle = calcHomingAngle(contractionAngles, expansionAngles)

    	    if settings.debug:
                homingAngleDegrees = degrees(homingAngle)
	        if homingAngleDegrees < 0:
       	            homingAngleDegrees += 360
   	        print "Homing Direction (degrees): ",homingAngleDegrees

	return homingAngle, isValid
