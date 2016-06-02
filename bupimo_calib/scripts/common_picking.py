#!/usr/bin/env python
"""
Common utilities for picking points (i.e. annotating) input images.
"""

import pygame
import Image
import os
import csv

def drawMarker(screen, scaleFactor, pick):
    innerRadius = 5
    middleRadius = innerRadius + 1
    outerRadius = middleRadius + 1
    lineWidth = 2

    screenPick = scaleFactor * pick[0], scaleFactor * pick[1]

    innerRect = (screenPick[0]-innerRadius, screenPick[1]-innerRadius, \
                 2*innerRadius, 2*innerRadius)
    middleRect = (screenPick[0]-middleRadius, screenPick[1]-middleRadius, \
                 2*middleRadius, 2*middleRadius)
    outerRect = (screenPick[0]-outerRadius, screenPick[1]-outerRadius, \
                 2*outerRadius, 2*outerRadius)
    pygame.draw.rect(screen, (255, 255, 255), innerRect, lineWidth)
    pygame.draw.rect(screen, (0, 0, 0), middleRect, lineWidth)
    pygame.draw.rect(screen, (255, 255, 255), outerRect, lineWidth)

def pick(imageFile, labels):
    """Prompt the user to pick points on the given image.

    Given the image filename and a list of labels, prompt the user to pick as
many points as there are labels.  Return the list of picked points.  Right now,
picking is done by left-clicking and a limited undo functionality is supported
by right-clicking.
    """

    pygame.init()

    # Load the image.
    img = pygame.image.load(imageFile)
    imgSize = img.get_size()

    # Determine the largest integer scale factor so that the image will still
    # fit the screen.
    info = pygame.display.Info()
    scaleFactor = min(info.current_w / imgSize[0], \
                      info.current_h / imgSize[1])
    #print "scaleFactor: ", scaleFactor

    scaledSize = scaleFactor * imgSize[0], scaleFactor * imgSize[1]
    scaledImg = pygame.transform.scale(img, scaledSize)

    screen = pygame.display.set_mode(scaledSize)

    # For each click, we step through the labels.  index gives the current
    # index into labels.
    index = 0
    
    screen.blit(scaledImg, (0,0))

    pickList = []
    done = False
    while not done:
        # Indicate the current label in the window's title bar.
        pygame.display.set_caption(str(labels[index]))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # We are finished (or gave up?)
                done = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                (button1, button2, button3) = pygame.mouse.get_pressed()
                rawPick = pygame.mouse.get_pos()
                pick = rawPick[0] / scaleFactor, rawPick[1] / scaleFactor
                print pick
                if button1:
                    pickList.append(pick)
                    index += 1
                    # Was this the last one?  BAD: By quitting now we can't
                    # possibly undo this final pick.
                    if index == len(labels):
                        done = True
                elif button3:
                    # Undo the last pick
                    if len(pickList) > 0:
                        pickList.pop()
                        index -= 1

        screen.blit(scaledImg, (0,0))
        for pick in pickList:
            drawMarker(screen, scaleFactor, pick)
        pygame.display.flip()

    pygame.quit()

    return pickList

def autoPick(imageFile, color, expectedNumber):
    """
    Automatically pick red or green pucks from the image.  If the expected
    number is not found then display what blobs have been extracted.

    The given image file is loaded and color should either be 'red' or 'green'.
    """
    os.system('java -jar /home/av/callblobfinder.jar ' + imageFile + ' ' + color + ' /tmp/pickList.txt')

    # Read the position list created above.
    pickList = readPositionList('/tmp/pickList.txt')

    if len(pickList) == expectedNumber:
        return pickList

    #
    # Otherwise, there has been over- or under- detection.  Display
    # the extracted blobs...
    #

    pygame.init()
    img = pygame.image.load(imageFile)
    imgSize = img.get_size()
    scaleFactor = 1
    screen = pygame.display.set_mode(imgSize)
    pygame.display.set_caption("autoPick: Wrong number selected!")

    # Draw the picked items overlaid on the input image.
    screen.blit(img, (0,0))
    for pick in pickList:
        drawMarker(screen, scaleFactor, pick)
    pygame.display.flip()

    done = False
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

    return pickList

def readPositionList(filename):
    f = open(filename, 'rt')
    posList = []
    try:
        reader = csv.reader(f, delimiter=',')
        for row in reader:
            posList.append((float(row[0]), float(row[1])))
        return posList
    finally:
        f.close()

