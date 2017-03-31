#! /usr/bin/env python2

import cv2
import numpy as np

colors = []

def on_mouse_click (event, x, y, flags, hsv):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(hsv[y,x].tolist())
        print "HSV: " + str(hsv[y, x])

def process_image(image, wait):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if colors:
        cv2.putText(image, 'HSV: ' + str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
    cv2.imshow('image', image)
    cv2.setMouseCallback('image', on_mouse_click, hsv)

    if cv2.waitKey(wait) & 0xFF == ord('q'):
        return

def post_compute_stuff():
    if len(colors) == 0:
        return
    minh = min(c[0] for c in colors)
    mins = min(c[1] for c in colors)
    minv = min(c[2] for c in colors)
    maxh = max(c[0] for c in colors)
    maxs = max(c[1] for c in colors)
    maxv = max(c[2] for c in colors)

    print "minimums: " + str([minh,mins,minv])
    print "maximums: " + str([maxh,maxs,maxv])

def process_camera():
    capture = cv2.VideoCapture(0)

    while True:
        _, image = capture.read()
        process_image(image, 1)

    capture.release()
    cv2.destroyAllWindows()

    post_compute_stuff()

def process_file(filename):
    image = cv2.imread(filename)
    print "Hit 'q' to quit."
    process_image(image, 0)
    post_compute_stuff()

if __name__ == "__main__":

    #process_camera()

    #process_file('hsv-color-model.png')
    process_file('raspicam.jpg')
