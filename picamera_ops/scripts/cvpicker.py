#! /usr/bin/env python2

import cv2, time
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

colors = []

def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())

def main():

    # Begin processing images
    with PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.ISO = 100
        camera.sa = 100
        camera.awb = "flash"
        camera.co = 100
        raw_capture = PiRGBArray(camera)
        time.sleep(5.0)
        for frame in camera.capture_continuous(raw_capture, \
                                format="bgr", use_video_port=True):
            image = frame.array

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HLS_FULL)
            if colors:
                cv2.putText(hsv, str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
            cv2.imshow('frame', hsv)
            cv2.setMouseCallback('frame', on_mouse_click, hsv)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

    # avgb = int(sum(c[0] for c in colors) / len(colors))
    # avgg = int(sum(c[0] for c in colors) / len(colors))
    # avgr = int(sum(c[0] for c in colors) / len(colors))
    # print avgb, avgg, avgr

    minb = min(c[0] for c in colors)
    ming = min(c[1] for c in colors)
    minr = min(c[2] for c in colors)
    maxb = max(c[0] for c in colors)
    maxg = max(c[1] for c in colors)
    maxr = max(c[2] for c in colors)
    print minr, ming, minb, maxr, maxg, maxb

    lb = [minb,ming,minr]
    ub = [maxb,maxg,maxr]
    print lb, ub

if __name__ == "__main__":
    main()
