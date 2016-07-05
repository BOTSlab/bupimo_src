/**
 * Captures a camera image using OpenCV.
 *
 * Andrew Vardy
 */
#ifndef OCVCAMERA_H
#define OCVCAMERA_H

#include "Camera.h"
#include "cv.h"
#include "highgui.h"

class OCVCamera : public Camera {
public:
    OCVCamera();
    virtual ~OCVCamera();
    virtual void getImg(Img *&img);
private:
    CvCapture *capture;
    IplImage *greyIpl;
};

#endif
