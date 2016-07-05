/**
 * Captures images from a Point Grey Research camera using the FlyCapture
 * library.
 *
 * @author Andrew Vardy
 */
#ifndef FLYCAMERA_H
#define FLYCAMERA_H

#include "Camera.h"
#include "FlyCapture2.h"
#include <iostream>
//#include <fstream>
using namespace std;

class FlyCamera : public Camera {
public:
    FlyCamera();
    virtual ~FlyCamera();
    virtual void getImg(Img *&img);
private:
    void printError(FlyCapture2::Error error);
    bool init();

    FlyCapture2::Camera cam;
//    ofstream logFile;
};

#endif
