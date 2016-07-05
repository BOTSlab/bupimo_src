/*
 * Obtains images for the current camera position by using a stored set of
 * images manipulated through class ImgSet.  The position of the camera in
 * is given by the text file /tmp/truePosition.txt which is written to by a
 * hacked version of the MobileSim package.  This allows a
 * complete closed loop simulation, such that commands to the robot are
 * executed in MobileSim.  Then stored images from the robot's perspective in
 * are obtained by recalling the appropriate images from ImgSet.
 *
 * Requires:
 *  - A set of images to be generated, accessed through ImgSet.
 *  - The program 'HackedMobileSim' to be installed and running.
 *  HackedMobileSim will create /tmp/truePosition.lock and
 *  /tmp/truePosition.txt which are required to exist every time 'getImg' is
 *  called.
 *
 * Andrew Vardy
 */
#ifndef IMGSETCAMERA_H
#define IMGSETCAMERA_H

#include "Camera.h"
#include "ImgSet.h"

class ImgSetCamera : public Camera {
public:
    ImgSetCamera();
    virtual ~ImgSetCamera();
    virtual bool init();
    virtual void getImg(Img *&img);
private:

    ImgSet* imgSet;
};

#endif
