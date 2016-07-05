/**
 * Obtains images for the current position as given by the Position2dProxy
 * reference passed to the constructor.  OfflineCamera needs to be instantiated
 * directly (i.e. not through Camera's 'getInstance' method).
 *
 * @author Andrew Vardy
 */

#ifndef OFFLINECAMERA_H
#define OFFLINECAMERA_H

#include "Camera.h"
#include "ImgSet.h"
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

class OfflineCamera : public Camera {
public:
    OfflineCamera(Position2dProxy &inPosition,
                  double inXOffset, double inYOffset);
    virtual ~OfflineCamera();
    virtual void getImg(Img *&img);
private:
    Position2dProxy &position;
    double xOffset, yOffset;

    ImgSet* imgSet;
};

#endif
