#include "OfflineCamera.h"
#include "Angles.h"
#include "SingleConfig.h"
#include <cassert>

OfflineCamera::OfflineCamera(Position2dProxy &inPosition,
                             double inXOffset, double inYOffset)
    : position(inPosition), xOffset(inXOffset), yOffset(inYOffset)
{
    SingleConfig* config = SingleConfig::getInstance();
    imgSet = new ImgSet(config->getString("ImgSet.dirName"),
                        config->getString("ImgSet.prefix"));
}

OfflineCamera::~OfflineCamera() {
    delete imgSet;
}

void OfflineCamera::getImg( Img *&img ) {
    double x = position.GetXPos() + xOffset;
    double y = position.GetYPos() + yOffset;
    double th = position.GetYaw();
    
    bool gotImg = imgSet->getImg(x, y, th, img);
    assert(gotImg);
}
