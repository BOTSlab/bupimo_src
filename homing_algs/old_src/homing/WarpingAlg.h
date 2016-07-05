/*
 * Uses the warping method for visual homing.  The core code for Warping is in
 * the 'warping' subdirectory.
 *
 * Andrew Vardy
 */

#ifndef WARPINGALG_H
#define WARPINGALG_H

#include "TotalHomingAlg.h"
#include "Angles.h"
#include "warping/Warping.h"

typedef FloatBlockVector WarpImage;

class WarpingAlg : public TotalHomingAlg {
public:
    WarpingAlg( int inWidth, int inHeight );
    ~WarpingAlg();
    virtual void snapshot( Img* inSS );
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);
private:
    void extract( Img *img, WarpImage *warpImage );

    Img *SS;
    Warping warping;
    int width, height;
    WarpImage warpSS, warpCV;
};

#endif
