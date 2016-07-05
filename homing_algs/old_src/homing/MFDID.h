/*
 * Matched-Filter Descent in Image Distances.  As described in:
 *     R. Moeller, A. Vardy. "Local visual homing by matched-filter descent in
 *     image distances", Biological Cybernetics, 95:413-430, 2006.
 *
 * Andrew Vardy
 */
#ifndef MFDID_H
#define MFDID_H

#include <cmath>
#include "TransHomingAlg.h"
#include "ImgWindow.h"
#include "ImgOps.h"

class MFDID : public TransHomingAlg {
public:
    MFDID( int width, int height);
    ~MFDID() {};
    Vec2 currentView( Img* CV );

private:
    Img T_11, T_12, T_21, T_22, gradX, gradY, diff, temp1, temp2, temp3, Vx, Vy;
};

#endif
