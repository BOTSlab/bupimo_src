/*
 * Exact vector mapping technique.
 *
 * Andrew Vardy
 */
#ifndef EXACTVECTORMAP_H
#define EXACTVECTORMAP_H

#include <cmath>
#include "Vec2.h"
#include "ImgOps.h"

class ExactVectorMap {
public:
    ExactVectorMap( int inWidth, int inHeight );
    ~ExactVectorMap();
    void map( Img* U, Img* V, Img* Mask, Img* Alpha );
private:
    int width, height;
    Img Valid, ThetaX, ThetaY, TanThetaY;
    Img *Dx, *Dy, *temp1, *temp2, *temp3;
    float delta;
};

#endif
