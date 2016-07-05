#include "ExactVectorMap.h"
#include "ImgWindow.h"

ExactVectorMap::ExactVectorMap( int inWidth, int inHeight ) :
    width(inWidth), height(inHeight),
    Valid(width, height),
    ThetaX(width, height),
    ThetaY(width, height),
    TanThetaY(width, height),
    Dx(NULL),
    Dy(NULL),
    temp1(NULL),
    temp2(NULL),
    temp3(NULL)
{
    delta = 2*M_PI / width;
    float gamma0 = delta * (height-1)/2.0f;
    for ( int j=0; j<height; j++ ) {
        float thetaY = gamma0 - delta*j;
        for ( int i=0; i<width; i++ ) {
            float thetaX = 2*M_PI - delta*i;

            if ( fabs(tan(thetaY)) < 0.01 ) {
                Valid.set(i, j, 0);
                ThetaX.set(i, j, 1);
                ThetaY.set(i, j, 1);
                TanThetaY.set(i, j, 1);
            } else {
                Valid.set(i, j, 1);
                ThetaX.set(i, j, thetaX);
                ThetaY.set(i, j, thetaY);
                TanThetaY.set(i, j, tan(thetaY));
            }
        }
    }
}

ExactVectorMap::~ExactVectorMap() {
    delete Dx;
    delete Dy;
    delete temp1;
    delete temp2;
    delete temp3;
}

void ExactVectorMap::map( Img* U, Img* V, Img* Mask, Img* Alpha ) {

    ImgOps::mult(U, delta, Dx);
    ImgOps::mult(V, delta, Dy);

    ImgOps::add(&ThetaY, Dy, temp1);
    ImgOps::tan(temp1, temp2);
    ImgOps::div(temp2, &TanThetaY, temp3);
    ImgOps::cos(Dx, temp2);
    ImgOps::sub(temp3, temp2, temp1);
    ImgOps::sin(Dx, temp2);
    ImgOps::atan2(temp2, temp1, temp3);
    ImgOps::sub(&ThetaX, temp3, temp2);

//    ImgOps::add(&temp2, M_PI, Alpha);
ImgOps::add(temp2, 0.0, Alpha);

    Mask->setAll(1);
    for ( int y=0; y<height; y++ ) {
        for ( int x=0; x<width; x++ ) {
            if ( Valid.get(x,y) == 0 || (U->get(x,y) == 0 && V->get(x,y) == 0) )
                Mask->set(x, y, 0);
        }
    }
}
