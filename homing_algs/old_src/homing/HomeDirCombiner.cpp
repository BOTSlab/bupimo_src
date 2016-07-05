#include "HomeDirCombiner.h"

HomeDirCombiner::HomeDirCombiner( int inWidth, int inHeight ) :
    width(inWidth), height(inHeight)
{
}

float HomeDirCombiner::combine( Img* Alpha, Img *Weight ) {
    float hx = 0;
    float hy = 0;
    for ( int y=0; y<height; y++ ) {
        for ( int x=0; x<width; x++ ) {
            float w = Weight->get(x,y);
            float alpha = Alpha->get(x,y);
            float vx = cos(alpha);
            float vy = sin(alpha);
            if ( w != 0 && ( vx != 0 || vy !=0 ) ) {
                hx += w * vx;
                hy += w * vy;
            }
        }
    }

    return atan2(hy, hx);
}
