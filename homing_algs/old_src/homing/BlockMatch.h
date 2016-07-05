/*
 * Uses block matching to find correspondences between images.
 *
 * Andrew Vardy
 */
#ifndef BLOCKMATCH_H
#define BLOCKMATCH_H

#include "Img.h"

class BlockMatch {
public:
    BlockMatch( int inWidth, int inHeight );
    void correspond( Img* SS, Img* CV, Img* U, Img* V );
    double dissimilarity( Img* U, Img* V );
private:
    float ssd( Img* A, Img* B, int ax, int ay, int bx, int by );

    int width, height, blockRadius, shiftX, shiftY, searchRadius;
};

#endif
