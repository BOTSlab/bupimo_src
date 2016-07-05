#include "BlockMatch.h"

BlockMatch::BlockMatch( int inWidth, int inHeight ) :
    width(inWidth),
    height(inHeight),
    blockRadius(3),
//    shiftX(4),
//    shiftY(4),
//    searchRadius(10)
    shiftX(2),
    shiftY(2),
    searchRadius(30)
{
}

//void BlockMatch::correspond( Img* SS, Img* CV, Img* U, Img* V ) {
//}

float BlockMatch::ssd( Img* A, Img* B, int ax, int ay, int bx, int by ) {
    int r = blockRadius;
    float ssd = 0;
    for ( int j=-r; j<=r; j++ )
        for ( int i=-r; i<=r; i++ ) {
            float a = A->getToroidal(ax + i, ay + j);
            float b = B->getToroidal(bx + i, by + j);
            float diff = a - b;
            ssd += diff * diff;
        }
    return ssd;
}

void BlockMatch::correspond( Img* SS, Img* CV, Img* U, Img* V ) {
    U->setAll(0);
    V->setAll(0);

    for ( int y=blockRadius; y<height-blockRadius; y+=shiftY )
        for ( int x=0; x<width; x+=shiftX ) {
            int bestI = 0;
            int bestJ = 0;

            // Count the SSD for 0 displacement as the current best
            float bestSSD = ssd(SS, CV, x, y, x, y);
            for ( int j=-searchRadius; j<=searchRadius; j++ ) {
                if ( y+j-blockRadius < 0 || y+j+blockRadius > height-1 )
                    continue;

                for ( int i=-searchRadius; i<=searchRadius; i++ ) {
                    if ( i != 0 || j != 0) {
                        float curSSD = ssd(SS, CV, x, y, x+i, y+j);
                        if ( curSSD < bestSSD ) {
                            bestSSD = curSSD;
                            bestI = i;
                            bestJ = j;
                        }
                    }
                }
            }

            U->set(x, y, bestI);
            V->set(x, y, bestJ);
        }
}

double BlockMatch::dissimilarity( Img* U, Img* V ) {
    double u, v, length, sum = 0;
    double maxLength = sqrt(2.0) * searchRadius;
    int n = 0;
    for ( int y=blockRadius; y<height-blockRadius; y+=shiftY )
        for ( int x=0; x<width; x+=shiftX ) {
            u = U->get(x, y);
            v = V->get(x, y);
            length = sqrt(u*u + v*v);
            sum += length;
            n++;
        }
    return sum / (n * maxLength);
}
