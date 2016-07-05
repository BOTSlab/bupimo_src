/*
 * Combines weighted home directions into one home direction.
 *
 * Andrew Vardy
 */
#ifndef HOMEDIRCOMBINER_H
#define HOMEDIRCOMBINER_H

#include <cmath>
#include "Img.h"

class HomeDirCombiner {
public:
    HomeDirCombiner( int inWidth, int inHeight );
    float combine( Img* Alpha, Img *Weight );
private:
    int width, height;
};

#endif
