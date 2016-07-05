/*
 * Unfolds images taken from a hyperbolic catadiotric image system into
 * rectangular panoramic images.  Each row of the output image corresponds
 * to a ring of constant elevation above the horizon in the input image; the
 * vertical angle subtended by this ring is constant for all rows of the output
 * image.
 *
 * Andrew Vardy
 */
#ifndef UNFOLDER_H
#define UNFOLDER_H

#include "Img.h"

class Unfolder {
public:
    Unfolder();

    Img* unfold( Img& Input, Img*& Output, bool debug=false );

private:
    double interpolate( Img &Input, double x, double y );
    double gamma2phi( double gamma );
    int round( double x );

    // Position of the centre of the mirror in the image (centreX, centreY)
    // and the distance of the horizon in pixels from this centre position.
    int centreX, centreY, horizon;

    // Parameters of mirror.
    double a, b;

    // Number of rows of output image (number of columns determined below
    // to make sure pixels subtend equal angles in both horizontal and vertical
    // directions).
    int nRows;

    // Number of rows to trim from the bottom of the output image.  This
    // parameter was added to remove some of the bottom rows without affecting
    // the overall transformation.  The same effect could be achieved by
    // introducing a 'minGamma' parameter.
    int nTrim;

    // Maximum elevation above (and below) the horizon (in radians).
    double maxGamma;

    // Derived parameters
    int nRows_2, nCols;
    double a_sqd, b_sqd, c_sqd, c, f, phiHor, s;
};

#endif
