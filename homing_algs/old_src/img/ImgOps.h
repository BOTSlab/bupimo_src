/*
 * Image processing operations.  All of these operations take one or two input
 * images as parameters, as well as other scalar parameters.  The final
 * parameter is a reference pointer to the output image.  If this is set to
 * NULL by the calling code then a new resultant image of the appropriate size
 * will be created and assigned to this parameter.
 *
 * \author Andrew Vardy
 */

#ifndef IMGOPS_H
#define IMGOPS_H

#include <iostream>
#include <cv.h>
#include "Img.h"
using namespace std;

class ImgOps {
public:

    // Rotate the image horizontally (shift in x-direction with circular
    // wrap-around).
    static void rotate(Img* A, int shift, Img*& B);

    // Convolve the image by kernels [-1 0 1] and [-1 0 1]^T.
    static void diffHorz(Img* A, Img*& B);
    static void diffVert(Img* A, Img*& B);

    static void smooth(Img* A, int radius, Img*& B);

    // Image arithmetic
    static void add(Img* A, float scalar, Img*& B);
    static void mult(Img* A, float scalar, Img*& B);
    static void add(Img* A, Img* B, Img*& C);
    static void sub(Img* A, Img* B, Img*& C);
    static void mult(Img* A, Img* B, Img*& C);
    static void div(Img* A, Img* B, Img*& C);
    static void comb( float ca, Img* A, float cb, Img* B, bool plus, Img*& C);

    // Logical operations
    static void logicAnd(Img* A, Img* B, Img*& C);
    static void logicOr(Img* A, Img* B, Img*& C);
    static void logicComb(Img* A, Img* B, bool andOp, Img*& C);

    // Trignometric and miscellenaous operations
    static float ssd(Img* A, Img*& B);
    static void sqdDiff(Img* A, Img* B, Img*& C);
    static void eqdWarp(Img* In, float alpha, float k, bool taylor);
    static void cos(Img* A, Img*& B);
    static void sin(Img* A, Img*& B);
    static void tan(Img* A, Img*& B);
    static void atan2(Img* A, Img* B, Img *&C);
    static void fabs(Img* A, Img*& B);

    // Various thresholding functions
    static void threshold(Img* In, float threshold, Img*& Out);
    static void negate(Img* In, Img*& Out);
    static void histThreshold(Img* In, int bins, float fraction, Img*& Out);
    static void adaptiveThreshold(Img* In, float percentile, Img*& Out);

    // Drawing
    static void drawVectorImage(Img* U, Img* V, int imageScale, int spacing,
                                float vectorScale, bool unit, Img*& Out);
    static void drawLine(Img *In, int x1, int y1, int x2, int y2);

    // Computing optic flow
    /*
    static void lucasKanade(Img &A, Img &B, Img *&U, Img *&V);
    static void hornSchunck(Img &A, Img &B, Img *&U, Img *&V);
    static void blockMatch(Img &A, Img &B, Img *&U, Img *&V);
    */

private:
    // Helper methods for computing optic flow
    static void preOpticFlow(Img &A, Img &B,
                             IplImage *&byteA, IplImage *&byteB);
    static void postOpticFlow(IplImage *byteA, IplImage *byteB);

    // General helper.  Creates image Out if it doesn't exist.  If it does
    // exist, makes sure it is the right size.
    static void setAndCheck(Img* In, Img*& Out);
};

#endif
