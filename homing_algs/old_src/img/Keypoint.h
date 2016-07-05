/**
 * Represents a SIFT keypoint (i.e. feature).  All attributes left public
 * for ease of use.
 *
 * @author Andrew Vardy
 */

#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <iostream>

class Keypoint {
public:
    /**
     * Construct a Keypoint.  The descriptor should be set later by calling
     * 'copyDescrip'.
     */
    Keypoint(double inX, double inY, double inSigma, double inAngle)
        : x(inX), y(inY), sigma(inSigma), angle(inAngle), match(NULL)
    {}

    /**
     * Copy constructor.
     */
    Keypoint(const Keypoint &other) {
        x = other.x;
        y = other.y;
        sigma = other.sigma;
        angle = other.angle;
        copyDescrip(other.descrip);
    }

    void copyDescrip(const unsigned char inDescrip[128]) {
        for (int i=0; i<128; i++)
            descrip[i] = inDescrip[i];
    }

    double x, y, sigma, angle;
    unsigned char descrip[128];

    Keypoint *match;
};

#endif
