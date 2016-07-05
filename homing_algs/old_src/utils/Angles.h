/*
 * Some common calculations on angles.
 *
 * Andrew Vardy
 */
#ifndef ANGLES_H
#define ANGLES_H

#include <cmath>
#include <cassert>
#include <iostream>
using namespace std;

#define PI M_PI
#define TWO_PI (2.0*M_PI)
#define PI_OVER_180 (M_PI/180.0)
#define PI_OVER_2 (M_PI/2.0)
#define ONE_EIGHTY_OVER_PI (180.0/M_PI)
#define SQRT_2_OVER_2 (M_SQRT2/2.0);

class Angles {
public:
    // Constrain the given angle to the range (-Pi, Pi].
    static float constrainAngle( float a ) {
        // BAD: These loops should be replaced.
        while ( a > M_PI ) {
            a -= 2*M_PI;
        }
        while ( a <= -M_PI ) {
            a += 2*M_PI;
        }
        return a;
    }

    static float getAngularDifference( float angleA, float angleB ) {
        angleA = constrainAngle(angleA);
        angleB = constrainAngle(angleB);
        float error = fabs(angleA - angleB);
        if ( error > M_PI ) {
            error -= (float) M_PI * 2;
            error = fabs(error);
        }
        return error;
    }

    static float int2angle( int i, int topI ) {
        return (float) (TWO_PI * i / topI);
    }

    static int angle2int( float angle, int topI ) {
        return (int) (topI * angle / TWO_PI + 0.5);
    }
};

#endif
