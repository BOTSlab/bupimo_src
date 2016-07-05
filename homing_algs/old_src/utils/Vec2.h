/*
 * Keep track of the magnitude and angle of a 2-element vector.
 *
 * Andrew Vardy
 */

#ifndef VEC2_H
#define VEC2_H

#include <string>
#include <sstream>
using namespace std;

class Vec2 {
public:
    Vec2( float inMag=0, float inAng=0 ) {
        mag = inMag;
        ang = inAng;
    }
    Vec2( const Vec2& other ) {
        mag = other.mag;
        ang = other.ang;
    }
    const Vec2& operator= (const Vec2& other) {
        if (this == &other)
            return *this;

        mag = other.mag;
        ang = other.ang;
        return *this;
    }
    string toString() {
        ostringstream oss;
        oss << "mag: " << mag << ", ang: " << ang;
        return oss.str();
    }
    float mag, ang;
};

#endif
