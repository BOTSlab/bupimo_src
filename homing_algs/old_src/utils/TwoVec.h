/**
 * A 2-D vector class.
 *
 * \author Andrew Vardy
 */

#ifndef TWOVEC_H
#define TWOVEC_H

#include <iostream>
#include <cmath>
using namespace std;

class TwoVec {
public:
    /**
     * Constructor.
     *
     * \param inX the initial x-coordinate of this vector.
     * \param inY the initial y-coordinate of this vector.
     */
    TwoVec(double inX=0, double inY=0) {
        x = inX; y = inY;
    }

    /**
     * \return the length of this vector.
     */
    double length() {
        return sqrt(x*x + y*y);
    }

    /**
     * Normalizes the vector (divides each component by it length so that its
     * new length becomes 1).
     *
     * \post If the vector has non-zero length, it will be normalized.  If
     * the vector has zero length it will not be modified.
     */
    void normalize() {
        double norm = length();
        if ( norm != 0 ) {
            x /= norm;
            y /= norm;
        }
    }

    double dot(const TwoVec &other) {
        return x*other.x + y*other.y;
    }

    double angle() {
        return atan2(y, x);
    }

    double mag() {
        return sqrt(x*x + y*y);
    }

// operator overloading:

    bool operator==(const TwoVec &other) {
        return x == other.x && y == other.y;
    }

    TwoVec operator+(const TwoVec &rhs) {
        return TwoVec(x+rhs.x, y+rhs.y);
    }

    TwoVec operator-(const TwoVec &rhs) {
        return TwoVec(x-rhs.x, y-rhs.y);
    }

    friend TwoVec operator*(const TwoVec &lhs, double rhs) {
        return TwoVec(lhs.x*rhs, lhs.y*rhs);
    }

    friend TwoVec operator*(double lhs, const TwoVec &rhs) {
        return TwoVec(lhs*rhs.x, lhs*rhs.y);
    }

    friend TwoVec operator/(const TwoVec &lhs, double rhs) {
        return TwoVec(lhs.x/rhs, lhs.y/rhs);
    }

    friend ostream& operator<<(ostream &out, TwoVec &vec) {
        out << "[" << vec.x << ", " << vec.y << "]" << endl;
        return out;
    }

    /**
     * The two elements of the vector.  Made public for ease of access.
     */
    double x, y;
};

#endif
