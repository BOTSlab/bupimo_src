/*
 * The pose or (x, y, theta) position of an object point in the plane.
 *
 * Andrew Vardy
 */
#ifndef POSE_H
#define POSE_H

class Pose {
public:
    Pose( float inX=0, float inY=0, float inTheta=0 ) {
        x = inX;
        y = inY;
        theta = inTheta;
    }

    Pose( const Pose& other ) {
        x = other.x;
        y = other.y;
        theta = other.theta;
    }

    const Pose& operator= (const Pose& other) {
        if (this == &other)
            return *this;

        x = other.x;
        y = other.y;
        theta = other.theta;
        return *this;
    }
    
    static float distanceBetween(const Pose &a, const Pose &b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return sqrt(dx*dx + dy*dy);
    }

    float x, y, theta;
};

#endif
