/**
 * Base class for odometry-based homing methods which attempt to move the robot
 * to a particular goal pose by moving to minimize the position of the goal as
 * expressed in the robot reference frame.  The 'setGoal' method is used to
 * define the goal in the global reference frame.
 *
 * @author Andrew Vardy
 */

#ifndef ODOHOMINGCONTROLLER_H
#define ODOHOMINGCONTROLLER_H

#include "Controller.h"

class OdoHomingController : public Controller {
public:
    OdoHomingController(DriverAdaptor &inAdaptor) : Controller(inAdaptor) {};

    /**
     * Set the position of the goal in global coordinates.
     */
    virtual void setGoal( double gIX, double gIY, double gITh ) {
        goalIX = gIX;
        goalIY = gIY;
        goalITh = gITh;
    }

    virtual double distanceToGoal() {
        double dx = adaptor.getX() - goalIX;
        double dy = adaptor.getY() - goalIY;
        return sqrt(dx*dx + dy*dy);
    }

    /**
     * Indicate whether the robot appears to be stalled, meaning that the last
     * commanded speeds are not actually sufficient to move the robot.
     */
    virtual bool isStalled() = 0;

protected:
    // The pose of the goal in global coordinates.
    double goalIX, goalIY, goalITh;
};

#endif
