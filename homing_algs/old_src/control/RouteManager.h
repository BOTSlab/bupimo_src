/**
 * A RouteManager selects the snapshot image to home to for
 * RouteHomingController.
 *
 * @Andrew Vardy
 */

#ifndef ROUTEMANAGER_H
#define ROUTEMANAGER_H

#include "DriverAdaptor.h"

class RouteManager {
public:
    /**
     * Compute and return the home angle.  Also, determine if the robot has
     * arrived at its current snapshot position and if the route is complete.
     */
    virtual double getHomeAngle(double displacement,
                                bool &arrived, bool &routeComplete) = 0;
};


#endif
