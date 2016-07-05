/**
 * Controller to implement incremental visual homing along a route.  A
 * RouteManager determines the current snapshot to home to, while
 * RouteHomingController carries out the movement by alternating between
 * computing new home vectors and acting upon them with odometry-based homing.
 */

#ifndef ROUTEHOMINGCONTROLLER_H
#define ROUTEHOMINGCONTROLLER_H

#include "Controller.h"
#include "MyManager.h"
#include "ImgWindow.h"
#include "rng.h"
#include <sstream>

class RouteHomingController : public Controller {
public:
    RouteHomingController(DriverAdaptor &inAdaptor);
    virtual ~RouteHomingController();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();

private:
    void saveCV();

    MyManager routeManager;

    // The controller's state alternates between STABILIZING, COMPUTING_HOME
    // and ODO_HOMING (and switches to DONE when the route is completed).  The
    // WAITING state is used only if WAIT_AFTER_HOMING is used to wait for
    // a prompt from the user before continuing.
    enum StateType { STABILIZING, COMPUTING_HOME, WAITING, ODO_HOMING, DONE };
    StateType state;

    double lastDistanceFromStart, odoHomingStartX, odoHomingStartY;
    int saveIndex, stabilizingCount, odoHomingStuckCount;

    // Used to generate random movements to get unstuck.
    RNG random;

    // Parameters...
    double ODO_HOMING_DISTANCE;
    bool WAIT_AFTER_HOMING;
    int ODO_HOMING_STUCK_COUNT, STABILIZING_STEPS;
};

#endif
