/**
 * Implements "Smooth Controller 1" as described in the second set of notes on
 * kinematics in COMP 4766/6778.  This controller moves the robot to the
 * position set through the 'setGoal' method (the goal angle is ignored).  The
 * controller is configured by parameters k_v and k_omega.  
 *
 * @author Andrew Vardy
 */

#ifndef ODO1CONTROLLER_H
#define ODO1CONTROLLER_H

#include "OdoHomingController.h"
#include "Angles.h"
#include <libplayerc++/playerc++.h>

class Odo1Controller : public OdoHomingController {
public:
    Odo1Controller(DriverAdaptor &inAdaptor);
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();
    virtual bool isStalled();
private:
    // Parameters.
    double k_v, k_omega;

    bool stalled;
    double lastGoalRX, lastGoalRY;
};

#endif
