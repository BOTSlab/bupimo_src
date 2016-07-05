/**
 * Implements a proportional controller for angle, with a constant speed.
 *
 * @author Andrew Vardy
 */

#ifndef SIMPLEODOCONTROLLER_H
#define SIMPLEODOCONTROLLER_H

#include "OdoHomingController.h"
#include "Angles.h"
#include <libplayerc++/playerc++.h>

class SimpleOdoController : public OdoHomingController {
public:
    SimpleOdoController(DriverAdaptor &inAdaptor);
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
