/**
 * Controller to test odometry-based homing.
 */

#ifndef TESTODO1CONTROLLER_H
#define TESTODO1CONTROLLER_H

#include "Controller.h"
#include "Odo1Controller.h"

class TestOdo1Controller : public Controller {
public:
    TestOdo1Controller(DriverAdaptor &inAdaptor);
    virtual ~TestOdo1Controller();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();

private:

    Odo1Controller odo1Controller;

    int index;
    double goalIX[5], goalIY[5];
};

#endif
