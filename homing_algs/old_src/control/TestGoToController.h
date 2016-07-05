/**
 * Controller to test Player's GoTo driver.
 */

#ifndef TESTGOTOCONTROLLER_H
#define TESTGOTOCONTROLLER_H

#include "Controller.h"

class TestGoToController : public Controller {
public:
    TestGoToController(DriverAdaptor &inAdaptor);
    virtual ~TestGoToController();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();
};

#endif
