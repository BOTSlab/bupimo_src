/**
 * Controller is the base class for all possible robot controllers.
 *
 * @Andrew Vardy
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "DriverAdaptor.h"
#include <iostream>

class Controller {
public:
    Controller(DriverAdaptor &inAdaptor) : adaptor(inAdaptor) {};

    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual void fire() = 0;
    virtual string getDescription() = 0;
protected:
    DriverAdaptor &adaptor;
};

#endif
