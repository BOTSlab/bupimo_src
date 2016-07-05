/**
 * Controller to implement direct visual homing, with the user interactively
 * specifying when a snapshot should be captured (by pressing 's').  Homing
 * will being after the user presses 'c'.  If WAIT_AFTER_HOMING is true then
 * the robot waits for the user to press 'c' again after every movement vector.
 * If STABILIZING_STEPS is greater than 0 then the robot waits this number of
 * steps before computing a home vector (waiting for the mechanical
 * oscillations from the previous movement to die down).
 */

#ifndef VISUALHOMINGCONTROLLER_H
#define VISUALHOMINGCONTROLLER_H

#include "Controller.h"
#include "TotalHomingAlg.h"
#include <sstream>

class VisualHomingController : public Controller {
public:
    VisualHomingController(DriverAdaptor &inAdaptor);
    virtual ~VisualHomingController();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();

private:
    void saveCV();

    // The controller's state alternates between STABILIZING, COMPUTING_HOME
    // and ODO_HOMING (and switches to WAITING when homing is completed).
    enum StateType { WAITING, STABILIZING, COMPUTING_HOME, ODO_HOMING };
    StateType state;

    // The homing algorithm.
    TotalHomingAlg *homingAlg;

    // The snapshot and current images.
    Img *S, *C;

    double odoHomingStartX, odoHomingStartY;
    int saveIndex, stabilizingCount;

    // Parameters...
    double ODO_HOMING_DISTANCE;
    bool WAIT_AFTER_HOMING;
    int STABILIZING_STEPS;
    double SIMILARITY_THRESHOLD;
};

#endif
