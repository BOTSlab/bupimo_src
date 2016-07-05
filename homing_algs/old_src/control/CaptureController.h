/**
 * Controller which allows manual control over the robot while simultaneously
 * capturing images to disk.  A new image is captured after moving through a
 * displacment of CAPTURE_DISPLACEMENT.
 */

#ifndef CAPTURECONTROLLER_H
#define CAPTURECONTROLLER_H

#include "ManualController.h"
#include <fstream>

class CaptureController : public ManualController {
public:
    CaptureController(DriverAdaptor &inAdaptor);
    virtual ~CaptureController();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();

private:

    enum StateType { FIRST, CAPTURE, WAITING, MOVING };
    StateType state;

    int index;
    double displacementSinceCapture, lastX, lastY;
    ofstream odoFile;

    // Should we go into the WAITING state after capturing, or just switch
    // back to MOVING?
    const bool WAIT_AFTER_CAPTURE;

    // The displacement along the path required between capture positions.
    const double CAPTURE_DISPLACEMENT;
};

#endif
