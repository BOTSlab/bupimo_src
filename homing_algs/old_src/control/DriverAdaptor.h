/**
 * DriverAdaptor provides an alternate interface to Driver for instances of
 * Controller.
 *
 * @Andrew Vardy
 */

#ifndef DRIVERADAPTOR_H
#define DRIVERADAPTOR_H

#include "CursesListener.h"
#include "Camera.h"
#include "Unfolder.h"
#include <libplayerc++/playerc++.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;
using namespace PlayerCc;

class DriverAdaptor {
//class DriverAdaptor : public ostringstream {
public:
    /**
     * Constructor.  TBD: Restrict so this can only be called by Driver.
     */
    DriverAdaptor(Position2dProxy *inPosition, 
                  CursesListener *inListener,
                  int *inKey, Camera *inCamera, bool inSimulator);

    virtual ~DriverAdaptor();

    /**
     * Return odometric estimate of x-position.
     */
    virtual double getX();

    /**
     * Return odometric estimate of y-position.
     */
    virtual double getY();

    /**
     * Return odometric estimate of orientation in radians.
     */
    virtual double getTheta();

    /**
     * Set the robot's forward and angular speeds (angular speed in rads/sec).
     */
    virtual void setSpeed(double v, double omega);

    /**
     * Command the robot to go to the given pose.  We assume the robot is
     * equipped with some kind of local planner that can actually execute this
     * movement, such as VFH+.
     */
    virtual void goTo(double x, double y, double theta);

    /**
     * Return true if the robot appears to be stalled.
     */
    virtual bool isStalled();

    /**
     * Stop the robot.
     */
    virtual void stop();

    /**
     * Print a message on top of the console screen.
     */
    virtual void printHeader(string msg);

    /**
     * Print a message to the console.
     */
    virtual void print(string msg);

    /**
     * Return the most recently pressed key.
     */
    virtual int getKey();

    /**
     * If img is NULL, allocate a new image than capture an image from the
     * camera and copy it into img.  Otherwise, just copy the camera image into
     * img.
     */
    virtual void getImg(Img *&img);

    /**
     * Capture a picture of the robot from the observation camera (or of the
     * Stage window if we are doing a simulation).
     */
    virtual void captureFromObservationCamera(string filename);

    /**
     * Overload << to allow a DriverAdaptor object to be treated like cout,
     * causing messages to be both displayed on the curses display as well as 
     * logged.
     *
     * NOTE: I found the code for this here (blame them):
     *
     *    http://www.daniweb.com/forums/thread48996.html
     */
/*
    template<class __input> inline
    DriverAdaptor& operator<<(const __input Val) {

        *((ostringstream*)this)<<Val;

        // Do the actual printing.
        print(str());
        return *this;
    }
*/

    ostringstream &oss() {
        return stringStream;
    }

    void printOss() {
        print(stringStream.str());
        stringStream.str("");
        stringStream.clear();
    }

private:
    Position2dProxy *position;
    CursesListener *listener;
    int *key;
    Camera *camera;
    bool simulator;

    Img *rawImg;
    Unfolder unfolder;

    ostringstream stringStream;
};

#endif
