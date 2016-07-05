/**
 * Provides the connection between the user (via keyboard), different robot
 * controllers (via instances of Controller), and the robot itself (via
 * Player).  Used by instantiating, calling 'addController' up to 10 times to
 * add up to 10 controllers, then calling 'loop'.
 *
 * On each iteration of 'loop' the sensors are read and the current active
 * Controller's 'fire' method is called.  At any moment, only one controller is
 * active.  Each controller is associated with a number 0-9 assigned according
 * to the order in which that Controller was added.  The user can switch the
 * active controller by pressing the appropriate key.  In addition, the user
 * can exit by pressing the escape or enter keys.
 *
 * Instances of Controller are intended to interact with Driver only through
 * DriverAdaptor.
 *
 * @author Andrew Vardy
 */

#ifndef DRIVER_H
#define DRIVER_H

#include "CursesListener.h"
#include "Camera.h"
#include "DriverAdaptor.h"
#include "Controller.h"
#include <iostream>
#include <libplayerc++/playerc++.h>

// Forward declaration of Controller and DriverAdaptor.  We use forward
// declaration as opposed to straightforward header inclusion so that these
// three classes can be independently compiled.
//class Controller;
//class DriverAdaptor;

using namespace std;
using namespace PlayerCc;

class Driver {
public:
    /**
     * Constructs an instance that expects to communicate with a local player
     * server (i.e. running on this machine).
     */
    Driver();

    /**
     * Destructor.  TBD: Nicely close connection with player.
     */
    virtual ~Driver();

    /**
     * Adds a Controller to the list of Controllers that can possibly be active.
     */
    void addController(Controller *controller);

    /**
     * The main control loop.
     */
    void loop();

    /**
     * Return a reference to the DriverAdaptor for this Driver.
     */
    DriverAdaptor &getDriverAdaptor();

private:
    ///
    /// Helper methods.
    ///
    void handleSimulatorFakes();

    // The robot client and proxies for all devices.
    PlayerClient robot;

    // Proxy for the robot base.  Used for both obtaining odometric position
    // and for setting velocity.
    Position2dProxy position;

    // Used to read keypresses from the user.  
    CursesListener listener;

    // The most recently pressed key.
    int key;

    // A camera will be instantiated depending on the argument to the
    // constructor.
    Camera *camera;

    // Whether we are connected to a simulator (true) or a real robot (false).
    bool simulator;

    // The simulator doesn't currently support the bumper device.  So we
    // instantiate this only if connected to the actual robot.
    BumperProxy *bumper;

    // Whether the bumper has been pressed.
    bool bumped;

    // Array to hold up to 10 controllers.  
    Controller *controllers[10];

    // Number of valid Controllers in the above array.
    int nControllers;

    // Currently active controller.
    Controller *activeController;

    DriverAdaptor *adaptor;
};

#endif
