#include "RouteHomingController.h"
#include "SingleConfig.h"
#include <cassert>
#include <glog/logging.h>

RouteHomingController::RouteHomingController(DriverAdaptor &inAdaptor) :
    Controller(inAdaptor),
    routeManager(inAdaptor),
    state(COMPUTING_HOME),
    lastDistanceFromStart(0),
    saveIndex(0),
    random(0)
{
    SingleConfig* config = SingleConfig::getInstance();
    ODO_HOMING_DISTANCE = 
            config->getDouble("RouteHomingController.odoHomingDistance");
    ODO_HOMING_STUCK_COUNT = 
            config->getInt("RouteHomingController.odoHomingStuckCount");
    WAIT_AFTER_HOMING =
            config->getBool("RouteHomingController.waitAfterHoming");
    STABILIZING_STEPS =
            config->getInt("RouteHomingController.stabilizingSteps");
}

RouteHomingController::~RouteHomingController() {
}

void RouteHomingController::activate() {
}

void RouteHomingController::deactivate() {
}

void RouteHomingController::fire() {

    double x = adaptor.getX();
    double y = adaptor.getY();
    double theta = adaptor.getTheta();

    // Handle the effects of being in each state and possible changes in state.
    if (state == WAITING) {
        // We should wait for confirmation from the user before continuing (he
        // may need to place a marker at the capture site).
        int key = adaptor.getKey();
        if (key == 'c') {
            state = COMPUTING_HOME;
        }
    } else if (state == STABILIZING) {
        //adaptor.print("STABILIZING");

        if (++stabilizingCount >= STABILIZING_STEPS)
            state = COMPUTING_HOME;

    } else if (state == COMPUTING_HOME) {
        //adaptor.print("COMPUTING_HOME");

        double dx = x - odoHomingStartX;
        double dy = y - odoHomingStartY;
        double actualDistance = sqrt(dx*dx + dy*dy);
        bool arrived = false, routeComplete = false;
        LOG(INFO) << "actualDistance: " << actualDistance;

        // In case the 'getHomeAngle' computation takes a long time, tell
        // the robot to stop first.
        adaptor.stop();
        double homeAngle = routeManager.getHomeAngle(actualDistance, 
                                                     arrived, routeComplete);
        if (routeComplete) {
            state = DONE;
            adaptor.stop();
            return;
        } else if (arrived) {
            // No point trying to continue to move towards the previous
            // snapshot.  We return here so that when we come back into
            // fire we compute a new homeAngle.
            return;
        }

        // Select a goal for odometry-based homing.  'distanceToMove' is
        // required to define some goal.  However, it is essentially ignored
        // because we don't wait for the movement to be complete.
        double distanceToMove = 1000 * ODO_HOMING_DISTANCE;
        double beta = homeAngle + theta;
        double goalOdoX = x + distanceToMove * cos(beta);
        double goalOdoY = y + distanceToMove * sin(beta);
        odoHomingStartX = x;
        odoHomingStartY = y;
        odoHomingStuckCount = 0;
        adaptor.goTo(goalOdoX, goalOdoY, 0);
        state = ODO_HOMING;

    } else if (state == ODO_HOMING) {
        //adaptor.print("ODO_HOMING");

        double dx = x - odoHomingStartX;
        double dy = y - odoHomingStartY;
        double distanceFromStart = sqrt(dx*dx + dy*dy);

        bool stuck = (distanceFromStart - lastDistanceFromStart < 0.00001);
        lastDistanceFromStart = distanceFromStart;
        if (stuck)
            odoHomingStuckCount++;

        if (adaptor.isStalled())
            LOG(INFO) << "stalled!";
        LOG(INFO) << "distanceFromStart: " << distanceFromStart;

        if (odoHomingStuckCount > ODO_HOMING_STUCK_COUNT) {
            LOG(INFO) << "stuck: choosing random goal";

            // The local navigation algorithm seems to be stuck.  Choose a new
            // random goal location.
            double beta = theta + random.normal(0, M_PI/4.0);
            double goalOdoX = x + 1000 * cos(beta);
            double goalOdoY = y + 1000 * sin(beta);
            adaptor.goTo(goalOdoX, goalOdoY, 0);
            odoHomingStuckCount = 0;
        } else if (distanceFromStart >= ODO_HOMING_DISTANCE) {
            adaptor.stop();
            if (WAIT_AFTER_HOMING) {
                LOG(INFO) << "Waiting for user input";
                adaptor.print("RouteHomingController: Press 'c' to continue..");
                state = WAITING;
            } else {
                if (STABILIZING_STEPS > 0) {
                    state = STABILIZING;
                    stabilizingCount = 0;
                } else {
                    state = COMPUTING_HOME;
                }
            }
        }

    } else if (state == DONE) {
        // adaptor.print("\tDONE");
    }
}

string RouteHomingController::getDescription() {
    return "RouteHomingController";
}
