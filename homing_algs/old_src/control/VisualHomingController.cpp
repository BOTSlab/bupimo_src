#include "VisualHomingController.h"
#include "SingleConfig.h"
#include <cassert>

VisualHomingController::VisualHomingController(DriverAdaptor &inAdaptor) :
    Controller(inAdaptor),
    state(WAITING),
    homingAlg(NULL),
    S(NULL),
    C(NULL),
    saveIndex(0)
{
    SingleConfig* config = SingleConfig::getInstance();
    ODO_HOMING_DISTANCE = 
            config->getDouble("VisualHomingController.odoHomingDistance");
    WAIT_AFTER_HOMING =
            config->getBool("VisualHomingController.waitAfterHoming");
    STABILIZING_STEPS =
            config->getInt("VisualHomingController.stabilizingSteps");
    SIMILARITY_THRESHOLD =
            config->getDouble("VisualHomingController.similarityThreshold");
}

VisualHomingController::~VisualHomingController() {
}

void VisualHomingController::activate() {
}

void VisualHomingController::deactivate() {
}

void VisualHomingController::fire() {

    // We can capture a snapshot image in any state.
    int key = adaptor.getKey();
    if (key == 's') {
        adaptor.getImg(S);
        if (homingAlg == NULL)
            homingAlg = TotalHomingAlg::createFromConfig(S->getWidth(),
                                                         S->getHeight());
        adaptor.print("VisualHomingController: Snapshot captured.");
        homingAlg->snapshot(S);
        state = WAITING;
    }

    // Handle the effects of being in each state and possible changes in state.
    if (state == WAITING) {
        // We should wait for confirmation from the user before continuing (he
        // may need to place a marker at the capture site).
        if (key == 'c') {
            if (S == NULL) {
                adaptor.print("VisualHomingController: No snapshot image!");
            } else {
                adaptor.print("VisualHomingController: Homing...");
                state = COMPUTING_HOME;
            }
        }
    } else if (state == STABILIZING) {
        adaptor.print("STABILIZING");

        if (++stabilizingCount >= STABILIZING_STEPS)
            state = COMPUTING_HOME;

    } else if (state == COMPUTING_HOME) {
        adaptor.print("COMPUTING_HOME");

        double homeAngle, similarity;
        adaptor.getImg(C);
        homingAlg->getHomeAngle(C, homeAngle, similarity);

        if (similarity > SIMILARITY_THRESHOLD) {
            adaptor.print("\tVisualHomingController: Homing complete!");
            state = WAITING;
            adaptor.stop();
            return;
        } 

        // Select a goal for odometry-based homing.  'distanceToMove' is
        // required to define some goal.  However, it is essentially ignored
        // because we don't wait for the movement to be complete.
        double distanceToMove = 1000 * ODO_HOMING_DISTANCE;
        double beta = homeAngle + adaptor.getTheta();
        double x = adaptor.getX();
        double y = adaptor.getY();
        double goalOdoX = x + distanceToMove * cos(beta);
        double goalOdoY = y + distanceToMove * sin(beta);
        odoHomingStartX = x;
        odoHomingStartY = y;
        adaptor.goTo(goalOdoX, goalOdoY, 0);
        state = ODO_HOMING;

    } else if (state == ODO_HOMING) {
        //adaptor.print("ODO_HOMING");

        double dx = adaptor.getX() - odoHomingStartX;
        double dy = adaptor.getY() - odoHomingStartY;
        double distanceFromStart = sqrt(dx*dx + dy*dy);

        ostringstream oss;
        oss << "ODO_HOMING: distanceFromStart: " << distanceFromStart;
        adaptor.print(oss.str());

        if (distanceFromStart >= ODO_HOMING_DISTANCE) {
            adaptor.stop();
            if (WAIT_AFTER_HOMING) {
                adaptor.print("VisualHomingController: Press 'c' to continue.");
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

    } 
}

string VisualHomingController::getDescription() {
    return "VisualHomingController";
}
